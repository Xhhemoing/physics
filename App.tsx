import React, { useState, useRef, useEffect, useCallback } from 'react';
import { Activity, X, Calculator, Clock, Camera, Minus, Trash2, Settings } from 'lucide-react';
import SimulationCanvas, { CanvasRef } from './components/SimulationCanvas';
import PropertiesPanel from './components/PropertiesPanel';
import DataCharts from './components/DataCharts';
import { Header } from './components/Header';
import { Toolbar } from './components/Toolbar';
import { PhysicsEngine } from './services/physicsEngine';
import { BodyType, PhysicsBody, SimulationState, Vector2, ConstraintType, FieldType, PhysicsField, FieldShape, Constraint } from './types';
import { Vec2 } from './services/vectorMath';

// Initial State
const INITIAL_STATE: SimulationState = {
  canvasName: 'New Project',
  bodies: [],
  constraints: [],
  fields: [],
  time: 0,
  paused: true,
  gravity: { x: 0, y: 10 },
  enableCoulomb: false,
  enableUniversalGravity: false,
  enableAirResistance: false, 
  selectedBodyId: null,
  selectedFieldId: null,
  selectedConstraintId: null,
  camera: { x: 0, y: 0, zoom: 1 } 
};

interface ArcBuilder {
    phase: 1 | 2 | 3; 
    center: Vector2;
    radius: number;
    startAngle: number;
    endAngle: number;
}

interface RampBuilder {
    start: Vector2;
    isConveyor: boolean;
}

interface Snapshot {
    id: string;
    timestamp: number;
    data: string; // JSON string of state
    thumbnail?: string;
}

const App: React.FC = () => {
  const [state, setState] = useState<SimulationState>(INITIAL_STATE);
  const [dragMode, setDragMode] = useState<string>('select');
  const [arcBuilder, setArcBuilder] = useState<ArcBuilder | null>(null);
  const [rampBuilder, setRampBuilder] = useState<RampBuilder | null>(null);
  const [polyBuilder, setPolyBuilder] = useState<Vector2[]>([]);
  const [constraintBuilder, setConstraintBuilder] = useState<string | null>(null); 
  const [pinnedBodyId, setPinnedBodyId] = useState<string | null>(null);
  const [snapshots, setSnapshots] = useState<Snapshot[]>([]);
  const [showSnapshots, setShowSnapshots] = useState(false);
  const [showGlobalSettings, setShowGlobalSettings] = useState(false);
  const [combineSelection, setCombineSelection] = useState<string[]>([]);
  
  // Equation Builder State
  const [showEquationBuilder, setShowEquationBuilder] = useState(false);
  const [eqParams, setEqParams] = useState({ x: "100 * Math.cos(t)", y: "100 * Math.sin(t)", tStart: 0, tEnd: 6.28, steps: 100, isHollow: true });

  const engineRef = useRef(new PhysicsEngine());
  const canvasRef = useRef<CanvasRef>(null);
  const reqRef = useRef<number>(0);
  const lastTimeRef = useRef<number>(0);

  // Animation Loop
  const tick = useCallback((time: number) => {
    if (!lastTimeRef.current) lastTimeRef.current = time;
    const dt = Math.min((time - lastTimeRef.current) / 1000, 0.05); 
    lastTimeRef.current = time;

    if (!state.paused) {
      setState(prev => {
        const newBodies = engineRef.current.step(
            dt, 
            prev.bodies, 
            prev.constraints, 
            prev.fields, 
            prev.gravity, 
            prev.enableCoulomb,
            prev.enableUniversalGravity,
            prev.enableAirResistance
        );
        
        // Update Trails
        newBodies.forEach(b => {
            if (b.showTrajectory) {
                if (!b.trail) b.trail = [];
                const last = b.trail[b.trail.length - 1];
                if (!last || Vec2.magSq(Vec2.sub(last, b.position)) > 4) {
                    b.trail.push({ ...b.position });
                    if (b.trail.length > 500) b.trail.shift();
                }
            }
        });

        return {
          ...prev,
          bodies: newBodies,
          time: prev.time + dt
        };
      });
    }

    reqRef.current = requestAnimationFrame(tick);
  }, [state.paused]);

  useEffect(() => {
    reqRef.current = requestAnimationFrame(tick);
    return () => {
      if (reqRef.current) cancelAnimationFrame(reqRef.current);
    };
  }, [tick]);

  // Handle Cut Event from Canvas
  useEffect(() => {
      const handleCut = (e: any) => {
          const { p1, p2 } = e.detail;
          if (state.selectedBodyId) {
              const body = state.bodies.find(b => b.id === state.selectedBodyId);
              if (body) {
                  const parts = engineRef.current.splitBody(body, p1, p2);
                  if (parts) {
                      setState(s => ({
                          ...s,
                          bodies: [...s.bodies.filter(b => b.id !== body.id), ...parts],
                          selectedBodyId: null
                      }));
                  }
              }
          }
      };
      window.addEventListener('canvas-cut', handleCut);
      return () => window.removeEventListener('canvas-cut', handleCut);
  }, [state.selectedBodyId, state.bodies]);

  const handleTogglePause = () => setState(s => ({ ...s, paused: !s.paused }));
  const handleReset = () => setState({ ...INITIAL_STATE, camera: state.camera, canvasName: state.canvasName });
  
  const handleSelectBody = (id: string | null) => {
      // Constraint Construction Logic
      if (['add_spring', 'add_rod', 'add_pin', 'add_rope'].includes(dragMode) && id) {
          if (!constraintBuilder) {
              setConstraintBuilder(id);
          } else if (constraintBuilder !== id) {
              const bodyA = state.bodies.find(b => b.id === constraintBuilder);
              const bodyB = state.bodies.find(b => b.id === id);
              if (bodyA && bodyB) {
                  const newConstraint: Constraint = {
                      id: `c_${Date.now()}`,
                      type: dragMode === 'add_spring' ? ConstraintType.SPRING 
                          : dragMode === 'add_rod' ? ConstraintType.ROD 
                          : dragMode === 'add_rope' ? ConstraintType.ROPE
                          : ConstraintType.PIN,
                      bodyAId: constraintBuilder,
                      bodyBId: id,
                      localAnchorA: { x: 0, y: 0 }, 
                      localAnchorB: { x: 0, y: 0 },
                      length: Vec2.dist(bodyA.position, bodyB.position),
                      stiffness: 0.5,
                      damping: 0.1
                  };
                  setState(s => ({ ...s, constraints: [...s.constraints, newConstraint] }));
              }
              setConstraintBuilder(null);
              setDragMode('select');
          }
          return;
      }
      
      // Combine Tool Selection Logic
      if (dragMode === 'tool_combine' && id) {
          if (!combineSelection.includes(id)) {
              setCombineSelection([...combineSelection, id]);
          }
          if (combineSelection.length >= 1 && !combineSelection.includes(id)) {
              const firstId = combineSelection[0];
              const secondId = id;
              handleCombineBodies(firstId, secondId);
              setCombineSelection([]);
              setDragMode('select');
          }
          return;
      }

      if (dragMode === 'tool_cut' && id) {
          setState(s => ({ ...s, selectedBodyId: id }));
          return;
      }

      if (dragMode === 'tool_traj' && id) {
          setState(s => ({
              ...s,
              bodies: s.bodies.map(b => b.id === id ? { ...b, showTrajectory: !b.showTrajectory } : b)
          }));
          return;
      }

      setState(s => ({ ...s, selectedBodyId: id, selectedFieldId: null, selectedConstraintId: null }));
  };

  const handleCombineBodies = (id1: string, id2: string) => {
      // ... (Implementation same as previous)
      const b1 = state.bodies.find(b => b.id === id1);
      const b2 = state.bodies.find(b => b.id === id2);
      if (!b1 || !b2) return;
      
      const getVerts = (b: PhysicsBody) => {
          if (b.type === BodyType.CIRCLE) {
               const steps = 16;
               const vs = [];
               for(let i=0; i<steps; i++) {
                   const a = (i/steps) * Math.PI*2;
                   vs.push({ x: (b.radius||20)*Math.cos(a), y: (b.radius||20)*Math.sin(a) });
               }
               return vs.map(v => Vec2.transform(v, b.position, b.angle));
          } else if (b.vertices) {
              return b.vertices.map(v => Vec2.transform(v, b.position, b.angle));
          } else if (b.type === BodyType.BOX) {
              const w = (b.width||40)/2; const h = (b.height||40)/2;
              return [{x:-w, y:-h}, {x:w, y:-h}, {x:w, y:h}, {x:-w, y:h}].map(v => Vec2.transform(v, b.position, b.angle));
          }
          return [b.position];
      };

      const allVerts = [...getVerts(b1), ...getVerts(b2)];
      const hull = Vec2.convexHull(allVerts);
      
      let cx = 0, cy = 0;
      hull.forEach(v => { cx += v.x; cy += v.y; });
      cx /= hull.length; cy /= hull.length;
      const center = { x: cx, y: cy };
      
      const localVerts = hull.map(v => ({ x: v.x - center.x, y: v.y - center.y }));
      
      const newBody: PhysicsBody = {
          id: `combined_${Date.now()}`,
          type: BodyType.POLYGON,
          position: center,
          velocity: Vec2.div(Vec2.add(b1.velocity, b2.velocity), 2),
          acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
          mass: b1.mass + b2.mass,
          inverseMass: 1 / (b1.mass + b2.mass || 1),
          restitution: (b1.restitution + b2.restitution)/2,
          friction: (b1.friction + b2.friction)/2,
          charge: b1.charge + b2.charge,
          angle: 0, angularVelocity: 0, momentInertia: 100, inverseInertia: 0.01,
          vertices: localVerts,
          color: b1.color,
          selected: true, showTrajectory: false, trail: []
      };

      setState(s => ({
          ...s,
          bodies: [...s.bodies.filter(b => b.id !== id1 && b.id !== id2), newBody],
          constraints: s.constraints.filter(c => c.bodyAId !== id1 && c.bodyAId !== id2 && c.bodyBId !== id1 && c.bodyBId !== id2),
          selectedBodyId: newBody.id
      }));
  };

  const handleSelectField = (id: string | null) => {
      setState(s => ({ ...s, selectedFieldId: id, selectedBodyId: null, selectedConstraintId: null }));
  };
  
  const handleSelectConstraint = (id: string | null) => {
      setState(s => ({ ...s, selectedConstraintId: id, selectedBodyId: null, selectedFieldId: null }));
  };

  const handleZoom = (deltaY: number, mouseX: number, mouseY: number) => {
      setState(s => {
          const scale = deltaY > 0 ? 0.9 : 1.1;
          const newZoom = Math.max(0.01, Math.min(20, s.camera.zoom * scale)); 
          const worldMouseX = (mouseX - s.camera.x) / s.camera.zoom;
          const worldMouseY = (mouseY - s.camera.y) / s.camera.zoom;
          const newCamX = mouseX - worldMouseX * newZoom;
          const newCamY = mouseY - worldMouseY * newZoom;
          return { ...s, camera: { x: newCamX, y: newCamY, zoom: newZoom } };
      });
  };

  const handleBodyMove = (id: string, pos: Vector2) => {
      setState(s => ({
          ...s,
          bodies: s.bodies.map(b => b.id === id ? { ...b, position: pos, velocity: { x: 0, y: 0 }, angularVelocity: 0, trail: [] } : b)
      }));
  };

  const handleVectorEdit = (id: string, type: 'velocity' | 'force', vector: Vector2) => {
      setState(s => ({
          ...s,
          bodies: s.bodies.map(b => b.id === id ? (type === 'velocity' ? { ...b, velocity: vector } : { ...b, constantForce: vector }) : b)
      }));
  };

  const handleAddField = (pos: Vector2, shape: FieldShape, vertices?: Vector2[]) => {
      const newField: PhysicsField = {
          id: `field_${Date.now()}`,
          type: FieldType.UNIFORM_ELECTRIC, // Default
          shape: shape,
          position: pos,
          size: vertices && vertices.length === 1 ? vertices[0] : {x: 200, y: 200}, 
          radius: vertices && vertices.length === 1 ? vertices[0].x : 100, 
          vertices: shape === FieldShape.POLYGON ? vertices : undefined,
          strength: { x: 10, y: 0 },
          equations: { ex: "Math.sin(x/50) * 50", ey: "0" }, 
          visible: true
      };

      setState(s => ({
          ...s,
          fields: [newField, ...s.fields], 
          selectedFieldId: newField.id,
          selectedBodyId: null,
          selectedConstraintId: null
      }));
      setDragMode('select');
  };

  const handleAddBody = (pos: Vector2) => {
    if (dragMode === 'add_ramp' || dragMode === 'add_conveyor') {
        if (!rampBuilder) {
            setRampBuilder({ start: pos, isConveyor: dragMode === 'add_conveyor' });
        } else {
            const start = rampBuilder.start; const end = pos; 
            const center = Vec2.mul(Vec2.add(start, end), 0.5);
            const delta = Vec2.sub(end, start);
            const length = Vec2.mag(delta);
            const angle = Math.atan2(delta.y, delta.x);

            const newBody: PhysicsBody = {
                id: `ramp_${Date.now()}`,
                type: BodyType.BOX,
                position: center,
                velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
                mass: 0, inverseMass: 0, restitution: 1.0, friction: 0.5, charge: 0,
                angle: angle, angularVelocity: 0, momentInertia: 0, inverseInertia: 0,
                width: length,
                height: 5, 
                color: rampBuilder.isConveyor ? '#8b5cf6' : '#94a3b8', 
                selected: true, showTrajectory: false, trail: [],
                showCharge: true
            };
            if (rampBuilder.isConveyor) { newBody.surfaceSpeed = 50; newBody.height = 10; }
            setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
            setRampBuilder(null); setDragMode('select');
        }
        return;
    }

    if (dragMode === 'add_poly') {
        if (polyBuilder.length > 0 && Vec2.dist(pos, polyBuilder[0]) < 20) {
             if (polyBuilder.length < 3) return; 
             let cx = 0, cy = 0; polyBuilder.forEach(v => { cx += v.x; cy += v.y; });
             cx /= polyBuilder.length; cy /= polyBuilder.length;
             const center = { x: cx, y: cy };
             const localVerts = polyBuilder.map(v => ({ x: v.x - center.x, y: v.y - center.y }));
             const newBody: PhysicsBody = {
                 id: `poly_${Date.now()}`,
                 type: BodyType.POLYGON,
                 position: center,
                 velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
                 mass: 5, inverseMass: 0.2, restitution: 0.5, friction: 0.5, charge: 0,
                 angle: 0, angularVelocity: 0, momentInertia: 100, inverseInertia: 0.01,
                 vertices: localVerts, color: '#14b8a6', selected: true, showTrajectory: false, trail: []
             };
             setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
             setPolyBuilder([]); setDragMode('select');
        } else { setPolyBuilder([...polyBuilder, pos]); }
        return;
    }

    if (dragMode === 'add_arc') {
        if (!arcBuilder) {
            setArcBuilder({ phase: 1, center: pos, radius: 0, startAngle: 0, endAngle: 0 });
        } else if (arcBuilder.phase === 1) {
            const r = Vec2.dist(arcBuilder.center, pos);
            const angle = Math.atan2(pos.y - arcBuilder.center.y, pos.x - arcBuilder.center.x);
            setArcBuilder({ ...arcBuilder, phase: 2, radius: r, startAngle: angle, endAngle: 0 });
        } else if (arcBuilder.phase === 2) {
             const angle = Math.atan2(pos.y - arcBuilder.center.y, pos.x - arcBuilder.center.x);
             const r = Vec2.dist(arcBuilder.center, pos);
             setArcBuilder({ ...arcBuilder, phase: 3, radius: r, startAngle: angle, endAngle: 0 });
        } else if (arcBuilder.phase === 3) {
            let endAngle = Math.atan2(pos.y - arcBuilder.center.y, pos.x - arcBuilder.center.x);
            const startPoint = { x: arcBuilder.center.x + Math.cos(arcBuilder.startAngle)*arcBuilder.radius, y: arcBuilder.center.y + Math.sin(arcBuilder.startAngle)*arcBuilder.radius };
            if (Vec2.dist(pos, startPoint) < 20) { endAngle = arcBuilder.startAngle + 2 * Math.PI - 0.01; } 
            
            const newBody: PhysicsBody = {
                id: `arc_${Date.now()}`,
                type: BodyType.ARC,
                position: arcBuilder.center,
                velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
                mass: 0, inverseMass: 0, restitution: 1.0, friction: 0.5, charge: 0,
                angle: 0, angularVelocity: 0, momentInertia: 0, inverseInertia: 0,
                radius: arcBuilder.radius, arcStartAngle: arcBuilder.startAngle, arcEndAngle: endAngle,
                color: '#e2e8f0', selected: true, showTrajectory: false, trail: [], showCharge: true
            };
            setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
            setArcBuilder(null); setDragMode('select');
        }
        return;
    }

    if (dragMode === 'add_spring' || dragMode === 'add_rod' || dragMode === 'add_pin' || dragMode === 'add_rope') return;
    if (dragMode.startsWith('tool_') || dragMode.startsWith('add_field_') || dragMode === 'select_nodrag') return;

    const newId = `body_${Date.now()}`;
    let newBody: PhysicsBody = {
        id: newId,
        type: BodyType.CIRCLE,
        position: pos,
        velocity: {x: 0, y: 0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
        mass: 5, inverseMass: 0.2, restitution: 1.0, friction: 0.4, charge: 0,
        angle: 0, angularVelocity: 0, momentInertia: 50, inverseInertia: 0.02,
        color: '#ec4899', selected: true, showTrajectory: false, trail: [],
        showVelocity: false, showAcceleration: false, showForce: false, showCharge: true
    };

    if (dragMode === 'add_box') {
        newBody.type = BodyType.BOX; newBody.width = 40; newBody.height = 40; newBody.color = '#3b82f6';
    } else if (dragMode === 'add_particle') {
        newBody.isParticle = true; newBody.radius = 5; newBody.mass = 1; newBody.inverseMass = 1; newBody.color = '#facc15';
    } else { newBody.radius = 20; }
    
    setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newId }));
    setDragMode('select'); 
  };

  const handleUpdateBody = (id: string, updates: Partial<PhysicsBody>) => {
    setState(s => ({ ...s, bodies: s.bodies.map(b => b.id === id ? { ...b, ...updates } : b) }));
  };

  const handleUpdateField = (id: string, updates: Partial<PhysicsField>) => {
      setState(s => ({ ...s, fields: s.fields.map(f => f.id === id ? { ...f, ...updates } : f) }));
  };
  
  const handleUpdateConstraint = (id: string, updates: Partial<Constraint>) => {
      setState(s => ({ ...s, constraints: s.constraints.map(c => c.id === id ? { ...c, ...updates } : c) }));
  };

  const handleDeleteBody = (id: string) => {
    setState(s => ({ ...s, bodies: s.bodies.filter(b => b.id !== id), constraints: s.constraints.filter(c => c.bodyAId !== id && c.bodyBId !== id), selectedBodyId: s.selectedBodyId === id ? null : s.selectedBodyId }));
    if (pinnedBodyId === id) setPinnedBodyId(null);
  };

  const handleDeleteField = (id: string) => {
      setState(s => ({ ...s, fields: s.fields.filter(f => f.id !== id), selectedFieldId: s.selectedFieldId === id ? null : s.selectedFieldId }));
  };
  
  const handleDeleteConstraint = (id: string) => {
      setState(s => ({ ...s, constraints: s.constraints.filter(c => c.id !== id), selectedConstraintId: s.selectedConstraintId === id ? null : s.selectedConstraintId }));
  };

  const handleSaveScene = () => {
      const exportData = {
          state,
          snapshots,
          version: "2.0"
      };
      const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(exportData));
      const downloadAnchorNode = document.createElement('a');
      downloadAnchorNode.setAttribute("href", dataStr);
      downloadAnchorNode.setAttribute("download", `${state.canvasName || 'physlab_scene'}.json`);
      document.body.appendChild(downloadAnchorNode);
      downloadAnchorNode.click();
      downloadAnchorNode.remove();
  };

  const handleImportScene = (event: React.ChangeEvent<HTMLInputElement>) => {
      const fileReader = new FileReader();
      if (event.target.files && event.target.files.length > 0) {
          fileReader.readAsText(event.target.files[0], "UTF-8");
          fileReader.onload = e => {
              if (e.target?.result) {
                  try {
                      const data = JSON.parse(e.target.result as string);
                      // Support both new (wrapped) and old (flat) formats
                      const newState = data.state || data;
                      if (newState.bodies && newState.camera) { 
                          setState({
                              ...INITIAL_STATE,
                              ...newState,
                              paused: true 
                          });
                          if (data.snapshots) setSnapshots(data.snapshots);
                      } else { 
                          alert("无效的场景文件"); 
                      }
                  } catch (err) { alert("JSON 解析错误"); }
              }
          };
          event.target.value = '';
      }
  };

  const createSnapshot = () => {
      const snap: Snapshot = { id: `snap_${Date.now()}`, timestamp: state.time, data: JSON.stringify(state) };
      setSnapshots([snap, ...snapshots]);
  };

  const restoreSnapshot = (snap: Snapshot) => {
      try { const loaded = JSON.parse(snap.data); setState({ ...loaded, paused: true }); } catch (e) { console.error(e); }
  };

  const setPreset = (type: 'smooth' | 'elastic') => {
      setState(s => ({ ...s, bodies: s.bodies.map(b => { 
          if (type === 'smooth') return { ...b, friction: 0 }; 
          if (type === 'elastic') return { ...b, restitution: 1.0 }; 
          return b; 
      }) }));
  };

  const clearAll = () => {
      if (confirm("确定要清空所有对象吗?")) { setState({ ...INITIAL_STATE, camera: state.camera, paused: true }); setPinnedBodyId(null); setSnapshots([]); }
  };
  
  const handleGenerateEquationBody = () => {
      try {
          const fx = new Function('t', `with(Math){ return ${eqParams.x} }`);
          const fy = new Function('t', `with(Math){ return ${eqParams.y} }`);
          const points: Vector2[] = [];
          const step = (eqParams.tEnd - eqParams.tStart) / Math.max(3, eqParams.steps);
          
          for(let t = eqParams.tStart; t <= eqParams.tEnd; t += step) {
              points.push({ x: fx(t), y: fy(t) });
          }
          
          let cx = 0, cy = 0; points.forEach(p => { cx += p.x; cy += p.y; });
          cx /= points.length; cy /= points.length;
          const center = {x: cx, y: cy};
          const localVerts = points.map(p => ({ x: p.x - center.x, y: p.y - center.y }));

          const mass = eqParams.isHollow ? 0 : 5;
          const invMass = mass === 0 ? 0 : 1/mass;

          const newBody: PhysicsBody = {
             id: `eq_poly_${Date.now()}`,
             type: BodyType.POLYGON,
             position: center,
             velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
             mass: mass, inverseMass: invMass, restitution: 0.5, friction: 0.5, charge: 0,
             angle: 0, angularVelocity: 0, momentInertia: 100, inverseInertia: 0.01,
             vertices: localVerts, color: eqParams.isHollow ? '#f59e0b' : '#14b8a6', 
             selected: true, showTrajectory: false, trail: [],
             isHollow: eqParams.isHollow
          };
          setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
          setShowEquationBuilder(false);
      } catch (e) {
          alert("方程错误: " + e);
      }
  };

  const formatTime = (t: number) => {
      const min = Math.floor(t / 60); const sec = Math.floor(t % 60); const ms = Math.floor((t * 100) % 100);
      return `${min.toString().padStart(2, '0')}:${sec.toString().padStart(2, '0')}:${ms.toString().padStart(2, '0')}`;
  };

  return (
    <div className="w-full h-screen flex flex-col bg-slate-950 text-slate-200 overflow-hidden font-sans">
      <Header 
          state={state}
          onTogglePause={handleTogglePause}
          onReset={handleReset}
          onClearAll={clearAll}
          onPreset={setPreset}
          onSaveScene={handleSaveScene}
          onImportScene={handleImportScene}
          canvasRef={canvasRef}
          showSnapshots={showSnapshots}
          setShowSnapshots={setShowSnapshots}
          showSettings={showGlobalSettings}
          setShowSettings={setShowGlobalSettings}
          onChangeName={(name) => setState(s => ({...s, canvasName: name}))}
      />

      <div className="flex-1 flex overflow-hidden relative">
          
          <Toolbar 
              dragMode={dragMode}
              setDragMode={setDragMode}
              setPolyBuilder={setPolyBuilder}
              setArcBuilder={setArcBuilder}
              setRampBuilder={setRampBuilder}
              setCombineSelection={setCombineSelection}
              setShowEquationBuilder={setShowEquationBuilder}
          />

          <main className="flex-1 relative bg-slate-950">
             <SimulationCanvas 
                ref={canvasRef}
                state={state}
                onSelectBody={handleSelectBody}
                onSelectField={handleSelectField}
                onSelectConstraint={handleSelectConstraint}
                onAddBody={handleAddBody}
                onAddField={handleAddField}
                onMoveBody={handleBodyMove}
                onVectorEdit={handleVectorEdit}
                dragMode={dragMode}
                onZoom={handleZoom}
                onPauseToggle={(p) => setState(s => ({...s, paused: p}))}
                arcCreation={arcBuilder} 
                rampCreation={rampBuilder}
                constraintBuilder={constraintBuilder}
             />

             {/* Equation Builder Modal */}
             {showEquationBuilder && (
                 <div className="absolute inset-0 bg-black/50 backdrop-blur-sm flex items-center justify-center z-50">
                     <div className="bg-slate-900 border border-slate-700 rounded-lg shadow-2xl w-80 p-4">
                         <div className="flex justify-between items-center mb-4 border-b border-slate-700 pb-2">
                             <h3 className="text-white font-bold flex items-center gap-2"><Calculator size={16}/> 方程生成物体</h3>
                             <button onClick={() => setShowEquationBuilder(false)} className="text-slate-400 hover:text-white"><X size={16}/></button>
                         </div>
                         <div className="space-y-3">
                             <div>
                                 <label className="text-xs text-slate-400 block mb-1">x(t) = </label>
                                 <input type="text" value={eqParams.x} onChange={e => setEqParams({...eqParams, x: e.target.value})} className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-xs font-mono" />
                             </div>
                             <div>
                                 <label className="text-xs text-slate-400 block mb-1">y(t) = </label>
                                 <input type="text" value={eqParams.y} onChange={e => setEqParams({...eqParams, y: e.target.value})} className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-xs font-mono" />
                             </div>
                             <div className="grid grid-cols-3 gap-2">
                                 <div>
                                     <label className="text-[10px] text-slate-500 block">Start t</label>
                                     <input type="number" value={eqParams.tStart} onChange={e => setEqParams({...eqParams, tStart: parseFloat(e.target.value)})} className="w-full bg-slate-800 border border-slate-600 rounded px-1 text-xs" />
                                 </div>
                                 <div>
                                     <label className="text-[10px] text-slate-500 block">End t</label>
                                     <input type="number" value={eqParams.tEnd} onChange={e => setEqParams({...eqParams, tEnd: parseFloat(e.target.value)})} className="w-full bg-slate-800 border border-slate-600 rounded px-1 text-xs" />
                                 </div>
                                 <div>
                                     <label className="text-[10px] text-slate-500 block">Steps</label>
                                     <input type="number" value={eqParams.steps} onChange={e => setEqParams({...eqParams, steps: parseFloat(e.target.value)})} className="w-full bg-slate-800 border border-slate-600 rounded px-1 text-xs" />
                                 </div>
                             </div>

                             <div className="flex items-center gap-2 pt-1 border-t border-slate-700">
                                 <input 
                                     type="checkbox" 
                                     id="hollowCheck" 
                                     checked={eqParams.isHollow} 
                                     onChange={e => setEqParams({...eqParams, isHollow: e.target.checked})}
                                     className="rounded bg-slate-800 border-slate-600"
                                 />
                                 <label htmlFor="hollowCheck" className="text-xs text-amber-400 flex flex-col">
                                     <span>链式结构 / 中空 (Chain/Hollow)</span>
                                     <span className="text-[9px] text-slate-500">推荐开启。关闭后将生成实心凸多边形，可能导致"空气墙"。</span>
                                 </label>
                             </div>

                             <button onClick={handleGenerateEquationBody} className="w-full bg-blue-600 hover:bg-blue-500 text-white py-1.5 rounded text-sm mt-2">生成 (Generate)</button>
                         </div>
                     </div>
                 </div>
             )}

             {/* Hints */}
             {dragMode === 'add_poly' && (
                  <div className="absolute top-4 left-1/2 -translate-x-1/2 bg-emerald-900/80 text-emerald-100 px-3 py-1 rounded-full text-xs shadow-lg pointer-events-none">
                      {polyBuilder.length === 0 ? "点击画布添加顶点" : polyBuilder.length < 3 ? "继续添加顶点..." : "点击起点闭合形状"}
                  </div>
             )}
             
             {dragMode === 'tool_combine' && (
                 <div className="absolute top-4 left-1/2 -translate-x-1/2 bg-blue-900/80 text-blue-100 px-3 py-1 rounded-full text-xs shadow-lg pointer-events-none">
                     {combineSelection.length === 0 ? "依次点击两个物体进行合并" : "点击第二个物体..."}
                 </div>
             )}

             {/* Overlay Info */}
             <div className="absolute top-4 right-4 bg-slate-900/80 backdrop-blur border border-slate-700 p-3 rounded-lg text-xs text-slate-300 pointer-events-none select-none shadow-xl flex flex-col gap-1 z-0">
                 <div className="flex justify-between w-40">
                     <span className="text-slate-500">缩放:</span>
                     <span className="font-mono">{(state.camera.zoom * 100).toFixed(0)}%</span>
                 </div>
                 <div className="flex justify-between">
                     <span className="text-slate-500">时间:</span>
                     <span className="font-mono text-emerald-400">{formatTime(state.time)}</span>
                 </div>
                 <div className="flex justify-between">
                     <span className="text-slate-500">对象:</span>
                     <span className="font-mono">{state.bodies.length}</span>
                 </div>
                 
                 {arcBuilder && (
                     <p className="text-blue-400 font-bold mt-1 border-t border-slate-700 pt-1">
                         {arcBuilder.phase === 1 ? '步骤 1: 点击圆心' : arcBuilder.phase === 2 ? '步骤 2: 点击起点(定义半径)' : '步骤 3: 点击终点'}
                     </p>
                 )}
                 {constraintBuilder && (
                     <p className="text-amber-400 font-bold mt-1 border-t border-slate-700 pt-1">
                         选择第二个物体...
                     </p>
                 )}
                 {dragMode === 'tool_cut' && (
                     <p className="text-red-400 font-bold mt-1 border-t border-slate-700 pt-1">
                        拖拽划线进行切割
                     </p>
                 )}
             </div>

             {/* Snapshots Panel */}
             {showSnapshots && (
                 <div className="absolute top-4 left-4 w-64 bg-slate-900/95 backdrop-blur border border-slate-700 rounded-lg shadow-xl p-3 z-20 flex flex-col max-h-[500px]">
                     <div className="flex justify-between items-center mb-2">
                         <h3 className="text-xs font-bold text-amber-500 uppercase flex items-center gap-2"><Clock size={14}/> 时间回溯</h3>
                         <button onClick={() => setShowSnapshots(false)} className="text-slate-500 hover:text-white"><Minus size={14}/></button>
                     </div>
                     <button onClick={createSnapshot} className="bg-blue-600 hover:bg-blue-500 text-white text-xs py-1.5 px-2 rounded mb-3 flex items-center justify-center gap-2">
                         <Camera size={14} /> 创建快照
                     </button>
                     <div className="flex-1 overflow-y-auto space-y-1">
                         {snapshots.length === 0 && <p className="text-[10px] text-slate-500 text-center italic">无快照记录</p>}
                         {snapshots.map(snap => (
                             <div key={snap.id} className="flex justify-between items-center bg-slate-800/50 p-2 rounded hover:bg-slate-700 transition group">
                                 <div className="flex flex-col">
                                     <span className="text-xs font-mono text-emerald-400">{formatTime(snap.timestamp)}</span>
                                     <span className="text-[9px] text-slate-500">Objects: {JSON.parse(snap.data).bodies.length}</span>
                                 </div>
                                 <div className="flex gap-1">
                                     <button onClick={() => restoreSnapshot(snap)} className="text-[10px] bg-slate-600 hover:bg-emerald-600 text-white px-2 py-0.5 rounded">恢复</button>
                                     <button onClick={() => setSnapshots(s => s.filter(x => x.id !== snap.id))} className="text-slate-500 hover:text-red-400 p-0.5"><Trash2 size={12}/></button>
                                 </div>
                             </div>
                         ))}
                     </div>
                 </div>
             )}

             {/* Global Settings Panel */}
             {showGlobalSettings && (
                 <div className="absolute top-4 left-72 w-56 bg-slate-900/95 backdrop-blur border border-slate-700 rounded-lg shadow-xl p-3 z-20">
                     <div className="flex justify-between items-center mb-3">
                         <h3 className="text-xs font-bold text-purple-500 uppercase flex items-center gap-2"><Settings size={14}/> 全局设置</h3>
                         <button onClick={() => setShowGlobalSettings(false)} className="text-slate-500 hover:text-white"><Minus size={14}/></button>
                     </div>
                     
                     <div className="space-y-4">
                         <div>
                             <label className="text-[10px] text-slate-400 uppercase mb-1 block">重力加速度 Gravity (g)</label>
                             <div className="grid grid-cols-2 gap-2">
                                 <div>
                                     <span className="text-[9px] text-slate-500">X</span>
                                     <input 
                                        type="number" value={state.gravity.x} 
                                        onChange={(e) => setState(s => ({...s, gravity: {...s.gravity, x: parseFloat(e.target.value)}}))}
                                        className="w-full bg-slate-800 border border-slate-600 rounded px-1 text-xs"
                                     />
                                 </div>
                                 <div>
                                     <span className="text-[9px] text-slate-500">Y</span>
                                     <input 
                                        type="number" value={state.gravity.y} 
                                        onChange={(e) => setState(s => ({...s, gravity: {...s.gravity, y: parseFloat(e.target.value)}}))}
                                        className="w-full bg-slate-800 border border-slate-600 rounded px-1 text-xs"
                                     />
                                 </div>
                             </div>
                         </div>

                         <div>
                             <div className="flex items-center justify-between mb-2">
                                 <label className="text-[10px] text-slate-400 uppercase block">库仑力 Coulomb</label>
                                 <input 
                                    type="checkbox" 
                                    checked={state.enableCoulomb} 
                                    onChange={(e) => setState(s => ({...s, enableCoulomb: e.target.checked}))}
                                    className="rounded border-slate-600 bg-slate-800"
                                 />
                             </div>

                             <div className="flex items-center justify-between mb-2">
                                 <label className="text-[10px] text-slate-400 uppercase block">万有引力 Universal Gravity</label>
                                 <input 
                                    type="checkbox" 
                                    checked={state.enableUniversalGravity} 
                                    onChange={(e) => setState(s => ({...s, enableUniversalGravity: e.target.checked}))}
                                    className="rounded border-slate-600 bg-slate-800"
                                 />
                             </div>
                             
                             <div className="flex items-center justify-between">
                                 <label className="text-[10px] text-slate-400 uppercase block">空气阻力 Air Resistance</label>
                                 <input 
                                    type="checkbox" 
                                    checked={state.enableAirResistance} 
                                    onChange={(e) => setState(s => ({...s, enableAirResistance: e.target.checked}))}
                                    className="rounded border-slate-600 bg-slate-800"
                                 />
                             </div>
                         </div>
                     </div>
                 </div>
             )}

             {/* Floating Chart (Pinned) */}
             {pinnedBodyId && (
                 <div className="absolute top-24 right-4 w-80 h-48 bg-slate-900/95 backdrop-blur border border-slate-600 rounded-lg shadow-2xl z-10 flex flex-col">
                      <div className="h-6 px-2 bg-slate-800 rounded-t-lg flex items-center justify-between">
                          <span className="text-[10px] text-slate-400 uppercase">实时图表 (Pinned)</span>
                      </div>
                      <div className="flex-1 p-1">
                          <DataCharts 
                              selectedBodyId={pinnedBodyId}
                              state={state}
                              isPinned={true}
                              onTogglePin={() => setPinnedBodyId(null)}
                          />
                      </div>
                 </div>
             )}
          </main>

          {/* Right Panel */}
          <aside className="w-80 bg-slate-900 border-l border-slate-800 flex flex-col z-10">
              <div className="flex-1 overflow-y-auto no-scrollbar border-b border-slate-800">
                  <PropertiesPanel 
                      body={state.selectedBodyId ? state.bodies.find(b => b.id === state.selectedBodyId) || null : null}
                      field={state.selectedFieldId ? state.fields.find(f => f.id === state.selectedFieldId) || null : null}
                      constraint={state.selectedConstraintId ? state.constraints.find(c => c.id === state.selectedConstraintId) || null : null}
                      onUpdateBody={handleUpdateBody}
                      onUpdateField={handleUpdateField}
                      onUpdateConstraint={handleUpdateConstraint}
                      onDeleteBody={handleDeleteBody}
                      onDeleteField={handleDeleteField}
                      onDeleteConstraint={handleDeleteConstraint}
                  />
              </div>
              
              <div className="h-64 bg-slate-900 flex flex-col p-2 transition-all">
                  <div className="flex items-center justify-between mb-2 px-1">
                      <span className="text-xs font-semibold text-slate-500 uppercase">实时数据 (Data)</span>
                      <Activity size={14} className="text-slate-600" />
                  </div>
                  <div className="flex-1 bg-slate-950 rounded border border-slate-800 relative overflow-hidden">
                      <DataCharts 
                          selectedBodyId={state.selectedBodyId} 
                          state={state} 
                          isPinned={pinnedBodyId === state.selectedBodyId}
                          onTogglePin={(id) => setPinnedBodyId(id)}
                      />
                  </div>
              </div>
          </aside>
      </div>
    </div>
  );
};

export default App;