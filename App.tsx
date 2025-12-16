
import React, { useState, useRef, useEffect, useCallback } from 'react';
import { Activity, X, Calculator, Clock, Camera, Minus, Trash2, Settings, Focus, ZoomIn, ZoomOut, Maximize, Video } from 'lucide-react';
import SimulationCanvas, { CanvasRef } from './components/SimulationCanvas';
import PropertiesPanel from './components/PropertiesPanel';
import DataCharts from './components/DataCharts';
import { Header } from './components/Header';
import { Toolbar } from './components/Toolbar';
import { PhysicsEngine } from './services/physicsEngine';
import { BodyType, PhysicsBody, SimulationState, Vector2, ConstraintType, FieldType, PhysicsField, FieldShape, Constraint, PinnedChart } from './types';
import { Vec2 } from './services/vectorMath';

const INITIAL_STATE: SimulationState = {
  canvasName: 'Project',
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
  camera: { x: 0, y: 0, zoom: 1, trackingBodyId: null },
  pinnedCharts: [],
  isRecording: false
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
    data: string;
}

const App: React.FC = () => {
  const [state, setState] = useState<SimulationState>(INITIAL_STATE);
  const [dragMode, setDragMode] = useState<string>('select');
  
  // Layout
  const [sidebarWidth, setSidebarWidth] = useState(320);
  const [toolbarWidth, setToolbarWidth] = useState(80);
  
  // Builders
  const [arcBuilder, setArcBuilder] = useState<ArcBuilder | null>(null);
  const [rampBuilder, setRampBuilder] = useState<RampBuilder | null>(null);
  const [polyBuilder, setPolyBuilder] = useState<Vector2[]>([]);
  const [constraintBuilder, setConstraintBuilder] = useState<string | null>(null); 
  
  const [combineSelection, setCombineSelection] = useState<string[]>([]);

  // UI State
  const [snapshots, setSnapshots] = useState<Snapshot[]>([]);
  const [showSnapshots, setShowSnapshots] = useState(false);
  const [showGlobalSettings, setShowGlobalSettings] = useState(false);
  const [showEquationBuilder, setShowEquationBuilder] = useState(false);
  const [eqParams, setEqParams] = useState({ x: "100 * Math.cos(t)", y: "100 * Math.sin(t)", tStart: 0, tEnd: 6.28, steps: 100, isHollow: true });

  const engineRef = useRef(new PhysicsEngine());
  const canvasRef = useRef<CanvasRef>(null);
  const reqRef = useRef<number>(0);
  const lastTimeRef = useRef<number>(0);
  const recorderRef = useRef<MediaRecorder | null>(null);
  const chunksRef = useRef<Blob[]>([]);

  // --- Animation Loop ---
  const tick = useCallback((time: number) => {
    if (!lastTimeRef.current) lastTimeRef.current = time;
    const dt = Math.min((time - lastTimeRef.current) / 1000, 0.05); 
    lastTimeRef.current = time;

    setState(prev => {
      // 1. Physics Step
      let nextState = { ...prev };
      
      if (!prev.paused) {
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
                  // Optimize trail density
                  const last = b.trail[b.trail.length - 1];
                  if (!last || Vec2.magSq(Vec2.sub(last, b.position)) > 100) { 
                      b.trail.push({ ...b.position });
                      if (b.trail.length > 200) b.trail.shift();
                  }
              }
          });
          nextState.bodies = newBodies;
          nextState.time = prev.time + dt;
      }

      // 2. Camera Tracking
      if (nextState.camera.trackingBodyId) {
          const target = nextState.bodies.find(b => b.id === nextState.camera.trackingBodyId);
          if (target) {
             const centerX = window.innerWidth / 2;
             const centerY = window.innerHeight / 2;
             const targetCamX = centerX - target.position.x * nextState.camera.zoom;
             const targetCamY = centerY - target.position.y * nextState.camera.zoom;
             nextState.camera = {
                 ...nextState.camera,
                 x: nextState.camera.x + (targetCamX - nextState.camera.x) * 0.1,
                 y: nextState.camera.y + (targetCamY - nextState.camera.y) * 0.1
             };
          }
      }

      return nextState;
    });

    reqRef.current = requestAnimationFrame(tick);
  }, []);

  useEffect(() => {
    reqRef.current = requestAnimationFrame(tick);
    return () => { if (reqRef.current) cancelAnimationFrame(reqRef.current); };
  }, [tick]);

  // --- Handlers ---

  const handleTogglePause = () => setState(s => ({ ...s, paused: !s.paused }));
  
  const handleReset = () => {
      setState(s => ({ ...s, time: 0, paused: true, bodies: s.bodies.map(b => ({...b, position: b.trail?.[0] || b.position, velocity: {x:0,y:0}, angularVelocity:0})) }));
  };

  const handleClearAll = () => {
      if(confirm("确定清空画布吗?")) {
          setState({...INITIAL_STATE, canvasName: state.canvasName, camera: state.camera, paused: true});
      }
  };

  const handlePreset = (type: 'smooth' | 'elastic') => {
      setState(s => ({
          ...s,
          bodies: s.bodies.map(b => ({
              ...b,
              friction: type === 'smooth' ? 0 : 0.5,
              restitution: type === 'elastic' ? 1.0 : b.restitution
          }))
      }));
  };

  const handleSaveScene = () => {
      const data = JSON.stringify(state);
      const blob = new Blob([data], {type: 'application/json'});
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `${state.canvasName || 'scene'}.json`;
      a.click();
  };

  const handleImportScene = (e: React.ChangeEvent<HTMLInputElement>) => {
      const file = e.target.files?.[0];
      if (file) {
          const reader = new FileReader();
          reader.onload = (ev) => {
              try {
                  const newState = JSON.parse(ev.target?.result as string);
                  setState({ ...newState, paused: true });
              } catch(err) {
                  alert("Invalid file format");
              }
          };
          reader.readAsText(file);
      }
  };

  const handleTakeSnapshot = () => {
      const snap: Snapshot = {
          id: `snap_${Date.now()}`,
          timestamp: Date.now(),
          data: JSON.stringify(state)
      };
      setSnapshots([snap, ...snapshots]);
  };

  const handleLoadSnapshot = (snap: Snapshot) => {
      try {
          const loaded = JSON.parse(snap.data);
          setState(loaded);
      } catch(e) {}
  };

  const handleStartRecording = () => {
      const stream = canvasRef.current?.getStream();
      if (stream) {
          chunksRef.current = [];
          const recorder = new MediaRecorder(stream, { mimeType: 'video/webm;codecs=vp9', videoBitsPerSecond: 5000000 });
          recorder.ondataavailable = (e) => { if (e.data.size > 0) chunksRef.current.push(e.data); };
          recorder.onstop = () => {
              const blob = new Blob(chunksRef.current, { type: 'video/webm' });
              const url = URL.createObjectURL(blob);
              const a = document.createElement('a');
              a.href = url;
              a.download = `${state.canvasName}-recording.webm`;
              a.click();
          };
          recorder.start();
          recorderRef.current = recorder;
          setState(s => ({ ...s, isRecording: true, paused: false }));
      } else {
          alert("Canvas stream unavailable");
      }
  };

  const handleStopRecording = () => {
      if (recorderRef.current && state.isRecording) {
          recorderRef.current.stop();
          setState(s => ({ ...s, isRecording: false, paused: true }));
      }
  };
  
  const handlePinChart = (bodyId: string) => {
      setState(s => {
          if (s.pinnedCharts.find(c => c.bodyId === bodyId)) return s;
          const newChart: PinnedChart = {
              id: `chart_${Date.now()}`,
              bodyId,
              position: { x: 100 + (s.pinnedCharts.length * 30), y: 100 + (s.pinnedCharts.length * 30) }
          };
          return { ...s, pinnedCharts: [...s.pinnedCharts, newChart] };
      });
  };

  const closeChart = (id: string) => {
      setState(s => ({ ...s, pinnedCharts: s.pinnedCharts.filter(c => c.id !== id) }));
  };

  const updateChartPosition = (id: string, x: number, y: number) => {
      setState(s => ({
          ...s,
          pinnedCharts: s.pinnedCharts.map(c => c.id === id ? { ...c, position: { x, y } } : c)
      }));
  };

  const handleCreateEquationBody = () => {
      try {
          const points: Vector2[] = [];
          const step = (eqParams.tEnd - eqParams.tStart) / eqParams.steps;
          for(let t = eqParams.tStart; t <= eqParams.tEnd; t += step) {
              const x = new Function('t', `with(Math){ return ${eqParams.x} }`)(t);
              const y = new Function('t', `with(Math){ return ${eqParams.y} }`)(t);
              points.push({x, y});
          }

          let cx = 0, cy = 0; points.forEach(p => { cx+=p.x; cy+=p.y; });
          cx/=points.length; cy/=points.length;
          const center = {x:cx, y:cy};
          const localVerts = points.map(p => ({x: p.x - cx, y: p.y - cy}));
          
          const newBody: PhysicsBody = {
              id: `eq_${Date.now()}`,
              label: `E${state.bodies.length+1}`,
              type: BodyType.POLYGON,
              position: center,
              velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
              mass: 0, inverseMass: 0, restitution: 0.5, friction: 0.5, charge: 0,
              angle: 0, angularVelocity: 0, momentInertia: 0, inverseInertia: 0,
              vertices: localVerts,
              isHollow: eqParams.isHollow,
              color: '#10b981', selected: true, showTrajectory: false, trail: []
          };
          
          setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
          setShowEquationBuilder(false);
      } catch(e) {
          alert("方程错误");
      }
  };

  // --- Building Logic ---
  const handleAddBody = (pos: Vector2) => {
    // Ramp Builder
    if (dragMode === 'add_ramp' || dragMode === 'add_conveyor') {
        if (!rampBuilder) {
            setRampBuilder({ start: pos, isConveyor: dragMode === 'add_conveyor' });
        } else {
            const start = rampBuilder.start; 
            const delta = Vec2.sub(pos, start);
            const len = Vec2.mag(delta);
            let angle = Math.atan2(delta.y, delta.x);
            // Snap Angle (Relaxed: only if close to 15 deg steps)
            const deg = angle * (180/Math.PI);
            const snappedDeg = Math.round(deg / 15) * 15;
            if (Math.abs(deg - snappedDeg) < 5) {
                angle = snappedDeg * (Math.PI/180);
            }
            
            const snappedEnd = { x: start.x + len * Math.cos(angle), y: start.y + len * Math.sin(angle) };
            const center = Vec2.mul(Vec2.add(start, snappedEnd), 0.5);

            const newBody: PhysicsBody = {
                id: `ramp_${Date.now()}`,
                label: `R${state.bodies.length+1}`,
                type: BodyType.LINE,
                position: center,
                velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
                mass: 0, inverseMass: 0, restitution: 0.5, friction: 0.5, charge: 0,
                angle: angle, angularVelocity: 0, momentInertia: 0, inverseInertia: 0,
                length: len,
                color: rampBuilder.isConveyor ? '#8b5cf6' : '#64748b', 
                selected: true, showTrajectory: false, trail: []
            };
            if (rampBuilder.isConveyor) { newBody.surfaceSpeed = 50; }
            setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
            setRampBuilder(null); setDragMode('select');
        }
        return;
    }
    
    // Poly Builder
    if (dragMode === 'add_poly') {
        if (polyBuilder.length > 0 && Vec2.dist(pos, polyBuilder[0]) < 20) {
             if (polyBuilder.length < 3) return; 
             // Finish Poly
             let cx = 0, cy = 0; polyBuilder.forEach(v => { cx += v.x; cy += v.y; });
             cx /= polyBuilder.length; cy /= polyBuilder.length;
             const center = { x: cx, y: cy };
             const localVerts = polyBuilder.map(v => ({ x: v.x - center.x, y: v.y - center.y }));
             const newBody: PhysicsBody = {
                 id: `poly_${Date.now()}`,
                 label: `P${state.bodies.length+1}`,
                 type: BodyType.POLYGON,
                 position: center,
                 velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
                 mass: 5, inverseMass: 0.2, restitution: 0.5, friction: 0.5, charge: 0,
                 angle: 0, angularVelocity: 0, momentInertia: 100, inverseInertia: 0.01,
                 vertices: localVerts, color: '#14b8a6', selected: true, showTrajectory: false, trail: []
             };
             setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
             setPolyBuilder([]); setDragMode('select');
        } else {
             setPolyBuilder([...polyBuilder, pos]);
        }
        return;
    }

    // Arc Builder
    if (dragMode === 'add_arc') {
        if (!arcBuilder) {
            setArcBuilder({ phase: 1, center: pos, radius: 0, startAngle: 0, endAngle: 0 });
        } else if (arcBuilder.phase === 1) {
            const r = Vec2.dist(arcBuilder.center, pos);
            const angle = Math.atan2(pos.y - arcBuilder.center.y, pos.x - arcBuilder.center.x);
            setArcBuilder({ ...arcBuilder, phase: 2, radius: r, startAngle: angle, endAngle: 0 });
        } else if (arcBuilder.phase === 2) {
             const angle = Math.atan2(pos.y - arcBuilder.center.y, pos.x - arcBuilder.center.x);
             let endAngle = angle;
             if (Math.abs(angle - arcBuilder.startAngle) < 0.1) endAngle = arcBuilder.startAngle + Math.PI*1.99;
             
             const newBody: PhysicsBody = {
                id: `arc_${Date.now()}`,
                label: `A${state.bodies.length+1}`,
                type: BodyType.ARC,
                position: arcBuilder.center,
                velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
                mass: 0, inverseMass: 0, restitution: 1.0, friction: 0.5, charge: 0,
                angle: 0, angularVelocity: 0, momentInertia: 0, inverseInertia: 0,
                radius: arcBuilder.radius, arcStartAngle: arcBuilder.startAngle, arcEndAngle: endAngle,
                color: '#e2e8f0', selected: true, showTrajectory: false, trail: [], isHollow: true
            };
            setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
            setArcBuilder(null); setDragMode('select');
        }
        return;
    }
    
    // Normal Bodies
    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin', 'add_rope'].includes(dragMode)) {
        const newId = `body_${Date.now()}`;
        const count = state.bodies.length + 1;
        let newBody: PhysicsBody = {
            id: newId,
            label: `B${count}`,
            type: BodyType.CIRCLE,
            position: pos,
            velocity: {x: 0, y: 0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
            mass: 5, inverseMass: 0.2, restitution: 0.8, friction: 0.4, charge: 0,
            angle: 0, angularVelocity: 0, momentInertia: 50, inverseInertia: 0.02,
            color: '#ec4899', selected: true, showTrajectory: false, trail: [],
            showVelocity: false, showForce: false, showAcceleration: false, showCharge: false,
            customGraph: { show: false, eqX: 't', eqY: 'v', color: '#ff00ff', data: [] }
        };
        if (dragMode === 'add_box') { newBody.type = BodyType.BOX; newBody.width = 40; newBody.height = 40; newBody.color = '#3b82f6'; }
        if (dragMode === 'add_particle') { newBody.isParticle = true; newBody.radius = 5; newBody.mass = 1; newBody.inverseMass = 1; newBody.color = '#facc15'; }
        else { newBody.radius = 20; }
        
        setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newId }));
        setDragMode('select');
    }
  };
  
  const handleCombineBodies = (id1: string, id2: string) => {
      // ... existing logic ...
      const b1 = state.bodies.find(b => b.id === id1);
      const b2 = state.bodies.find(b => b.id === id2);
      if (!b1 || !b2) return;
      
      const getVerts = (b: PhysicsBody) => {
          if (b.type === BodyType.CIRCLE) {
               const steps = 12;
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
      
      let cx = 0, cy = 0; hull.forEach(v => { cx += v.x; cy += v.y; });
      cx /= hull.length; cy /= hull.length;
      const center = { x: cx, y: cy };
      const localVerts = hull.map(v => ({ x: v.x - center.x, y: v.y - center.y }));
      
      const newBody: PhysicsBody = {
          id: `combined_${Date.now()}`,
          label: `C${state.bodies.length}`,
          type: BodyType.POLYGON,
          position: center,
          velocity: Vec2.div(Vec2.add(b1.velocity, b2.velocity), 2),
          acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
          mass: b1.mass + b2.mass, inverseMass: 1 / (b1.mass + b2.mass || 1),
          restitution: (b1.restitution + b2.restitution)/2, friction: (b1.friction + b2.friction)/2,
          charge: b1.charge + b2.charge,
          angle: 0, angularVelocity: 0, momentInertia: 100, inverseInertia: 0.01,
          vertices: localVerts, color: b1.color, selected: true, showTrajectory: false, trail: []
      };

      setState(s => ({
          ...s,
          bodies: [...s.bodies.filter(b => b.id !== id1 && b.id !== id2), newBody],
          constraints: s.constraints.filter(c => c.bodyAId !== id1 && c.bodyAId !== id2 && c.bodyBId !== id1 && c.bodyBId !== id2),
          selectedBodyId: newBody.id
      }));
  };

  const handleMoveBody = (id: string, pos: Vector2) => {
      setState(s => {
         const body = s.bodies.find(b => b.id === id);
         if (!body) return s;
         return { 
             ...s, 
             bodies: s.bodies.map(b => b.id === id ? { 
                 ...b, 
                 position: pos, 
                 velocity: {x:0,y:0}, 
                 force: {x:0,y:0} 
             } : b) 
         };
      });
  };

  return (
    <div className="w-full h-screen flex flex-col bg-slate-950 text-slate-200 overflow-hidden font-sans">
      <Header 
          state={state}
          onTogglePause={handleTogglePause}
          onReset={handleReset}
          onClearAll={handleClearAll}
          onPreset={handlePreset}
          onSaveScene={handleSaveScene}
          onImportScene={handleImportScene}
          canvasRef={canvasRef}
          showSnapshots={showSnapshots}
          setShowSnapshots={setShowSnapshots}
          showSettings={showGlobalSettings}
          setShowSettings={setShowGlobalSettings}
          onChangeName={(n) => setState(s => ({...s, canvasName: n}))}
      />

      <div className="flex-1 flex overflow-hidden relative">
          {/* Resizable Toolbar */}
          <div className="relative z-20 flex flex-col" style={{ width: toolbarWidth }}>
             <Toolbar 
                dragMode={dragMode}
                setDragMode={setDragMode}
                setPolyBuilder={setPolyBuilder}
                setArcBuilder={setArcBuilder}
                setRampBuilder={setRampBuilder}
                setCombineSelection={setCombineSelection}
                setShowEquationBuilder={setShowEquationBuilder}
            />
            {/* Toolbar Resizer */}
            <div 
                  className="w-1 cursor-col-resize hover:bg-blue-500 bg-slate-800 transition-colors absolute right-0 top-0 bottom-0 z-50"
                  onMouseDown={(e) => {
                      const startX = e.clientX;
                      const startW = toolbarWidth;
                      const onMove = (ev: MouseEvent) => setToolbarWidth(Math.max(60, Math.min(200, startW + (ev.clientX - startX))));
                      const onUp = () => { document.removeEventListener('mousemove', onMove); document.removeEventListener('mouseup', onUp); };
                      document.addEventListener('mousemove', onMove);
                      document.addEventListener('mouseup', onUp);
                  }}
              />
          </div>

          <main className="flex-1 relative bg-slate-950">
             <SimulationCanvas 
                ref={canvasRef}
                state={state}
                onSelectBody={(id) => {
                    if (dragMode === 'tool_combine' && id) {
                         const newSel = [...combineSelection, id];
                         setCombineSelection(newSel);
                         if (newSel.length >= 2) {
                             handleCombineBodies(newSel[0], newSel[1]);
                             setCombineSelection([]); setDragMode('select');
                         }
                         return;
                    }
                    if (['add_spring', 'add_rod', 'add_pin', 'add_rope'].includes(dragMode) && id) {
                        if (!constraintBuilder) setConstraintBuilder(id);
                        else {
                            const type = dragMode === 'add_spring' ? ConstraintType.SPRING : dragMode === 'add_rod' ? ConstraintType.ROD : dragMode === 'add_rope' ? ConstraintType.ROPE : ConstraintType.PIN;
                            const bA = state.bodies.find(b => b.id === constraintBuilder);
                            const bB = state.bodies.find(b => b.id === id);
                            if (bA && bB) {
                                const newC: Constraint = {
                                    id: `c_${Date.now()}`, type, bodyAId: constraintBuilder, bodyBId: id,
                                    localAnchorA: {x:0, y:0}, localAnchorB: {x:0, y:0},
                                    length: Vec2.dist(bA.position, bB.position), stiffness: 0.5, damping: 0.1
                                };
                                setState(s => ({ ...s, constraints: [...s.constraints, newC] }));
                            }
                            setConstraintBuilder(null); setDragMode('select');
                        }
                        return;
                    }
                    setState(s => ({ ...s, selectedBodyId: id, selectedFieldId: null, selectedConstraintId: null }));
                }}
                onSelectField={(id) => setState(s => ({ ...s, selectedFieldId: id, selectedBodyId: null, selectedConstraintId: null }))}
                onSelectConstraint={(id) => setState(s => ({ ...s, selectedConstraintId: id, selectedBodyId: null, selectedFieldId: null }))}
                onAddBody={handleAddBody}
                onAddField={(pos, size, shape, verts) => {
                     const newField: PhysicsField = {
                        id: `field_${Date.now()}`, type: FieldType.UNIFORM_ELECTRIC, shape, position: pos, size: size, radius: size.x, vertices: verts, strength: {x:10, y:0}, visible: true
                     };
                     setState(s => ({ ...s, fields: [...s.fields, newField], selectedFieldId: newField.id }));
                     setDragMode('select');
                }}
                onMoveBody={handleMoveBody}
                onVectorEdit={(id, type, vec) => setState(s => ({ ...s, bodies: s.bodies.map(b => b.id === id ? (type === 'velocity' ? { ...b, velocity: vec } : { ...b, constantForce: vec }) : b) }))}
                dragMode={dragMode}
                onZoom={(delta, mx, my) => {
                    setState(s => {
                        const scale = delta > 0 ? 0.9 : 1.1;
                        const newZoom = Math.max(0.1, Math.min(10, s.camera.zoom * scale));
                        const wx = (mx - s.camera.x) / s.camera.zoom;
                        const wy = (my - s.camera.y) / s.camera.zoom;
                        return { ...s, camera: { ...s.camera, zoom: newZoom, x: mx - wx * newZoom, y: my - wy * newZoom } };
                    });
                }}
                onPan={(dx, dy) => setState(s => ({ ...s, camera: { ...s.camera, x: s.camera.x + dx, y: s.camera.y + dy, trackingBodyId: null } }))}
                onPauseToggle={(p) => setState(s => ({ ...s, paused: p }))}
                arcCreation={arcBuilder}
                rampCreation={rampBuilder}
                constraintBuilder={constraintBuilder}
                polyBuilder={polyBuilder}
             />
             
             {state.pinnedCharts.map(chart => (
                 <div 
                    key={chart.id}
                    className="absolute shadow-xl border border-slate-700 rounded overflow-hidden z-40 bg-slate-900"
                    style={{ left: chart.position.x, top: chart.position.y, width: 300, height: 200 }}
                    onMouseDown={(e) => {
                        e.stopPropagation();
                        const startX = e.clientX; const startY = e.clientY;
                        const initialX = chart.position.x; const initialY = chart.position.y;
                        const moveHandler = (ev: MouseEvent) => {
                            updateChartPosition(chart.id, initialX + (ev.clientX - startX), initialY + (ev.clientY - startY));
                        };
                        const upHandler = () => {
                            document.removeEventListener('mousemove', moveHandler);
                            document.removeEventListener('mouseup', upHandler);
                        };
                        document.addEventListener('mousemove', moveHandler);
                        document.addEventListener('mouseup', upHandler);
                    }}
                 >
                     <DataCharts selectedBodyId={chart.bodyId} state={state} onClose={() => closeChart(chart.id)} width={300} height={200} />
                 </div>
             ))}

             <div className="absolute top-4 right-4 z-30">
                 <button 
                    onClick={state.isRecording ? handleStopRecording : handleStartRecording}
                    className={`flex items-center gap-2 px-3 py-1.5 rounded-full shadow-lg border border-slate-700 transition-all font-bold text-xs ${state.isRecording ? 'bg-red-600 animate-pulse text-white' : 'bg-slate-800 text-white hover:bg-slate-700'}`}
                 >
                     <Video size={16} />
                     {state.isRecording ? "停止录制 (Stop)" : "录制 (WebM)"}
                 </button>
             </div>

             <div className="absolute bottom-6 right-6 flex flex-col gap-2 z-20">
                 <div className="bg-slate-800/80 p-1 rounded-lg backdrop-blur flex flex-col gap-1 border border-slate-700">
                     <button onClick={() => setState(s => ({ ...s, camera: { ...s.camera, zoom: s.camera.zoom * 1.2 } }))} className="p-2 text-slate-300 hover:text-white hover:bg-slate-700 rounded"><ZoomIn size={18} /></button>
                     <button onClick={() => setState(s => ({ ...s, camera: { ...s.camera, zoom: s.camera.zoom / 1.2 } }))} className="p-2 text-slate-300 hover:text-white hover:bg-slate-700 rounded"><ZoomOut size={18} /></button>
                     <button onClick={() => setState(s => ({ ...s, camera: { x: 0, y: 0, zoom: 1, trackingBodyId: null } }))} className="p-2 text-slate-300 hover:text-white hover:bg-slate-700 rounded"><Maximize size={18} /></button>
                 </div>
                 
                 {state.selectedBodyId && (
                     <button 
                        onClick={() => setState(s => ({ ...s, camera: { ...s.camera, trackingBodyId: s.camera.trackingBodyId === s.selectedBodyId ? null : s.selectedBodyId } }))}
                        className={`p-2 rounded-full shadow-lg border border-slate-700 ${state.camera.trackingBodyId === state.selectedBodyId ? 'bg-blue-600 text-white' : 'bg-slate-800 text-slate-400'}`}
                        title="Focus Object"
                     >
                         <Focus size={20} />
                     </button>
                 )}
             </div>

             {/* Snapshots & Settings Overlays */}
             {showSnapshots && (
                 <div className="absolute top-14 right-10 bg-slate-900 border border-slate-700 p-4 rounded shadow-xl w-64 z-30">
                     <h3 className="text-sm font-bold mb-2">历史快照</h3>
                     <button onClick={handleTakeSnapshot} className="w-full bg-blue-600 hover:bg-blue-500 text-white py-1 rounded text-xs mb-2">创建快照</button>
                     <div className="max-h-60 overflow-y-auto space-y-1">
                         {snapshots.map(s => (
                             <div key={s.id} onClick={() => handleLoadSnapshot(s)} className="p-2 bg-slate-800 hover:bg-slate-700 rounded cursor-pointer text-xs flex justify-between">
                                 <span>{new Date(s.timestamp).toLocaleTimeString()}</span>
                                 <span className="text-slate-500">{(s.data.length/1024).toFixed(1)}KB</span>
                             </div>
                         ))}
                     </div>
                 </div>
             )}
             
             {showGlobalSettings && (
                 <div className="absolute top-14 right-10 bg-slate-900 border border-slate-700 p-4 rounded shadow-xl w-64 z-30">
                     <h3 className="text-sm font-bold mb-3">全局物理设置</h3>
                     <div className="space-y-3 text-xs">
                         <div>
                             <label className="block text-slate-400 mb-1">重力 Gravity Y</label>
                             <input type="number" value={state.gravity.y} onChange={e => setState({...state, gravity: {...state.gravity, y: parseFloat(e.target.value)}})} className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1"/>
                         </div>
                         <div className="flex items-center gap-2">
                             <input type="checkbox" checked={state.enableAirResistance} onChange={e => setState({...state, enableAirResistance: e.target.checked})} />
                             <label>空气阻力 Air Resistance</label>
                         </div>
                         <div className="flex items-center gap-2">
                             <input type="checkbox" checked={state.enableCoulomb} onChange={e => setState({...state, enableCoulomb: e.target.checked})} />
                             <label>库仑力 Coulomb</label>
                         </div>
                         <div className="flex items-center gap-2">
                             <input type="checkbox" checked={state.enableUniversalGravity} onChange={e => setState({...state, enableUniversalGravity: e.target.checked})} />
                             <label>万有引力 Gravity</label>
                         </div>
                     </div>
                 </div>
             )}

             {showEquationBuilder && (
                 <div className="absolute inset-0 bg-black/50 flex items-center justify-center z-50">
                     <div className="bg-slate-900 border border-slate-700 rounded-lg p-6 w-96 shadow-2xl">
                         <h3 className="text-lg font-bold mb-4 text-emerald-400">参数方程创建物体</h3>
                         <div className="space-y-3">
                             <div>
                                 <label className="text-xs text-slate-400 block mb-1">X = f(t)</label>
                                 <input value={eqParams.x} onChange={e => setEqParams({...eqParams, x: e.target.value})} className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm font-mono"/>
                             </div>
                             <div>
                                 <label className="text-xs text-slate-400 block mb-1">Y = g(t)</label>
                                 <input value={eqParams.y} onChange={e => setEqParams({...eqParams, y: e.target.value})} className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm font-mono"/>
                             </div>
                             <div className="grid grid-cols-2 gap-2">
                                 <div>
                                     <label className="text-xs text-slate-400 block mb-1">t Start</label>
                                     <input type="number" value={eqParams.tStart} onChange={e => setEqParams({...eqParams, tStart: parseFloat(e.target.value)})} className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm"/>
                                 </div>
                                 <div>
                                     <label className="text-xs text-slate-400 block mb-1">t End</label>
                                     <input type="number" value={eqParams.tEnd} onChange={e => setEqParams({...eqParams, tEnd: parseFloat(e.target.value)})} className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm"/>
                                 </div>
                             </div>
                             <div className="flex items-center gap-2">
                                <input type="checkbox" checked={eqParams.isHollow} onChange={e => setEqParams({...eqParams, isHollow: e.target.checked})} />
                                <label className="text-xs text-slate-400">空心 (Chain)</label>
                             </div>
                             <div className="flex gap-2 mt-4">
                                 <button onClick={handleCreateEquationBody} className="flex-1 bg-emerald-600 hover:bg-emerald-500 text-white py-1 rounded text-sm">生成</button>
                                 <button onClick={() => setShowEquationBuilder(false)} className="flex-1 bg-slate-700 hover:bg-slate-600 text-white py-1 rounded text-sm">取消</button>
                             </div>
                         </div>
                     </div>
                 </div>
             )}
          </main>

          {/* Resizable Sidebar */}
          <div className="relative z-20 flex" style={{ width: sidebarWidth }}>
              {/* Resizer Handle */}
              <div 
                  className="w-1 cursor-col-resize hover:bg-blue-500 bg-slate-800 transition-colors absolute left-0 top-0 bottom-0 z-50"
                  onMouseDown={(e) => {
                      const startX = e.clientX;
                      const startW = sidebarWidth;
                      const onMove = (ev: MouseEvent) => setSidebarWidth(Math.max(200, Math.min(600, startW + (startX - ev.clientX)))); // Drag left increases width
                      const onUp = () => { document.removeEventListener('mousemove', onMove); document.removeEventListener('mouseup', onUp); };
                      document.addEventListener('mousemove', onMove);
                      document.addEventListener('mouseup', onUp);
                  }}
              />
              <div className="flex-1 bg-slate-900 border-l border-slate-800 flex flex-col overflow-hidden">
                   <div className="flex-1 overflow-y-auto no-scrollbar border-b border-slate-800">
                      <PropertiesPanel 
                          body={state.selectedBodyId ? state.bodies.find(b => b.id === state.selectedBodyId) || null : null}
                          field={state.selectedFieldId ? state.fields.find(f => f.id === state.selectedFieldId) || null : null}
                          constraint={state.selectedConstraintId ? state.constraints.find(c => c.id === state.selectedConstraintId) || null : null}
                          onUpdateBody={(id, u) => setState(s => ({...s, bodies: s.bodies.map(b => b.id === id ? {...b, ...u} : b)}))}
                          onUpdateField={(id, u) => setState(s => ({...s, fields: s.fields.map(f => f.id === id ? {...f, ...u} : f)}))}
                          onUpdateConstraint={(id, u) => setState(s => ({...s, constraints: s.constraints.map(c => c.id === id ? {...c, ...u} : c)}))}
                          onDeleteBody={(id) => setState(s => ({...s, bodies: s.bodies.filter(b => b.id !== id), constraints: s.constraints.filter(c => c.bodyAId !== id && c.bodyBId !== id), selectedBodyId: null}))}
                          onDeleteField={(id) => setState(s => ({...s, fields: s.fields.filter(f => f.id !== id), selectedFieldId: null}))}
                          onDeleteConstraint={(id) => setState(s => ({...s, constraints: s.constraints.filter(c => c.id !== id), selectedConstraintId: null}))}
                      />
                   </div>
              </div>
          </div>
      </div>
    </div>
  );
};

export default App;
