
import React, { useState, useRef, useEffect, useCallback } from 'react';
import { Play, Pause, RotateCcw, MousePointer2, Circle, Square, Triangle, Camera, Download, Link, Eclipse, MoveRight, Upload, Zap, Activity, Minus, Plus, Scan, Crosshair, Disc, Layers, ChevronDown, ChevronRight, Magnet, Globe } from 'lucide-react';
import SimulationCanvas, { CanvasRef } from './components/SimulationCanvas';
import PropertiesPanel from './components/PropertiesPanel';
import DataCharts from './components/DataCharts';
import { PhysicsEngine } from './services/physicsEngine';
import { BodyType, PhysicsBody, SimulationState, Vector2, ConstraintType, FieldType, PhysicsField, FieldShape } from './types';
import { Vec2 } from './services/vectorMath';

// Initial State
const INITIAL_STATE: SimulationState = {
  bodies: [],
  constraints: [],
  fields: [],
  time: 0,
  paused: true,
  gravity: { x: 0, y: 10 },
  selectedBodyId: null,
  selectedFieldId: null,
  selectedConstraintId: null,
  camera: { x: 0, y: 0, zoom: 1 } // Origin at top-left
};

interface ArcBuilder {
    phase: 0 | 1 | 2; 
    center: Vector2;
    radius: number;
    startAngle: number;
}

interface RampBuilder {
    start: Vector2;
    isConveyor: boolean;
}

const App: React.FC = () => {
  const [state, setState] = useState<SimulationState>(INITIAL_STATE);
  const [dragMode, setDragMode] = useState<string>('select');
  const [arcBuilder, setArcBuilder] = useState<ArcBuilder | null>(null);
  const [rampBuilder, setRampBuilder] = useState<RampBuilder | null>(null);
  const [constraintBuilder, setConstraintBuilder] = useState<string | null>(null); // Holds first body ID
  const [pinnedBodyId, setPinnedBodyId] = useState<string | null>(null);
  
  const engineRef = useRef(new PhysicsEngine());
  const canvasRef = useRef<CanvasRef>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const reqRef = useRef<number>(0);
  const lastTimeRef = useRef<number>(0);

  // Animation Loop
  const tick = useCallback((time: number) => {
    if (!lastTimeRef.current) lastTimeRef.current = time;
    const dt = Math.min((time - lastTimeRef.current) / 1000, 0.05); // Cap dt for stability
    lastTimeRef.current = time;

    if (!state.paused) {
      setState(prev => {
        const newBodies = engineRef.current.step(dt, prev.bodies, prev.constraints, prev.fields, prev.gravity);
        
        // Update Trails
        newBodies.forEach(b => {
            if (b.showTrajectory) {
                if (!b.trail) b.trail = [];
                const last = b.trail[b.trail.length - 1];
                if (!last || Vec2.dist(last, b.position) > 2) {
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

  const handleTogglePause = () => setState(s => ({ ...s, paused: !s.paused }));
  const handleReset = () => setState({ ...INITIAL_STATE, camera: state.camera });
  const handleResetView = () => setState(s => ({ ...s, camera: { x: 0, y: 0, zoom: 1 } }));
  
  const handleSelectBody = (id: string | null) => {
      if (dragMode === 'tool_traj' && id) {
          setState(s => ({
              ...s,
              bodies: s.bodies.map(b => b.id === id ? { ...b, showTrajectory: !b.showTrajectory } : b)
          }));
      } else {
          setState(s => ({ ...s, selectedBodyId: id, selectedFieldId: null, selectedConstraintId: null }));
      }
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
          const newZoom = Math.max(0.01, Math.min(20, s.camera.zoom * scale)); // Wider zoom range
          
          const worldMouseX = (mouseX - s.camera.x) / s.camera.zoom;
          const worldMouseY = (mouseY - s.camera.y) / s.camera.zoom;
          
          const newCamX = mouseX - worldMouseX * newZoom;
          const newCamY = mouseY - worldMouseY * newZoom;

          return {
              ...s,
              camera: { x: newCamX, y: newCamY, zoom: newZoom }
          };
      });
  };

  const setZoom = (z: number) => {
      setState(s => {
          const newZoom = Math.max(0.01, Math.min(20, z));
          const cx = window.innerWidth / 2;
          const cy = window.innerHeight / 2;
          const worldCx = (cx - s.camera.x) / s.camera.zoom;
          const worldCy = (cy - s.camera.y) / s.camera.zoom;
          const newCamX = cx - worldCx * newZoom;
          const newCamY = cy - worldCy * newZoom;
          
          return { ...s, camera: { ...s.camera, x: newCamX, y: newCamY, zoom: newZoom } };
      });
  };

  const handleBodyMove = (id: string, pos: Vector2) => {
      setState(s => ({
          ...s,
          bodies: s.bodies.map(b => {
              if (b.id === id) {
                  return {
                      ...b,
                      position: pos,
                      velocity: { x: 0, y: 0 }, 
                      angularVelocity: 0,
                      trail: [] 
                  };
              }
              return b;
          })
      }));
  };

  const handleVectorEdit = (id: string, type: 'velocity' | 'force', vector: Vector2) => {
      setState(s => ({
          ...s,
          bodies: s.bodies.map(b => {
              if (b.id === id) {
                  if (type === 'velocity') return { ...b, velocity: vector };
                  if (type === 'force') return { ...b, constantForce: vector };
              }
              return b;
          })
      }));
  };

  const handleAddField = (pos: Vector2, shape: FieldShape, vertices?: Vector2[]) => {
      const newField: PhysicsField = {
          id: `field_${Date.now()}`,
          type: FieldType.UNIFORM_ELECTRIC, // Default
          shape: shape,
          position: pos,
          size: vertices && vertices.length === 1 ? vertices[0] : {x: 200, y: 200}, // Vertices[0] carries size for box
          radius: vertices && vertices.length === 1 ? vertices[0].x : 100, // Vertices[0].x carries radius for circle
          vertices: shape === FieldShape.POLYGON ? vertices : undefined,
          strength: { x: 10, y: 0 },
          equations: { ex: "Math.sin(x/50) * 50", ey: "0" }, // Default equations
          visible: true
      };

      setState(s => ({
          ...s,
          fields: [newField, ...s.fields], // Add to front (bottom layer conceptually, though render order handles it)
          selectedFieldId: newField.id,
          selectedBodyId: null,
          selectedConstraintId: null
      }));
      setDragMode('select');
  };

  const handleAddBody = (pos: Vector2) => {
    // Ramp/Conveyor
    if (dragMode === 'add_ramp' || dragMode === 'add_conveyor') {
        if (!rampBuilder) {
            setRampBuilder({ start: pos, isConveyor: dragMode === 'add_conveyor' });
        } else {
            const start = rampBuilder.start;
            const end = pos; // Use snapped pos directly
            const center = Vec2.mul(Vec2.add(start, end), 0.5);
            const delta = Vec2.sub(end, start);
            const length = Vec2.mag(delta);
            const angle = Math.atan2(delta.y, delta.x);

            if (length > 1) { // Avoid zero length
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
                    showCharge: true, showVelocity: false, showForce: false
                };
                
                if (rampBuilder.isConveyor) {
                    newBody.surfaceSpeed = 50; 
                    newBody.height = 10; 
                }

                setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
            }
            setRampBuilder(null);
            setDragMode('select');
        }
        return;
    }

    // Arc
    if (dragMode === 'add_arc') {
        if (!arcBuilder) {
            setArcBuilder({ phase: 1, center: pos, radius: 0, startAngle: 0 });
        } else if (arcBuilder.phase === 1) {
            const r = Vec2.dist(arcBuilder.center, pos);
            const startAngle = Math.atan2(pos.y - arcBuilder.center.y, pos.x - arcBuilder.center.x);
            setArcBuilder({ ...arcBuilder, phase: 2, radius: r, startAngle: startAngle });
        } else if (arcBuilder.phase === 2) {
            const angle = Math.atan2(pos.y - arcBuilder.center.y, pos.x - arcBuilder.center.x);
            
            // Logic to determine end angle relative to start angle
            // Normalize angles 0 to 2PI
            const norm = (a: number) => (a % (2*Math.PI) + 2*Math.PI) % (2*Math.PI);
            
            // If dragging very close to start, make full circle
            let finalEndAngle = angle;
            if (Math.abs(norm(angle) - norm(arcBuilder.startAngle)) < 0.1) {
                finalEndAngle = arcBuilder.startAngle + 2 * Math.PI - 0.01;
            }

            const newBody: PhysicsBody = {
                id: `arc_${Date.now()}`,
                type: BodyType.ARC,
                position: arcBuilder.center,
                velocity: {x:0, y:0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
                mass: 0, inverseMass: 0, restitution: 1.0, friction: 0.5, charge: 0,
                angle: 0, angularVelocity: 0, momentInertia: 0, inverseInertia: 0,
                radius: arcBuilder.radius,
                arcStartAngle: arcBuilder.startAngle,
                arcEndAngle: finalEndAngle,
                color: '#e2e8f0', selected: true, showTrajectory: false, trail: [],
                showCharge: true, showVelocity: false, showForce: false
            };
            
            setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
            setArcBuilder(null);
            setDragMode('select');
        }
        return;
    }

    if (dragMode === 'add_spring' || dragMode === 'add_rod' || dragMode === 'add_pin') {
        const clickedBody = state.bodies.slice().reverse().find(b => {
             // Basic hit test to find body under cursor for constraint
             if (b.type === BodyType.CIRCLE) {
                 const hitR = b.isParticle ? 10 : (b.radius || 20);
                 return Vec2.dist(pos, b.position) < hitR;
             }
             if (b.type === BodyType.BOX) {
                 const localPos = Vec2.rotate(Vec2.sub(pos, b.position), -b.angle);
                 const w = b.width || 40; const h = b.height || 40;
                 return Math.abs(localPos.x) < w/2 && Math.abs(localPos.y) < h/2;
             }
             return false;
        });

        if (clickedBody) {
             if (!constraintBuilder) {
                 setConstraintBuilder(clickedBody.id);
             } else {
                 if (constraintBuilder !== clickedBody.id) {
                     const type = dragMode === 'add_spring' ? ConstraintType.SPRING : 
                                  dragMode === 'add_rod' ? ConstraintType.ROD : ConstraintType.PIN;
                     
                     const bodyA = state.bodies.find(b => b.id === constraintBuilder)!;
                     const bodyB = clickedBody;
                     
                     // Calculate local anchors based on world click positions? 
                     // For simplicity, snapping to center is default, but for Box/Rod often surface is better.
                     // Current impl snaps to center for stability.
                     const dist = Vec2.dist(bodyA.position, bodyB.position);

                     const newConstraint = {
                         id: `c_${Date.now()}`,
                         type: type,
                         bodyAId: bodyA.id,
                         bodyBId: bodyB.id,
                         localAnchorA: {x:0, y:0},
                         localAnchorB: {x:0, y:0},
                         length: dist, 
                         stiffness: 0.8,
                         damping: 0.05
                     };
                     setState(s => ({ ...s, constraints: [...s.constraints, newConstraint] }));
                 }
                 setConstraintBuilder(null);
                 setDragMode('select');
             }
        }
        return;
    }

    if (dragMode.startsWith('tool_') || dragMode.startsWith('add_field_') || dragMode === 'select_nodrag') return;

    const newId = `body_${Date.now()}`;
    let newBody: PhysicsBody = {
        id: newId,
        type: BodyType.CIRCLE,
        position: pos,
        velocity: {x: 0, y: 0}, acceleration: {x:0, y:0}, force: {x:0, y:0}, constantForce: {x:0, y:0},
        mass: 5, inverseMass: 0.2, restitution: 1.0, friction: 0.4, charge: 0,
        angle: 0, angularVelocity: 0, momentInertia: 50, inverseInertia: 0.02,
        color: '#ec4899',
        selected: true, showTrajectory: false, trail: [],
        showVelocity: false, showAcceleration: false, showForce: false, showCharge: true
    };

    if (dragMode === 'add_box') {
        newBody.type = BodyType.BOX;
        newBody.width = 40;
        newBody.height = 40;
        newBody.color = '#3b82f6';
    } else if (dragMode === 'add_triangle') {
        newBody.type = BodyType.POLYGON;
        newBody.color = '#10b981';
        const size = 30;
        newBody.vertices = [
            { x: 0, y: -size },
            { x: size * Math.cos(Math.PI/6), y: size * Math.sin(Math.PI/6) },
            { x: -size * Math.cos(Math.PI/6), y: size * Math.sin(Math.PI/6) }
        ];
    } else if (dragMode === 'add_particle') {
        newBody.isParticle = true;
        newBody.radius = 5; // Very small for physics
        newBody.mass = 1;
        newBody.inverseMass = 1;
        newBody.color = '#facc15';
    } else {
        // Standard Ball
        newBody.radius = 20;
    }
    
    setState(s => ({
        ...s,
        bodies: [...s.bodies, newBody],
        selectedBodyId: newId,
        selectedFieldId: null,
        selectedConstraintId: null
    }));
    setDragMode('select'); 
  };

  const handleUpdateBody = (id: string, updates: Partial<PhysicsBody>) => {
    setState(s => ({
        ...s,
        bodies: s.bodies.map(b => b.id === id ? { ...b, ...updates } : b)
    }));
  };

  const handleUpdateField = (id: string, updates: Partial<PhysicsField>) => {
      setState(s => ({
          ...s,
          fields: s.fields.map(f => f.id === id ? { ...f, ...updates } : f)
      }));
  };
  
  const handleUpdateConstraint = (id: string, updates: Partial<any>) => {
      setState(s => ({
          ...s,
          constraints: s.constraints.map(c => c.id === id ? { ...c, ...updates } : c)
      }));
  };

  const handleDeleteBody = (id: string) => {
    setState(s => ({
        ...s,
        bodies: s.bodies.filter(b => b.id !== id),
        constraints: s.constraints.filter(c => c.bodyAId !== id && c.bodyBId !== id),
        selectedBodyId: s.selectedBodyId === id ? null : s.selectedBodyId
    }));
    if (pinnedBodyId === id) setPinnedBodyId(null);
  };

  const handleDeleteField = (id: string) => {
      setState(s => ({
          ...s,
          fields: s.fields.filter(f => f.id !== id),
          selectedFieldId: s.selectedFieldId === id ? null : s.selectedFieldId
      }));
  };

  const handleDeleteConstraint = (id: string) => {
      setState(s => ({
          ...s,
          constraints: s.constraints.filter(c => c.id !== id),
          selectedConstraintId: null
      }));
  };

  const handleSaveScene = () => {
      const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(state));
      const downloadAnchorNode = document.createElement('a');
      downloadAnchorNode.setAttribute("href", dataStr);
      downloadAnchorNode.setAttribute("download", "physlab_scene.json");
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
                      const parsedState = JSON.parse(e.target.result as string);
                      if (parsedState.bodies && parsedState.camera) {
                          setState(parsedState);
                      } else {
                          alert("无效的场景文件");
                      }
                  } catch (err) {
                      alert("JSON 解析错误");
                  }
              }
          };
      }
  };

  const setPreset = (type: 'smooth' | 'elastic') => {
      setState(s => ({
          ...s,
          bodies: s.bodies.map(b => {
              if (b.inverseMass === 0) return b; 
              if (type === 'smooth') return { ...b, friction: 0 };
              if (type === 'elastic') return { ...b, restitution: 1.0, friction: 0 }; 
              return b;
          })
      }));
  };

  const setClear = () => { setDragMode('select'); setArcBuilder(null); setRampBuilder(null); setConstraintBuilder(null); };

  const handleTogglePin = (id: string | null) => {
      setPinnedBodyId(id);
  };

  return (
    <div className="flex h-screen w-screen bg-slate-950 overflow-hidden font-sans text-slate-200">
      <input 
        type="file" 
        ref={fileInputRef} 
        style={{display: 'none'}} 
        onChange={handleImportScene} 
        accept=".json"
      />

      {/* LEFT: Toolbar */}
      <div className="w-16 bg-slate-900 border-r border-slate-800 flex flex-col items-center py-4 gap-2 z-10 shadow-xl overflow-y-auto custom-scrollbar shrink-0">
        <div className="mb-2 shrink-0">
             <div className="w-10 h-10 bg-gradient-to-br from-blue-600 to-indigo-600 rounded-xl flex items-center justify-center font-bold text-white text-sm shadow-lg shadow-blue-900/20">Ph</div>
        </div>

        <div className="w-full flex flex-col items-center gap-1">
             <ToolBtn icon={<MousePointer2 size={18} />} active={dragMode === 'select'} onClick={setClear} tooltip="选择/移动 (Select/Move)" />
             <ToolBtn icon={<Crosshair size={18} />} active={dragMode === 'select_nodrag'} onClick={() => setDragMode('select_nodrag')} tooltip="仅选择 (Select Only)" />
        </div>
        
        <div className="w-8 h-px bg-slate-800 my-1 shrink-0"></div>

        {/* Collapsible Groups */}
        <ToolGroup title="物体" defaultExpanded={false}>
             <ToolBtn icon={<Circle size={18} />} active={dragMode === 'add_ball'} onClick={() => setDragMode('add_ball')} tooltip="小球 (Ball)" />
             <ToolBtn icon={<Disc size={18} />} active={dragMode === 'add_particle'} onClick={() => setDragMode('add_particle')} tooltip="质点 (Particle)" />
             <ToolBtn icon={<Square size={18} />} active={dragMode === 'add_box'} onClick={() => setDragMode('add_box')} tooltip="方块 (Block)" />
             <ToolBtn icon={<Triangle size={18} />} active={dragMode === 'add_triangle'} onClick={() => setDragMode('add_triangle')} tooltip="多边形 (Poly)" />
             <ToolBtn icon={<Layers size={18} />} active={dragMode === 'add_ramp'} onClick={() => setDragMode('add_ramp')} tooltip="斜面/地面 (Ramp/Plane)" />
             <ToolBtn icon={<Eclipse size={18} />} active={dragMode === 'add_arc'} onClick={() => setDragMode('add_arc')} tooltip="轨道 (Track)" />
             <ToolBtn icon={<MoveRight size={18} />} active={dragMode === 'add_conveyor'} onClick={() => setDragMode('add_conveyor')} tooltip="传送带 (Conveyor)" />
        </ToolGroup>

        <ToolGroup title="约束" defaultExpanded={false}>
             <ToolBtn icon={<Activity size={18} />} active={dragMode === 'add_spring'} onClick={() => setDragMode('add_spring')} tooltip="弹簧 (Spring)" />
             <ToolBtn icon={<Link size={18} />} active={dragMode === 'add_rod'} onClick={() => setDragMode('add_rod')} tooltip="杆 (Rod)" />
             <ToolBtn icon={<Zap size={18} />} active={dragMode === 'add_pin'} onClick={() => setDragMode('add_pin')} tooltip="铰链 (Pin)" />
        </ToolGroup>
        
        <ToolGroup title="场" defaultExpanded={false}>
             <ToolBtn icon={<Square size={18} />} active={dragMode === 'add_field_box'} onClick={() => setDragMode('add_field_box')} tooltip="矩形场 (Box Field)" />
             <ToolBtn icon={<Circle size={18} />} active={dragMode === 'add_field_circle'} onClick={() => setDragMode('add_field_circle')} tooltip="圆形场 (Circle Field)" />
             <ToolBtn icon={<Triangle size={18} />} active={dragMode === 'add_field_poly'} onClick={() => setDragMode('add_field_poly')} tooltip="多边形场 (Poly Field)" />
        </ToolGroup>

        <ToolGroup title="工具" defaultExpanded={false}>
             <ToolBtn icon={<Globe size={18} />} active={dragMode === 'tool_traj'} onClick={() => setDragMode('tool_traj')} tooltip="轨迹开关 (Trail Toggle)" />
             <ToolBtn icon={<Scan size={18} />} active={dragMode === 'tool_velocity'} onClick={() => setDragMode('tool_velocity')} tooltip="设置初速度 (Velocity)" />
             <ToolBtn icon={<Magnet size={18} />} active={dragMode === 'tool_force'} onClick={() => setDragMode('tool_force')} tooltip="设置恒力 (Force)" />
        </ToolGroup>

      </div>

      {/* CENTER: Canvas */}
      <div className="flex-1 flex flex-col relative min-w-0">
        <div className="h-14 bg-slate-900 border-b border-slate-800 flex items-center px-4 justify-between z-10 shadow-sm gap-4 overflow-x-auto">
            {/* Play Controls */}
            <div className="flex items-center space-x-2 shrink-0">
                <button 
                    onClick={handleTogglePause}
                    className={`flex items-center space-x-2 px-3 py-1.5 rounded-md font-medium text-sm transition shadow-lg ${state.paused ? 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-emerald-900/20' : 'bg-amber-600 hover:bg-amber-500 text-white shadow-amber-900/20'}`}
                >
                    {state.paused ? <Play size={16} fill="currentColor" /> : <Pause size={16} fill="currentColor" />}
                    <span className="hidden sm:inline">{state.paused ? "运行" : "暂停"}</span>
                </button>
                <button 
                    onClick={handleReset}
                    className="flex items-center space-x-2 px-3 py-1.5 rounded-md hover:bg-slate-800 text-slate-400 hover:text-white transition text-sm"
                    title="重置场景"
                >
                    <RotateCcw size={16} />
                </button>
            </div>

            {/* View Controls (Zoom/Reset) */}
             <div className="flex items-center space-x-2 bg-slate-800/30 px-2 py-1 rounded border border-slate-800 shrink-0">
                <button 
                    onClick={() => setZoom(state.camera.zoom - 0.2)}
                    className="p-1 hover:bg-slate-700 rounded text-slate-400 hover:text-white"
                >
                    <Minus size={14} />
                </button>
                <input 
                    type="range" min="0.1" max="5" step="0.1"
                    value={state.camera.zoom}
                    onChange={(e) => setZoom(parseFloat(e.target.value))}
                    className="w-16 sm:w-24 h-1 bg-slate-700 rounded-lg appearance-none cursor-pointer"
                />
                <button 
                    onClick={() => setZoom(state.camera.zoom + 0.2)}
                    className="p-1 hover:bg-slate-700 rounded text-slate-400 hover:text-white"
                >
                    <Plus size={14} />
                </button>
                <div className="w-px h-4 bg-slate-700 mx-1"></div>
                <button 
                    onClick={handleResetView}
                    className="p-1 hover:bg-slate-700 rounded text-slate-400 hover:text-white"
                    title="重置视图"
                >
                    <Scan size={14} />
                </button>
            </div>

            {/* Presets & Gravity */}
            <div className="hidden md:flex items-center space-x-4 bg-slate-800/30 px-3 py-1 rounded border border-slate-800 shrink-0">
                <div className="flex items-center space-x-2 border-r border-slate-700 pr-3">
                    <span className="text-[10px] text-slate-500 uppercase tracking-wider">重力 g</span>
                    <input 
                        type="range" min="0" max="30" step="1" 
                        value={state.gravity.y}
                        onChange={(e) => setState(s => ({...s, gravity: {x:0, y: Number(e.target.value)}}))}
                        className="w-16 lg:w-20 h-1 bg-slate-700 rounded-lg appearance-none cursor-pointer"
                    />
                    <span className="text-xs font-mono w-8 text-right">{state.gravity.y}</span>
                </div>
                
                <button onClick={() => setPreset('smooth')} className="text-xs flex items-center space-x-1 text-slate-400 hover:text-blue-400 transition" title="Friction = 0">
                    <Activity size={14} /> <span>光滑</span>
                </button>
                <button onClick={() => setPreset('elastic')} className="text-xs flex items-center space-x-1 text-slate-400 hover:text-purple-400 transition" title="Restitution = 1">
                    <Zap size={14} /> <span>弹性</span>
                </button>
            </div>

            {/* File Operations */}
            <div className="flex space-x-1 sm:space-x-2 shrink-0">
                 <button onClick={() => canvasRef.current?.exportImage()} className="p-2 text-slate-400 hover:text-white transition" title="截图">
                    <Camera size={18} />
                 </button>
                 <button onClick={handleSaveScene} className="p-2 text-slate-400 hover:text-white transition" title="保存">
                    <Download size={18} />
                 </button>
                 <button onClick={() => fileInputRef.current?.click()} className="p-2 text-slate-400 hover:text-white transition" title="导入">
                    <Upload size={18} />
                 </button>
            </div>
        </div>

        <div className="flex-1 relative bg-slate-950 overflow-hidden">
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
                arcCreation={arcBuilder ? { 
                    phase: arcBuilder.phase, 
                    center: arcBuilder.center, 
                    radius: arcBuilder.radius, 
                    endAngle: 0 
                } : null}
                rampCreation={rampBuilder ? { start: rampBuilder.start } : null}
                constraintBuilder={constraintBuilder}
             />
             
             {pinnedBodyId && (
                 <div className="absolute top-4 right-4 w-80 md:w-96 bg-slate-900/90 border border-slate-700 rounded-xl shadow-2xl backdrop-blur-sm z-20 overflow-hidden flex flex-col p-3 ring-1 ring-slate-700">
                    <div className="flex justify-between items-center mb-2">
                        <span className="text-xs font-bold text-slate-300 uppercase tracking-wider">固定视图 (Pinned)</span>
                    </div>
                    <DataCharts 
                        selectedBodyId={pinnedBodyId} 
                        state={state} 
                        isPinned={true}
                        onTogglePin={handleTogglePin}
                    />
                 </div>
             )}

             <div className="absolute top-4 left-4 pointer-events-none flex flex-col items-start space-y-2">
                 {dragMode === 'select_nodrag' && <div className="hud-badge bg-slate-600">选择模式 (不移动)</div>}
                 {dragMode === 'tool_traj' && <div className="hud-badge bg-blue-600">点击物体切换轨迹显示</div>}
                 {dragMode === 'tool_velocity' && <div className="hud-badge bg-blue-600">点击物体 -> 拖拽设置初速度</div>}
                 {dragMode === 'tool_force' && <div className="hud-badge bg-red-600">点击物体 -> 拖拽设置恒力</div>}
                 {dragMode.startsWith('add_field_') && <div className="hud-badge bg-emerald-600">拖拽绘制场区域</div>}
                 
                 {constraintBuilder && (
                     <div className="hud-badge bg-amber-600 animate-pulse">
                         点击第二个物体进行连接
                     </div>
                 )}
                 <div className="text-[10px] text-slate-600 font-mono">
                     Zoom: {(state.camera.zoom * 100).toFixed(0)}%
                 </div>
             </div>
        </div>
      </div>

      {/* RIGHT: Properties */}
      <div className="w-72 lg:w-80 bg-slate-900 border-l border-slate-800 flex flex-col z-10 shadow-2xl shrink-0">
        <div className="flex-1 overflow-y-auto custom-scrollbar">
            <PropertiesPanel 
                body={state.bodies.find(b => b.id === state.selectedBodyId) || null} 
                field={state.fields.find(f => f.id === state.selectedFieldId) || null}
                constraint={state.constraints.find(c => c.id === state.selectedConstraintId) || null}
                onUpdateBody={handleUpdateBody}
                onUpdateField={handleUpdateField}
                onUpdateConstraint={handleUpdateConstraint}
                onDeleteBody={handleDeleteBody}
                onDeleteField={handleDeleteField}
                onDeleteConstraint={handleDeleteConstraint}
            />
        </div>
        <div className="border-t border-slate-800 bg-slate-900 p-4 shrink-0">
            <DataCharts 
                selectedBodyId={state.selectedBodyId} 
                state={state} 
                isPinned={false}
                onTogglePin={handleTogglePin}
            />
        </div>
      </div>
    </div>
  );
};

const ToolGroup: React.FC<{title: string, children: React.ReactNode, defaultExpanded: boolean}> = ({ title, children, defaultExpanded }) => {
    const [expanded, setExpanded] = useState(defaultExpanded);
    return (
        <div className="w-full flex flex-col items-center gap-1">
            <button 
                onClick={() => setExpanded(!expanded)} 
                className="w-full flex items-center justify-center py-1 hover:bg-slate-800 rounded transition"
            >
                {expanded ? <ChevronDown size={12} className="text-slate-500" /> : <ChevronRight size={12} className="text-slate-500" />}
            </button>
            {expanded && (
                <div className="flex flex-col items-center gap-1 animate-in slide-in-from-top-2 duration-200">
                    {children}
                </div>
            )}
        </div>
    );
};

const ToolBtn: React.FC<{icon: React.ReactNode, active: boolean, onClick: () => void, tooltip: string, className?: string}> = ({ icon, active, onClick, tooltip, className }) => (
    <button 
        onClick={onClick}
        title={tooltip}
        className={`p-2 rounded-lg transition-all duration-200 group relative shrink-0 ${active ? 'bg-blue-600 text-white shadow-lg shadow-blue-900/50' : 'text-slate-400 hover:bg-slate-800 hover:text-slate-200'} ${className}`}
    >
        {icon}
        <span className="absolute left-14 top-1.5 bg-slate-800 text-white text-xs px-2 py-1 rounded opacity-0 group-hover:opacity-100 transition pointer-events-none whitespace-nowrap z-50 border border-slate-700 shadow-xl">
            {tooltip}
        </span>
    </button>
);

export default App;
