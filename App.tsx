
import React, { useState, useRef, useEffect, useCallback } from 'react';
import { Play, Pause, RotateCcw, MousePointer2, Circle, Square, Triangle, Camera, Download, Link, Eclipse, MoveRight, Upload, Zap, Activity, BarChart2, TrendingUp, Navigation, ArrowRight, Gauge, Layers, Plus, Minus, Scan } from 'lucide-react';
import SimulationCanvas, { CanvasRef } from './components/SimulationCanvas';
import PropertiesPanel from './components/PropertiesPanel';
import DataCharts from './components/DataCharts';
import { PhysicsEngine } from './services/physicsEngine';
import { BodyType, PhysicsBody, SimulationState, FieldType, Vector2, ConstraintType } from './types';
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
  camera: { x: window.innerWidth / 2, y: window.innerHeight / 2, zoom: 1 }
};

interface ArcBuilder {
    phase: 0 | 1 | 2; // 0: Set Center, 1: Set Radius, 2: Set Angle
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
  const [dragMode, setDragMode] = useState<'pan' | 'select' | 'add_circle' | 'add_box' | 'add_triangle' | 'add_arc' | 'add_conveyor' | 'add_ramp' | 'add_spring' | 'add_rod' | 'add_pin' | 'tool_traj' | 'tool_velocity' | 'tool_force'>('select');
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
  const handleResetView = () => setState(s => ({ ...s, camera: { x: window.innerWidth / 2, y: window.innerHeight / 2, zoom: 1 } }));
  
  const handleSelectBody = (id: string | null) => {
      // Logic to toggle trajectory if tool is active
      if (dragMode === 'tool_traj' && id) {
          setState(s => ({
              ...s,
              bodies: s.bodies.map(b => b.id === id ? { ...b, showTrajectory: !b.showTrajectory } : b)
          }));
      } else {
          setState(s => ({ ...s, selectedBodyId: id }));
      }
  };

  // Zoom Handler
  const handleZoom = (deltaY: number, mouseX: number, mouseY: number) => {
      setState(s => {
          const scale = deltaY > 0 ? 0.9 : 1.1;
          const newZoom = Math.max(0.1, Math.min(5, s.camera.zoom * scale));
          
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
          const newZoom = Math.max(0.1, Math.min(5, z));
          const cx = window.innerWidth / 2;
          const cy = window.innerHeight / 2;
          // Zoom towards center of screen approximately
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
                      velocity: { x: 0, y: 0 }, // Reset velocity
                      angularVelocity: 0,
                      trail: [] // Reset trail on move
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

  const handleAddBody = (pos: Vector2) => {
    // Ramp/Conveyor
    if (dragMode === 'add_ramp' || dragMode === 'add_conveyor') {
        if (!rampBuilder) {
            setRampBuilder({ start: pos, isConveyor: dragMode === 'add_conveyor' });
        } else {
            const start = rampBuilder.start;
            const end = pos;
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
                height: 5, // Thinner ramp
                color: rampBuilder.isConveyor ? '#8b5cf6' : '#94a3b8', 
                selected: true, showTrajectory: false, trail: [],
                showCharge: true
            };
            
            if (rampBuilder.isConveyor) {
                newBody.surfaceSpeed = 50; 
                newBody.height = 10; // Conveyors slightly thicker
            }

            setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
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
            
            // Check if user clicked back near the start point (Full Circle)
            // Use distance of mouse to start point
            const startPoint = {
                x: arcBuilder.center.x + arcBuilder.radius * Math.cos(arcBuilder.startAngle),
                y: arcBuilder.center.y + arcBuilder.radius * Math.sin(arcBuilder.startAngle)
            };
            const distToStart = Vec2.dist(pos, startPoint);
            
            let finalEndAngle = angle;
            if (distToStart < 20) {
                // If close to start, make it a full loop
                finalEndAngle = arcBuilder.startAngle + 2 * Math.PI;
            } else {
                // Normalize for standard arc drawing direction if needed, but simplistic is fine
                // Ensure end > start for simplicity in rendering if we want counter-clockwise
                if (finalEndAngle < arcBuilder.startAngle) finalEndAngle += 2*Math.PI;
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
                showCharge: true
            };
            
            setState(s => ({ ...s, bodies: [...s.bodies, newBody], selectedBodyId: newBody.id }));
            setArcBuilder(null);
            setDragMode('select');
        }
        return;
    }

    // Constraint Creation Logic (Spring, Rod)
    if (dragMode === 'add_spring' || dragMode === 'add_rod' || dragMode === 'add_pin') {
        const clickedBody = state.bodies.slice().reverse().find(b => {
             if (b.type === BodyType.CIRCLE) return Vec2.dist(pos, b.position) < (b.radius || 20);
             if (b.type === BodyType.BOX) {
                 const w = b.width || 40; const h = b.height || 40;
                 return pos.x > b.position.x - w/2 && pos.x < b.position.x + w/2 && pos.y > b.position.y - h/2 && pos.y < b.position.y + h/2;
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
                     
                     // Use Center of Mass anchor
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

    // Do not create body if in a tool mode
    if (dragMode.startsWith('tool_')) return;

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
    } else {
        newBody.radius = 20;
    }
    
    setState(s => ({
        ...s,
        bodies: [...s.bodies, newBody],
        selectedBodyId: newId
    }));
    setDragMode('select'); 
  };

  const handleUpdateBody = (id: string, updates: Partial<PhysicsBody>) => {
    setState(s => ({
        ...s,
        bodies: s.bodies.map(b => b.id === id ? { ...b, ...updates } : b)
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
      <div className="w-16 bg-slate-900 border-r border-slate-800 flex flex-col items-center py-4 gap-2 z-10 shadow-xl overflow-y-auto custom-scrollbar">
        <div className="mb-2 shrink-0">
             <div className="w-10 h-10 bg-gradient-to-br from-blue-600 to-indigo-600 rounded-xl flex items-center justify-center font-bold text-white text-sm shadow-lg shadow-blue-900/20">Ph</div>
        </div>

        {/* General */}
        <div className="w-full flex flex-col items-center gap-1">
             <div className="text-[9px] text-slate-500 font-bold uppercase tracking-wider mb-1 scale-75">通用</div>
             <ToolBtn icon={<MousePointer2 size={18} />} active={dragMode === 'select'} onClick={setClear} tooltip="选择 (Select)" />
             <ToolBtn icon={<Navigation size={18} />} active={dragMode === 'pan'} onClick={() => setDragMode('pan')} tooltip="平移 (Pan)" />
        </div>
        
        <div className="w-8 h-px bg-slate-800 my-1 shrink-0"></div>

        {/* Objects */}
        <div className="w-full flex flex-col items-center gap-1">
             <div className="text-[9px] text-slate-500 font-bold uppercase tracking-wider mb-1 scale-75">物体</div>
             <ToolBtn icon={<Circle size={18} />} active={dragMode === 'add_circle'} onClick={() => setDragMode('add_circle')} tooltip="小球 (Ball)" />
             <ToolBtn icon={<Square size={18} />} active={dragMode === 'add_box'} onClick={() => setDragMode('add_box')} tooltip="方块 (Block)" />
             <ToolBtn icon={<Triangle size={18} />} active={dragMode === 'add_triangle'} onClick={() => setDragMode('add_triangle')} tooltip="多边形 (Poly)" />
             <ToolBtn icon={<Layers size={18} />} active={dragMode === 'add_ramp'} onClick={() => setDragMode('add_ramp')} tooltip="斜面/地面 (Ramp/Plane)" />
             <ToolBtn icon={<Eclipse size={18} />} active={dragMode === 'add_arc'} onClick={() => setDragMode('add_arc')} tooltip="轨道 (Track)" />
             <ToolBtn icon={<MoveRight size={18} />} active={dragMode === 'add_conveyor'} onClick={() => setDragMode('add_conveyor')} tooltip="传送带 (Conveyor)" />
        </div>
        
        <div className="w-8 h-px bg-slate-800 my-1 shrink-0"></div>
        
        {/* Constraints */}
        <div className="w-full flex flex-col items-center gap-1">
             <div className="text-[9px] text-slate-500 font-bold uppercase tracking-wider mb-1 scale-75">约束</div>
             <ToolBtn icon={<Activity size={18} />} active={dragMode === 'add_spring'} onClick={() => setDragMode('add_spring')} tooltip="弹簧 (Spring)" />
             <ToolBtn icon={<Link size={18} />} active={dragMode === 'add_rod'} onClick={() => setDragMode('add_rod')} tooltip="杆 (Rod)" />
             <ToolBtn icon={<Zap size={18} />} active={dragMode === 'add_pin'} onClick={() => setDragMode('add_pin')} tooltip="铰链 (Pin)" />
        </div>

        <div className="w-8 h-px bg-slate-800 my-1 shrink-0"></div>

        {/* Research */}
        <div className="w-full flex flex-col items-center gap-1">
             <div className="text-[9px] text-slate-500 font-bold uppercase tracking-wider mb-1 scale-75">研究</div>
             <ToolBtn icon={<TrendingUp size={18} />} active={dragMode === 'tool_traj'} onClick={() => setDragMode('tool_traj')} tooltip="轨迹开关 (Trail Toggle)" />
             <ToolBtn icon={<ArrowRight size={18} />} active={dragMode === 'tool_velocity'} onClick={() => setDragMode('tool_velocity')} tooltip="设置初速度 (Velocity)" />
             <ToolBtn icon={<Gauge size={18} />} active={dragMode === 'tool_force'} onClick={() => setDragMode('tool_force')} tooltip="设置恒力 (Force)" />
        </div>
      </div>

      {/* CENTER: Canvas */}
      <div className="flex-1 flex flex-col relative">
        <div className="h-14 bg-slate-900 border-b border-slate-800 flex items-center px-4 justify-between z-10 shadow-sm gap-4">
            {/* Play Controls */}
            <div className="flex items-center space-x-2">
                <button 
                    onClick={handleTogglePause}
                    className={`flex items-center space-x-2 px-3 py-1.5 rounded-md font-medium text-sm transition shadow-lg ${state.paused ? 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-emerald-900/20' : 'bg-amber-600 hover:bg-amber-500 text-white shadow-amber-900/20'}`}
                >
                    {state.paused ? <Play size={16} fill="currentColor" /> : <Pause size={16} fill="currentColor" />}
                    <span>{state.paused ? "运行" : "暂停"}</span>
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
             <div className="flex items-center space-x-2 bg-slate-800/30 px-2 py-1 rounded border border-slate-800">
                <button 
                    onClick={() => setZoom(state.camera.zoom - 0.2)}
                    className="p-1 hover:bg-slate-700 rounded text-slate-400 hover:text-white"
                >
                    <Minus size={14} />
                </button>
                <input 
                    type="range" min="0.1" max="3" step="0.1"
                    value={state.camera.zoom}
                    onChange={(e) => setZoom(parseFloat(e.target.value))}
                    className="w-24 h-1 bg-slate-700 rounded-lg appearance-none cursor-pointer"
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
            <div className="flex items-center space-x-4 bg-slate-800/30 px-3 py-1 rounded border border-slate-800">
                <div className="flex items-center space-x-2 border-r border-slate-700 pr-3">
                    <span className="text-[10px] text-slate-500 uppercase tracking-wider">重力 g</span>
                    <input 
                        type="range" min="0" max="30" step="1" 
                        value={state.gravity.y}
                        onChange={(e) => setState(s => ({...s, gravity: {x:0, y: Number(e.target.value)}}))}
                        className="w-20 h-1 bg-slate-700 rounded-lg appearance-none cursor-pointer"
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
            <div className="flex space-x-2">
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
                onAddBody={handleAddBody}
                onMoveBody={handleBodyMove}
                onVectorEdit={handleVectorEdit}
                dragMode={dragMode}
                onZoom={handleZoom}
                arcCreation={arcBuilder ? { 
                    phase: arcBuilder.phase, 
                    center: arcBuilder.center, 
                    radius: arcBuilder.radius, 
                    endAngle: 0 
                } : null}
                rampCreation={rampBuilder ? { start: rampBuilder.start } : null}
                constraintBuilder={constraintBuilder}
             />
             
             {/* Pinned Chart Overlay */}
             {pinnedBodyId && (
                 <div className="absolute top-4 right-4 w-96 bg-slate-900/90 border border-slate-700 rounded-xl shadow-2xl backdrop-blur-sm z-20 overflow-hidden flex flex-col p-3 ring-1 ring-slate-700">
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

             {/* HUD Messages */}
             <div className="absolute top-4 left-4 pointer-events-none flex flex-col items-start space-y-2">
                 {dragMode === 'tool_traj' && <div className="hud-badge bg-blue-600">点击物体切换轨迹显示</div>}
                 {dragMode === 'tool_velocity' && <div className="hud-badge bg-blue-600">拖拽物体设置初速度</div>}
                 {dragMode === 'tool_force' && <div className="hud-badge bg-red-600">拖拽物体设置恒力</div>}
                 
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
      <div className="w-80 bg-slate-900 border-l border-slate-800 flex flex-col z-10 shadow-2xl">
        <div className="flex-1 overflow-y-auto">
            <PropertiesPanel 
                body={state.bodies.find(b => b.id === state.selectedBodyId) || null} 
                onUpdate={handleUpdateBody}
                onDelete={handleDeleteBody}
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

const ToolBtn: React.FC<{icon: React.ReactNode, active: boolean, onClick: () => void, tooltip: string}> = ({ icon, active, onClick, tooltip }) => (
    <button 
        onClick={onClick}
        title={tooltip}
        className={`p-2 rounded-lg transition-all duration-200 group relative shrink-0 ${active ? 'bg-blue-600 text-white shadow-lg shadow-blue-900/50' : 'text-slate-400 hover:bg-slate-800 hover:text-slate-200'}`}
    >
        {icon}
        <span className="absolute left-14 top-1.5 bg-slate-800 text-white text-xs px-2 py-1 rounded opacity-0 group-hover:opacity-100 transition pointer-events-none whitespace-nowrap z-50 border border-slate-700 shadow-xl">
            {tooltip}
        </span>
    </button>
);

export default App;
