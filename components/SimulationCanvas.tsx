
import React, { useRef, useEffect, useState, useImperativeHandle, forwardRef } from 'react';
import { BodyType, PhysicsBody, SimulationState, FieldType, Vector2, ConstraintType } from '../types';
import { Vec2 } from '../services/vectorMath';

interface Props {
  state: SimulationState;
  onSelectBody: (id: string | null) => void;
  onAddBody: (position: Vector2) => void;
  onMoveBody: (id: string, position: Vector2) => void;
  onVectorEdit: (id: string, vectorType: 'velocity' | 'force', vector: Vector2) => void;
  dragMode: string;
  onZoom: (delta: number, mouseX: number, mouseY: number) => void;
  arcCreation: { phase: number, center: Vector2, radius: number, endAngle: number } | null;
  rampCreation: { start: Vector2 } | null;
  constraintBuilder: string | null; // ID of the first body
}

export interface CanvasRef {
  exportImage: () => void;
}

const SimulationCanvas = forwardRef<CanvasRef, Props>(({ state, onSelectBody, onAddBody, onMoveBody, onVectorEdit, dragMode, onZoom, arcCreation, rampCreation, constraintBuilder }, ref) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isDraggingCam, setIsDraggingCam] = useState(false);
  const [draggedBodyId, setDraggedBodyId] = useState<string | null>(null);
  const [dragStart, setDragStart] = useState<Vector2>({ x: 0, y: 0 });
  
  // Vector Editing State
  const [vectorEditBodyId, setVectorEditBodyId] = useState<string | null>(null);
  
  const [mousePos, setMousePos] = useState<Vector2>({ x: 0, y: 0 });
  const [snappedMousePos, setSnappedMousePos] = useState<Vector2>({ x: 0, y: 0 });

  useImperativeHandle(ref, () => ({
    exportImage: () => {
        if (!canvasRef.current) return;
        const link = document.createElement('a');
        link.download = `physlab-snapshot-${Date.now()}.png`;
        link.href = canvasRef.current.toDataURL();
        link.click();
    }
  }));

  // Helper to convert screen to world
  const screenToWorld = (sx: number, sy: number) => {
     return {
         x: (sx - state.camera.x) / state.camera.zoom,
         y: (sy - state.camera.y) / state.camera.zoom
     };
  };

  const snapToGrid = (v: Vector2): Vector2 => {
      const GRID = 20;
      const SNAP_THRESHOLD = 8; // Weak snap: only snap if within 8px
      
      const snappedX = Math.round(v.x / GRID) * GRID;
      const snappedY = Math.round(v.y / GRID) * GRID;
      
      const res = { x: v.x, y: v.y };

      if (Math.abs(v.x - snappedX) < SNAP_THRESHOLD) {
          res.x = snappedX;
      }
      if (Math.abs(v.y - snappedY) < SNAP_THRESHOLD) {
          res.y = snappedY;
      }
      return res;
  };

  // Rendering Loop
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);

    const render = () => {
      // Clear
      ctx.fillStyle = '#0f172a'; // Slate-950
      ctx.fillRect(0, 0, rect.width, rect.height);

      ctx.save();
      // Apply Camera
      ctx.translate(state.camera.x, state.camera.y);
      ctx.scale(state.camera.zoom, state.camera.zoom);

      // Grid & Axis
      drawGrid(ctx, rect.width, rect.height);

      // Fields
      state.fields.forEach(field => {
        if (!field.visible) return;
        ctx.fillStyle = field.type === FieldType.UNIFORM_ELECTRIC ? 'rgba(239, 68, 68, 0.1)' 
                      : field.type === FieldType.UNIFORM_MAGNETIC ? 'rgba(59, 130, 246, 0.1)'
                      : 'rgba(16, 185, 129, 0.1)';
        ctx.strokeStyle = field.type === FieldType.UNIFORM_ELECTRIC ? '#ef4444' 
                        : field.type === FieldType.UNIFORM_MAGNETIC ? '#3b82f6' : '#10b981';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);
        ctx.fillRect(field.position.x, field.position.y, field.size.x, field.size.y);
        ctx.strokeRect(field.position.x, field.position.y, field.size.x, field.size.y);
        ctx.setLineDash([]);
        
        ctx.fillStyle = ctx.strokeStyle;
        ctx.font = '12px sans-serif';
        ctx.fillText(field.type === FieldType.UNIFORM_MAGNETIC ? '磁场 (B-Field)' : '电场 (E-Field)', field.position.x + 5, field.position.y + 15);
      });

      // Trails (Render first so they are behind bodies)
      state.bodies.forEach(body => {
          if (body.showTrajectory && body.trail && body.trail.length > 1) {
              ctx.beginPath();
              ctx.moveTo(body.trail[0].x, body.trail[0].y);
              for(let i=1; i<body.trail.length; i++) {
                  ctx.lineTo(body.trail[i].x, body.trail[i].y);
              }
              ctx.strokeStyle = body.color;
              ctx.globalAlpha = 0.5;
              ctx.lineWidth = 1;
              ctx.stroke();
              ctx.globalAlpha = 1.0;
          }
      });

      // Bodies
      state.bodies.forEach(body => {
        ctx.save();
        ctx.translate(body.position.x, body.position.y);

        // Analysis Vectors (Before Rotation or separate)
        // We draw these before rotating the context for the body so they are axis-aligned if needed, 
        // but typically vectors start at center. Let's draw them after translation but before body rotation 
        // so they are not rotated with body (usually vectors are world space).
        // Actually, simpler to draw body first then vectors on top.
        
        ctx.save();
        ctx.rotate(body.angle);

        ctx.fillStyle = body.color;
        ctx.lineWidth = body.id === state.selectedBodyId ? 3 : 2;
        ctx.strokeStyle = body.id === state.selectedBodyId ? '#ffffff' : '#000000';
        
        // Shadow for depth
        ctx.shadowColor = 'rgba(0,0,0,0.5)';
        ctx.shadowBlur = 10;
        ctx.shadowOffsetX = 2;
        ctx.shadowOffsetY = 2;

        if (body.type === BodyType.CIRCLE) {
          ctx.beginPath();
          ctx.arc(0, 0, body.radius || 20, 0, Math.PI * 2);
          ctx.fill();
          ctx.shadowColor = 'transparent'; // Reset shadow for stroke
          ctx.stroke();
          
          // Orientation Line
          ctx.beginPath();
          ctx.moveTo(0, 0);
          ctx.lineTo((body.radius || 20) * 0.8, 0);
          ctx.strokeStyle = 'rgba(0,0,0,0.3)';
          ctx.lineWidth = 2;
          ctx.stroke();

        } else if (body.type === BodyType.BOX) {
          const w = body.width || 40;
          const h = body.height || 40;
          const isRamp = h < 12 && w > h * 2; // Heuristic for ramp (thinner threshold)

          if (isRamp && body.mass === 0) {
              // Draw as "Plane-like" Ramp (Thinner look)
              ctx.shadowColor = 'transparent';
              ctx.beginPath();
              ctx.rect(-w/2, -h/2, w, h);
              ctx.fillStyle = body.color; 
              ctx.fill();
              
              // Draw top line bold
              ctx.beginPath();
              ctx.moveTo(-w/2, -h/2);
              ctx.lineTo(w/2, -h/2);
              ctx.strokeStyle = '#fff';
              ctx.lineWidth = 2;
              ctx.stroke();

          } else {
              // Standard Box
              ctx.fillRect(-w/2, -h/2, w, h);
              ctx.shadowColor = 'transparent';
              ctx.strokeRect(-w/2, -h/2, w, h);
          }

          // Conveyor Belt Visualization
          if (body.surfaceSpeed && body.surfaceSpeed !== 0) {
             const dir = Math.sign(body.surfaceSpeed);
             ctx.strokeStyle = 'rgba(255,255,255,0.4)';
             ctx.lineWidth = 2;
             ctx.setLineDash([5, 5]);
             ctx.lineDashOffset = -state.time * 20 * dir; // Animate
             ctx.beginPath();
             ctx.moveTo(-w/2, -h/2); ctx.lineTo(w/2, -h/2);
             ctx.moveTo(-w/2, h/2); ctx.lineTo(w/2, h/2);
             ctx.stroke();
             ctx.setLineDash([]);

             // Arrows
             ctx.fillStyle = 'rgba(255,255,255,0.4)';
             for(let x = -w/2 + 10; x < w/2; x+=20) {
                 ctx.beginPath();
                 ctx.moveTo(x, -h/2 - 5);
                 ctx.lineTo(x + 5 * dir, -h/2 - 5);
                 ctx.lineTo(x, -h/2 - 8);
                 ctx.fill();
             }
          }

        } else if (body.type === BodyType.POLYGON) {
          if (body.vertices && body.vertices.length > 0) {
              ctx.beginPath();
              ctx.moveTo(body.vertices[0].x, body.vertices[0].y);
              for (let i = 1; i < body.vertices.length; i++) {
                  ctx.lineTo(body.vertices[i].x, body.vertices[i].y);
              }
              ctx.closePath();
              ctx.fill();
              ctx.shadowColor = 'transparent';
              ctx.stroke();
          }
        } else if (body.type === BodyType.PLANE) {
          ctx.shadowColor = 'transparent';
          ctx.beginPath();
          ctx.moveTo(-10000, 0);
          ctx.lineTo(10000, 0);
          ctx.strokeStyle = body.color;
          ctx.lineWidth = 2;
          ctx.stroke();
          
          for(let i=-2000; i<2000; i+=25) {
              ctx.beginPath();
              ctx.moveTo(i, 0);
              ctx.lineTo(i - 10, 15);
              ctx.strokeStyle = 'rgba(255,255,255,0.1)';
              ctx.lineWidth = 1;
              ctx.stroke();
          }
        } else if (body.type === BodyType.ARC) {
             const r = body.radius || 100;
             const start = body.arcStartAngle || 0;
             const end = body.arcEndAngle || Math.PI;
             
             ctx.shadowColor = 'transparent';
             ctx.beginPath();
             ctx.arc(0, 0, r, start, end);
             ctx.strokeStyle = body.color;
             ctx.lineWidth = 5;
             ctx.stroke();
             
             // Draw inner ticks to show solid side
             ctx.strokeStyle = 'rgba(255,255,255,0.2)';
             ctx.lineWidth = 1;
             const steps = 20;
             for (let i=0; i<=steps; i++) {
                 const a = start + (end-start)*(i/steps);
                 const x1 = Math.cos(a) * r;
                 const y1 = Math.sin(a) * r;
                 const x2 = Math.cos(a) * (r + 10); // Outward ticks
                 const y2 = Math.sin(a) * (r + 10);
                 ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
             }
        }
        
        // Show Charge
        if (body.showCharge && body.charge !== 0) {
             ctx.fillStyle = body.charge > 0 ? 'rgba(239, 68, 68, 0.8)' : 'rgba(59, 130, 246, 0.8)';
             ctx.font = 'bold 20px monospace';
             ctx.textAlign = 'center';
             ctx.textBaseline = 'middle';
             ctx.fillText(body.charge > 0 ? '+' : '-', 0, 0);
        }

        ctx.restore(); // End Body Rotation

        // Draw Analysis Vectors (World Space)
        if (body.showVelocity && Vec2.magSq(body.velocity) > 0.1) {
            drawVector(ctx, body.velocity, '#3b82f6', 'v', 1.0);
        }
        if (body.showAcceleration && Vec2.magSq(body.acceleration) > 0.1) {
            drawVector(ctx, body.acceleration, '#a855f7', 'a', 0.5); // Scale accel slightly up visually if needed
        }
        if (body.showForce && Vec2.magSq(body.force) > 0.1) {
            drawVector(ctx, body.force, '#ef4444', 'F', 0.1); // Force can be large, scale down
        }

        ctx.restore();
      });

      // Constraints (Draw on top)
      state.constraints.forEach(c => {
        const bodyA = state.bodies.find(b => b.id === c.bodyAId);
        const bodyB = state.bodies.find(b => b.id === c.bodyBId);
        if (bodyA && bodyB) {
            // Calculate world points
            const pA = Vec2.transform(c.localAnchorA, bodyA.position, bodyA.angle);
            const pB = Vec2.transform(c.localAnchorB, bodyB.position, bodyB.angle);

            ctx.beginPath();
            ctx.moveTo(pA.x, pA.y);
            ctx.lineTo(pB.x, pB.y);
            ctx.lineWidth = c.type === ConstraintType.SPRING ? 3 : 5;
            ctx.strokeStyle = c.type === ConstraintType.SPRING ? '#fbbf24' : '#94a3b8'; 
            
            if (c.type === ConstraintType.SPRING) {
                drawSpring(ctx, pA, pB, 6);
            } else if (c.type === ConstraintType.ROD) {
                ctx.stroke();
                // Draw dots at ends
                ctx.fillStyle = '#94a3b8';
                ctx.beginPath(); ctx.arc(pA.x, pA.y, 4, 0, Math.PI*2); ctx.fill();
                ctx.beginPath(); ctx.arc(pB.x, pB.y, 4, 0, Math.PI*2); ctx.fill();
            } else if (c.type === ConstraintType.PIN) {
                ctx.fillStyle = '#f59e0b';
                ctx.beginPath(); ctx.arc(pA.x, pA.y, 6, 0, Math.PI*2); ctx.fill();
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 2;
                ctx.stroke();
            }
        }
      });

      // Constraint Builder Preview Line
      if (constraintBuilder) {
          const bodyA = state.bodies.find(b => b.id === constraintBuilder);
          if (bodyA) {
              ctx.save();
              ctx.beginPath();
              ctx.moveTo(bodyA.position.x, bodyA.position.y);
              ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              ctx.strokeStyle = '#fbbf24';
              ctx.lineWidth = 2;
              ctx.setLineDash([5, 5]);
              ctx.stroke();
              
              // End point dot
              ctx.fillStyle = '#fbbf24';
              ctx.beginPath(); 
              ctx.arc(snappedMousePos.x, snappedMousePos.y, 4, 0, Math.PI*2); 
              ctx.fill();
              ctx.restore();
          }
      }

      // Vector Editing Visualization (Velocity / Force)
      if (vectorEditBodyId) {
          const body = state.bodies.find(b => b.id === vectorEditBodyId);
          if (body) {
              ctx.save();
              ctx.translate(body.position.x, body.position.y);
              
              const isForce = dragMode === 'tool_force';
              const targetPos = Vec2.sub(mousePos, body.position);
              
              // Draw Arrow
              ctx.beginPath();
              ctx.moveTo(0, 0);
              ctx.lineTo(targetPos.x, targetPos.y);
              ctx.strokeStyle = isForce ? '#ef4444' : '#3b82f6';
              ctx.lineWidth = 3;
              ctx.stroke();
              
              // Arrow Head
              const angle = Math.atan2(targetPos.y, targetPos.x);
              const headLen = 10;
              ctx.beginPath();
              ctx.moveTo(targetPos.x, targetPos.y);
              ctx.lineTo(targetPos.x - headLen * Math.cos(angle - Math.PI / 6), targetPos.y - headLen * Math.sin(angle - Math.PI / 6));
              ctx.lineTo(targetPos.x - headLen * Math.cos(angle + Math.PI / 6), targetPos.y - headLen * Math.sin(angle + Math.PI / 6));
              ctx.lineTo(targetPos.x, targetPos.y);
              ctx.fillStyle = isForce ? '#ef4444' : '#3b82f6';
              ctx.fill();

              // Text Value
              const mag = isForce ? Vec2.mag(targetPos) * 10 : Vec2.mag(targetPos); // Scale for display
              ctx.fillStyle = '#fff';
              ctx.font = '12px monospace';
              const label = isForce ? `F: ${mag.toFixed(0)} N` : `v: ${mag.toFixed(1)} m/s`;
              ctx.fillText(label, targetPos.x + 10, targetPos.y);

              ctx.restore();
          }
      }

      // Arc Creation Overlay
      if (arcCreation) {
          ctx.save();
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2;
          ctx.setLineDash([5, 5]);

          if (arcCreation.phase >= 1) {
              const currentRadius = arcCreation.phase === 1 
                  ? Vec2.dist(arcCreation.center, snappedMousePos) 
                  : arcCreation.radius;
                  
              // Draw Radius Line
              ctx.beginPath();
              ctx.moveTo(arcCreation.center.x, arcCreation.center.y);
              ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI * 2);
              ctx.stroke();
              
              // Center point
              ctx.fillStyle = '#38bdf8';
              ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, 4, 0, Math.PI*2); ctx.fill();

              if (arcCreation.phase === 2) {
                   // Draw temporary arc to mouse
                   const dx = snappedMousePos.x - arcCreation.center.x;
                   const dy = snappedMousePos.y - arcCreation.center.y;
                   const mouseAngle = Math.atan2(dy, dx);

                   ctx.fillStyle = 'rgba(56, 189, 248, 0.2)';
                   ctx.beginPath();
                   ctx.moveTo(arcCreation.center.x, arcCreation.center.y);
                   ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, mouseAngle);
                   ctx.lineTo(arcCreation.center.x, arcCreation.center.y);
                   ctx.fill();
              }
          }
          ctx.restore();
      }

      // Ramp/Conveyor Creation Overlay with Angle Guides
      if (rampCreation) {
          ctx.save();
          // Draw horizontal/vertical reference lines
          ctx.beginPath();
          ctx.strokeStyle = 'rgba(255,255,255,0.2)';
          ctx.lineWidth = 1;
          ctx.setLineDash([2, 4]);
          // Horizontal
          ctx.moveTo(rampCreation.start.x - 500, rampCreation.start.y);
          ctx.lineTo(rampCreation.start.x + 500, rampCreation.start.y);
          // Vertical
          ctx.moveTo(rampCreation.start.x, rampCreation.start.y - 500);
          ctx.lineTo(rampCreation.start.x, rampCreation.start.y + 500);
          ctx.stroke();

          // Angle Snap Guide
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2;
          ctx.setLineDash([5, 5]);
          ctx.beginPath();
          ctx.moveTo(rampCreation.start.x, rampCreation.start.y);
          ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
          ctx.stroke();
          
          ctx.fillStyle = '#38bdf8';
          ctx.beginPath(); ctx.arc(rampCreation.start.x, rampCreation.start.y, 4, 0, Math.PI*2); ctx.fill();
          ctx.beginPath(); ctx.arc(snappedMousePos.x, snappedMousePos.y, 4, 0, Math.PI*2); ctx.fill();
          
          // Show Angle Text
          const delta = Vec2.sub(snappedMousePos, rampCreation.start);
          const angleRad = Math.atan2(delta.y, delta.x);
          const angleDeg = (angleRad * 180 / Math.PI).toFixed(1);
          ctx.font = '12px monospace';
          ctx.fillStyle = '#fff';
          ctx.fillText(`${angleDeg}°`, snappedMousePos.x + 10, snappedMousePos.y - 10);
          
          ctx.restore();
      }
      
      // Cursor Snap Indicator
      if (dragMode.startsWith('add_') || draggedBodyId) {
          ctx.save();
          ctx.strokeStyle = 'rgba(255,255,255,0.5)';
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(snappedMousePos.x - 5, snappedMousePos.y);
          ctx.lineTo(snappedMousePos.x + 5, snappedMousePos.y);
          ctx.moveTo(snappedMousePos.x, snappedMousePos.y - 5);
          ctx.lineTo(snappedMousePos.x, snappedMousePos.y + 5);
          ctx.stroke();
          ctx.restore();
      }

      ctx.restore();
    };

    render();
  }, [state, mousePos, snappedMousePos, arcCreation, rampCreation, draggedBodyId, dragMode, constraintBuilder, vectorEditBodyId]);

  const drawVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number) => {
      // Vector in world units, may need scaling for visualization
      const visualVec = Vec2.mul(vec, scale); 
      const len = Vec2.mag(visualVec);
      if (len < 5) return; // Too small to draw

      ctx.save();
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(visualVec.x, visualVec.y);
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.stroke();

      // Arrow Head
      const angle = Math.atan2(visualVec.y, visualVec.x);
      const headLen = 8;
      ctx.beginPath();
      ctx.moveTo(visualVec.x, visualVec.y);
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle - Math.PI / 6), visualVec.y - headLen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle + Math.PI / 6), visualVec.y - headLen * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();

      // Label
      ctx.fillStyle = '#fff';
      ctx.font = '10px monospace';
      ctx.fillText(label, visualVec.x + 5, visualVec.y + 5);
      ctx.restore();
  };

  const drawGrid = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
    // Simple infinite grid illusion
    const gridSize = 100;
    const farLeft = ((-state.camera.x / state.camera.zoom) - 1000);
    const farTop = ((-state.camera.y / state.camera.zoom) - 1000);
    const w = 4000 / state.camera.zoom;
    const h = 4000 / state.camera.zoom;
    
    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth = 1;
    
    // Verticals
    for(let x = Math.floor(farLeft/gridSize)*gridSize; x < farLeft+w; x+=gridSize) {
        ctx.beginPath(); ctx.moveTo(x, farTop); ctx.lineTo(x, farTop+h); ctx.stroke();
        
        // Axis Labels (X)
        if (x % (gridSize * 2) === 0) {
            ctx.fillStyle = 'rgba(255,255,255,0.3)';
            ctx.font = '10px monospace';
            ctx.fillText(x.toFixed(0), x + 2, farTop + h/2 + 10);
        }
    }
    // Horizontals
    for(let y = Math.floor(farTop/gridSize)*gridSize; y < farTop+h; y+=gridSize) {
        ctx.beginPath(); ctx.moveTo(farLeft, y); ctx.lineTo(farLeft+w, y); ctx.stroke();
        
        // Axis Labels (Y)
        if (y % (gridSize * 2) === 0) {
            ctx.fillStyle = 'rgba(255,255,255,0.3)';
            ctx.font = '10px monospace';
            ctx.fillText((-y).toFixed(0), farLeft + w/2 + 5, y - 2); 
        }
    }
    
    // Main Axis
    ctx.strokeStyle = 'rgba(255,255,255,0.3)';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(farLeft, 0); ctx.lineTo(farLeft+w, 0); ctx.stroke(); // X
    ctx.beginPath(); ctx.moveTo(0, farTop); ctx.lineTo(0, farTop+h); ctx.stroke(); // Y
    
    // Draw Axis Numbers relative to 0,0
    ctx.fillStyle = '#fff';
    ctx.font = '10px monospace';
    // X Axis numbers
    for (let x = Math.floor(farLeft/gridSize)*gridSize; x < farLeft+w; x+=gridSize) {
        if (x === 0) continue;
        ctx.fillText(x.toString(), x, 12);
    }
    // Y Axis numbers
    for (let y = Math.floor(farTop/gridSize)*gridSize; y < farTop+h; y+=gridSize) {
        if (y === 0) continue;
        ctx.fillText((-y).toString(), 4, y);
    }
    ctx.fillText("0", 4, -4);
  };

  const drawSpring = (ctx: CanvasRenderingContext2D, p1: Vector2, p2: Vector2, width: number) => {
     const dist = Vec2.dist(p1, p2);
     const dir = Vec2.normalize(Vec2.sub(p2, p1));
     const perp = Vec2.perp(dir);
     const numCoils = 12;
     
     ctx.beginPath();
     ctx.moveTo(p1.x, p1.y);
     for(let i=0; i<=numCoils; i++) {
        const t = i / numCoils;
        const p = Vec2.add(Vec2.mul(p1, 1-t), Vec2.mul(p2, t));
        let offset = 0;
        if (i > 0 && i < numCoils) offset = i % 2 === 0 ? width : -width;
        
        const zig = Vec2.add(p, Vec2.mul(perp, offset));
        ctx.lineTo(zig.x, zig.y);
     }
     ctx.stroke();
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    const rect = canvasRef.current!.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const worldPos = screenToWorld(sx, sy);
    const snapped = snapToGrid(worldPos);

    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin'].includes(dragMode)) {
        // Use snapped pos for creation
        onAddBody(snapped);
        return;
    }

    // Hit Testing
    let clickedBodyId: string | null = null;
    // Reverse iterate for z-index
    for(let i=state.bodies.length-1; i>=0; i--) {
        const b = state.bodies[i];
        if (b.type === BodyType.CIRCLE) {
            if (Vec2.dist(worldPos, b.position) < (b.radius || 20)) {
                clickedBodyId = b.id;
                break;
            }
        } else if (b.type === BodyType.BOX || b.type === BodyType.POLYGON) {
            // Simple bounding check for selection
            const w = b.width || 40;
            const h = b.height || 40;
             if (worldPos.x > b.position.x - w/2 && worldPos.x < b.position.x + w/2 &&
                worldPos.y > b.position.y - h/2 && worldPos.y < b.position.y + h/2) {
                clickedBodyId = b.id;
                break;
            }
        } else if (b.type === BodyType.ARC) {
             const dist = Vec2.dist(worldPos, b.position);
             const r = b.radius || 100;
             if (Math.abs(dist - r) < 10) {
                 clickedBodyId = b.id;
                 break;
             }
        }
    }

    if (clickedBodyId) {
        onSelectBody(clickedBodyId);

        // Vector Tools Logic
        if (dragMode === 'tool_velocity' || dragMode === 'tool_force') {
            setVectorEditBodyId(clickedBodyId);
            return;
        }

        // Constraint Creation Pass-through
        if (['add_spring', 'add_rod', 'add_pin'].includes(dragMode)) {
             onAddBody(worldPos); // App handles the logic
             return;
        }

        // Start Dragging Body if in select mode
        if (dragMode === 'select') {
            setDraggedBodyId(clickedBodyId);
        }
    } else {
        if (dragMode === 'pan' || dragMode === 'select') {
            setIsDraggingCam(true);
            setDragStart({ x: e.clientX, y: e.clientY });
            onSelectBody(null);
        }
    }
  };

  const handleMouseMove = (e: React.MouseEvent) => {
      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const worldPos = screenToWorld(sx, sy);
      setMousePos(worldPos); 
      
      // Calculate Snapping
      if (rampCreation) {
          const start = rampCreation.start;
          const delta = Vec2.sub(worldPos, start);
          const dist = Vec2.mag(delta);
          
          if (dist > 5) {
              const angleRad = Math.atan2(delta.y, delta.x);
              const angleDeg = angleRad * 180 / Math.PI;
              
              const snapAngles = [0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -45, -60, -90, -120, -135, -150];
              let closest = angleDeg;
              let minDiff = 10; // Snap threshold degrees

              for (const a of snapAngles) {
                  let diff = Math.abs(a - angleDeg);
                  if (diff > 180) diff = 360 - diff;
                  
                  if (diff < minDiff) {
                      minDiff = diff;
                      closest = a;
                  }
              }
              
              const snapRad = closest * Math.PI / 180;
              const snappedEnd = {
                  x: start.x + Math.cos(snapRad) * dist,
                  y: start.y + Math.sin(snapRad) * dist
              };
              setSnappedMousePos(snappedEnd);
          } else {
              setSnappedMousePos(worldPos);
          }
      } else {
          setSnappedMousePos(snapToGrid(worldPos));
      }

      // Handle Vector Editing
      if (vectorEditBodyId) {
          const body = state.bodies.find(b => b.id === vectorEditBodyId);
          if (body) {
              const rawVec = Vec2.sub(worldPos, body.position);
              // Scale down visual drag to physics value
              // 100 pixels on screen = 10 m/s or 100 N?
              // Let's say 1 pixel = 0.1 units for velocity, 1 unit for force
              let physicsVec = rawVec;
              if (dragMode === 'tool_force') {
                  physicsVec = Vec2.mul(rawVec, 10); // Sensitivity
              }
              onVectorEdit(vectorEditBodyId, dragMode === 'tool_velocity' ? 'velocity' : 'force', physicsVec);
          }
          return;
      }

      if (draggedBodyId) {
          onMoveBody(draggedBodyId, snappedMousePos); // Use snapped for dragging
          return;
      }

      if (isDraggingCam) {
           onZoom(0, sx + (dragStart.x - e.clientX), sy + (dragStart.y - e.clientY)); 
      }
  };

  const handleMouseUp = () => {
      setIsDraggingCam(false);
      setDraggedBodyId(null);
      setVectorEditBodyId(null);
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const rect = canvasRef.current!.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    onZoom(e.deltaY, sx, sy);
  };

  return (
    <canvas 
      ref={canvasRef}
      className="w-full h-full cursor-crosshair block"
      onMouseDown={handleMouseDown}
      onWheel={handleWheel}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
    />
  );
});

export default SimulationCanvas;
