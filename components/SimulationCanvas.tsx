

import React, { useRef, useEffect, useState, useImperativeHandle, forwardRef, useMemo } from 'react';
import { BodyType, SimulationState, FieldType, Vector2, FieldShape, ConstraintType, PhysicsField } from '../types';
import { Vec2 } from '../services/vectorMath';

interface Props {
  state: SimulationState;
  onSelectBody: (id: string | null) => void;
  onSelectField: (id: string | null) => void;
  onSelectConstraint: (id: string | null) => void;
  onAddBody: (position: Vector2) => void;
  onAddField: (position: Vector2, shape: FieldShape, vertices?: Vector2[]) => void;
  onMoveBody: (id: string, position: Vector2) => void;
  onVectorEdit: (id: string, vectorType: 'velocity' | 'force', vector: Vector2) => void;
  dragMode: string;
  onZoom: (delta: number, mouseX: number, mouseY: number) => void;
  onPauseToggle: (paused: boolean) => void;
  arcCreation: { phase: number, center: Vector2, radius: number, startAngle: number, endAngle: number } | null;
  rampCreation: { start: Vector2 } | null;
  constraintBuilder: string | null;
}

export interface CanvasRef {
  exportImage: () => void;
}

const SimulationCanvas = forwardRef<CanvasRef, Props>(({ state, onSelectBody, onSelectField, onSelectConstraint, onAddBody, onAddField, onMoveBody, onVectorEdit, dragMode, onZoom, onPauseToggle, arcCreation, rampCreation, constraintBuilder }, ref) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isDraggingCam, setIsDraggingCam] = useState(false);
  const [draggedBodyId, setDraggedBodyId] = useState<string | null>(null);
  const [dragStart, setDragStart] = useState<Vector2>({ x: 0, y: 0 }); // Screen coordinates
  const [hasMoved, setHasMoved] = useState(false);
  
  // Touch
  const lastTouchRef = useRef<{ dist: number, x: number, y: number } | null>(null);

  // Vector Interaction
  const [draggedVector, setDraggedVector] = useState<{ bodyId: string, type: 'velocity' | 'force' } | null>(null);

  // Vector Builder
  const [vectorBuilder, setVectorBuilder] = useState<{bodyId: string, startPos: Vector2} | null>(null);

  // Field Builder
  const [fieldBuilder, setFieldBuilder] = useState<{start: Vector2, shape: FieldShape, vertices?: Vector2[]} | null>(null);
  
  // Cut Tool Line
  const [cutStart, setCutStart] = useState<Vector2 | null>(null);

  const [mousePos, setMousePos] = useState<Vector2>({ x: 0, y: 0 });
  const [snappedMousePos, setSnappedMousePos] = useState<Vector2>({ x: 0, y: 0 });
  const [snapIndicator, setSnapIndicator] = useState<Vector2 | null>(null);

  useImperativeHandle(ref, () => ({
    exportImage: () => {
        if (!canvasRef.current) return;
        const link = document.createElement('a');
        link.download = `physlab-snapshot-${Date.now()}.png`;
        link.href = canvasRef.current.toDataURL();
        link.click();
    }
  }));

  const screenToWorld = (sx: number, sy: number) => {
     return {
         x: (sx - state.camera.x) / state.camera.zoom,
         y: (sy - state.camera.y) / state.camera.zoom
     };
  };

  const evaluateCustomField = (eq: string, x: number, y: number, t: number): number => {
      try {
          const f = new Function('x', 'y', 't', `with(Math){ return ${eq}; }`);
          const res = f(x, y, t);
          return isNaN(res) ? 0 : res;
      } catch (e) {
          return 0;
      }
  };
  
  const getGridStep = (zoom: number) => {
    const targetPx = 80; 
    const rawVal = targetPx / zoom;
    const power = Math.floor(Math.log10(rawVal));
    const base = Math.pow(10, power);
    const mult = rawVal / base;
    return mult < 2 ? 1 * base : mult < 5 ? 2 * base : 5 * base;
  };

  const snapToGrid = (v: Vector2): Vector2 => {
      const gridSize = getGridStep(state.camera.zoom);
      const snappedX = Math.round(v.x / gridSize) * gridSize;
      const snappedY = Math.round(v.y / gridSize) * gridSize;
      const distSq = Math.pow((v.x - snappedX) * state.camera.zoom, 2) + Math.pow((v.y - snappedY) * state.camera.zoom, 2);
      if (distSq < 225) return { x: snappedX, y: snappedY };
      return { x: v.x, y: v.y }; 
  };
  
  // Adsorption / Snapping logic for bodies
  const getSnappingPosition = (currentPos: Vector2, movingBodyId: string): Vector2 => {
      let bestPos = currentPos;
      let minDist = 20 / state.camera.zoom; // Snapping threshold
      let snapped = false;

      // Check against other static bodies (Ramps, Arcs, etc)
      for (const body of state.bodies) {
          if (body.id === movingBodyId) continue;
          if (body.inverseMass !== 0) continue; // Only snap to static stuff

          // Snap to surface of Box (Ramp)
          if (body.type === BodyType.BOX) {
               // Simple snap to center for now, or corners?
               // Let's snap to the surface line
               // Convert currentPos to local
               const local = Vec2.invTransform(currentPos, body.position, body.angle);
               const w = (body.width || 40);
               const h = (body.height || 40);
               // Top surface is at local y = -h/2
               if (Math.abs(local.x) <= w/2 + 5 && Math.abs(local.y + h/2) < 20) {
                   const snapLocal = { x: local.x, y: -h/2 - (5) }; // Sit on top (radius?)
                   const snapWorld = Vec2.transform(snapLocal, body.position, body.angle);
                   if (Vec2.dist(currentPos, snapWorld) < minDist) {
                       bestPos = snapWorld;
                       minDist = Vec2.dist(currentPos, snapWorld);
                       snapped = true;
                   }
               }
          }
          
          // Snap to Arc
          if (body.type === BodyType.ARC) {
              const r = body.radius || 100;
              const d = Vec2.sub(currentPos, body.position);
              const dist = Vec2.mag(d);
              if (Math.abs(dist - r) < 20) {
                  const dir = Vec2.normalize(d);
                  const snapWorld = Vec2.add(body.position, Vec2.mul(dir, r)); // On the line
                   if (Vec2.dist(currentPos, snapWorld) < minDist) {
                       bestPos = snapWorld;
                       minDist = Vec2.dist(currentPos, snapWorld);
                       snapped = true;
                   }
              }
          }
      }
      
      setSnapIndicator(snapped ? bestPos : null);
      return snapped ? bestPos : snapToGrid(currentPos);
  };

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
      ctx.fillStyle = '#0f172a'; // Slate-950
      ctx.fillRect(0, 0, rect.width, rect.height);

      ctx.save();
      ctx.translate(state.camera.x, state.camera.y);
      ctx.scale(state.camera.zoom, state.camera.zoom);

      drawGrid(ctx);

      // Render Field Vectors Grid
      if (state.fields.length > 0) {
          drawFieldGrid(ctx, rect.width/dpr, rect.height/dpr);
      }

      // Fields (Area Visualization)
      state.fields.forEach(field => {
        if (!field.visible) return;
        ctx.save();
        const isSelected = field.id === state.selectedFieldId;
        const alpha = isSelected ? 0.2 : 0.1;
        const color = field.type === FieldType.UNIFORM_ELECTRIC ? '#ef4444' 
                        : field.type === FieldType.UNIFORM_MAGNETIC ? '#3b82f6' 
                        : field.type === FieldType.CUSTOM ? '#d946ef'
                        : '#10b981'; // Gravity
        
        ctx.fillStyle = color + Math.floor(alpha*255).toString(16).padStart(2,'0');
        ctx.strokeStyle = color;
        ctx.lineWidth = isSelected ? 2 / state.camera.zoom : 1 / state.camera.zoom;
        ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);

        if (field.shape === FieldShape.BOX) {
            ctx.fillRect(field.position.x, field.position.y, field.size.x, field.size.y);
            ctx.strokeRect(field.position.x, field.position.y, field.size.x, field.size.y);
        } else if (field.shape === FieldShape.CIRCLE) {
            const r = field.radius || 100;
            ctx.beginPath(); ctx.arc(field.position.x, field.position.y, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        } else if (field.shape === FieldShape.POLYGON && field.vertices) {
            ctx.beginPath();
            if (field.vertices.length > 0) {
                ctx.moveTo(field.vertices[0].x, field.vertices[0].y);
                for(let i=1; i<field.vertices.length; i++) ctx.lineTo(field.vertices[i].x, field.vertices[i].y);
            }
            ctx.closePath(); ctx.fill(); ctx.stroke();
        }
        ctx.restore();
      });

      // Bodies
      state.bodies.forEach(body => {
        ctx.save();
        ctx.translate(body.position.x, body.position.y);
        ctx.save();
        ctx.rotate(body.angle);
        
        ctx.fillStyle = body.color;
        const isSelected = body.id === state.selectedBodyId;

        ctx.lineWidth = isSelected ? 3 / state.camera.zoom : 2 / state.camera.zoom;
        ctx.strokeStyle = isSelected ? '#ffffff' : '#000000';
        
        if (isSelected) {
            ctx.shadowColor = 'rgba(255,255,255,0.8)'; ctx.shadowBlur = 10;
        }

        if (body.type === BodyType.CIRCLE) {
          ctx.beginPath();
          const r = body.isParticle ? 6 / state.camera.zoom : (body.radius || 20);
          ctx.arc(0, 0, r, 0, Math.PI * 2);
          if (body.isHollow) {
              ctx.strokeStyle = body.color; ctx.lineWidth = 4/state.camera.zoom; ctx.stroke();
          } else {
              ctx.fill(); ctx.shadowColor = 'transparent'; ctx.stroke();
          }
          if (!body.isParticle && !body.isHollow) {
              ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(r * 0.8, 0);
              ctx.strokeStyle = 'rgba(0,0,0,0.3)'; ctx.lineWidth = 2 / state.camera.zoom; ctx.stroke();
          }
        } else if (body.type === BodyType.BOX) {
          const w = body.width || 40;
          const h = body.height || 40;
          if (h <= 6 && body.inverseMass === 0) {
              drawHatchedLine(ctx, -w/2, 0, w/2, 0);
          } else {
              if (body.isHollow) {
                  ctx.strokeStyle = body.color; ctx.lineWidth = 4/state.camera.zoom; ctx.strokeRect(-w/2, -h/2, w, h);
              } else {
                  ctx.fillRect(-w/2, -h/2, w, h); ctx.shadowColor = 'transparent'; ctx.strokeRect(-w/2, -h/2, w, h);
              }
              if (body.surfaceSpeed && body.surfaceSpeed !== 0) {
                  const dir = Math.sign(body.surfaceSpeed);
                  ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth = 2 / state.camera.zoom;
                  ctx.setLineDash([5, 5]); ctx.lineDashOffset = -state.time * 20 * dir; 
                  ctx.beginPath(); ctx.moveTo(-w/2, -h/2); ctx.lineTo(w/2, -h/2); ctx.moveTo(-w/2, h/2); ctx.lineTo(w/2, h/2); ctx.stroke();
                  ctx.setLineDash([]);
              }
          }
        } else if (body.type === BodyType.POLYGON) {
           if (body.vertices && body.vertices.length > 0) {
              ctx.beginPath(); ctx.moveTo(body.vertices[0].x, body.vertices[0].y);
              for (let i = 1; i < body.vertices.length; i++) ctx.lineTo(body.vertices[i].x, body.vertices[i].y);
              
              if (body.isHollow && body.vertices.length > 2) {
                   ctx.closePath();
                   ctx.strokeStyle = body.color; ctx.lineWidth = 4/state.camera.zoom; ctx.stroke();
              } else if (body.isHollow) {
                   ctx.strokeStyle = body.color; ctx.lineWidth = 4/state.camera.zoom; ctx.stroke();
              } else {
                   ctx.closePath();
                   ctx.fill(); ctx.shadowColor = 'transparent'; ctx.stroke();
              }
          }
        } else if (body.type === BodyType.ARC) {
             const r = body.radius || 100;
             const start = body.arcStartAngle || 0;
             const end = body.arcEndAngle || Math.PI;
             drawHatchedArc(ctx, 0, 0, r, start, end, isSelected ? '#ffffff' : body.color);
        }
        
        if (body.showCharge && body.charge !== 0) {
             const offset = body.isParticle ? -15 : 0;
             ctx.fillStyle = body.charge > 0 ? 'rgba(239, 68, 68, 0.8)' : 'rgba(59, 130, 246, 0.8)';
             ctx.font = `bold ${20/state.camera.zoom}px monospace`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
             ctx.fillText(body.charge > 0 ? '+' : '-', 0, offset / state.camera.zoom);
        }

        ctx.restore();

        // Vectors
        if (body.showVelocity && Vec2.magSq(body.velocity) > 0.01) drawVector(ctx, body.velocity, '#3b82f6', 'v', 1.0);
        if (body.showAcceleration && Vec2.magSq(body.acceleration) > 0.01) drawVector(ctx, body.acceleration, '#a855f7', 'a', 5.0);
        if (body.showForce) {
            if (Vec2.magSq(body.force) > 0.01) drawVector(ctx, body.force, '#ef4444', 'F_net', 1.0);
            if (body.forceComponents) {
                Object.entries(body.forceComponents).forEach(([name, val]) => {
                     const vec = val as Vector2;
                     if (Vec2.magSq(vec) > 0.1) {
                         let color = '#fff';
                         if (name === 'Gravity' || name === 'Univ. Gravity') color = '#22c55e'; if (name === 'Normal' || name.includes('Normal')) color = '#eab308';
                         if (name === 'Friction' || name === 'Drag') color = '#f97316'; if (name === 'Electric') color = '#f472b6';
                         if (name === 'Magnetic') color = '#3b82f6'; if (name === 'Spring') color = '#06b6d4';
                         if (name === 'Custom') color = '#d946ef'; if (name === 'Coulomb') color = '#f0abfc';
                         drawVector(ctx, vec, color, name, 1.0, true);
                     }
                });
            }
        }

        if (Vec2.magSq(body.constantForce) > 0 || draggedVector?.type === 'force') {
            drawInteractiveVector(ctx, body.constantForce, '#ef4444', 'F_app', 1.0, true);
        }
        if ((body.showVelocity && Vec2.magSq(body.velocity) > 0) || draggedVector?.type === 'velocity') {
            drawInteractiveVector(ctx, body.velocity, '#3b82f6', 'v0', 1.0, true);
        }

        ctx.restore();
      });
      
      // Trails
      state.bodies.forEach(body => {
          if (body.showTrajectory && body.trail && body.trail.length > 1) {
              ctx.save();
              ctx.beginPath();
              ctx.moveTo(body.trail[0].x, body.trail[0].y);
              for(let i=1; i<body.trail.length; i++) ctx.lineTo(body.trail[i].x, body.trail[i].y);
              
              ctx.strokeStyle = body.color; 
              ctx.globalAlpha = 0.5; 
              ctx.lineWidth = 1.5 / state.camera.zoom; 
              ctx.lineJoin = 'round'; // Smooth joins
              ctx.lineCap = 'round';
              ctx.stroke();
              ctx.restore();
          }
      });

      // Constraints
      state.constraints.forEach(c => {
        const bodyA = state.bodies.find(b => b.id === c.bodyAId);
        const bodyB = state.bodies.find(b => b.id === c.bodyBId);
        if (bodyA && bodyB) {
            const pA = Vec2.transform(c.localAnchorA, bodyA.position, bodyA.angle);
            const pB = Vec2.transform(c.localAnchorB, bodyB.position, bodyB.angle);
            ctx.beginPath(); ctx.moveTo(pA.x, pA.y); ctx.lineTo(pB.x, pB.y);
            const isSelected = state.selectedConstraintId === c.id;
            ctx.lineWidth = (c.type === ConstraintType.SPRING ? 3 : 5) / state.camera.zoom;
            ctx.strokeStyle = isSelected ? '#ffffff' : (c.type === ConstraintType.SPRING ? '#fbbf24' : '#94a3b8'); 
            
            if (c.type === ConstraintType.SPRING) drawSpring(ctx, pA, pB, 6 / state.camera.zoom);
            else if (c.type === ConstraintType.ROD) { 
                ctx.stroke(); 
                ctx.fillStyle = '#94a3b8'; ctx.beginPath(); ctx.arc(pA.x, pA.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); 
                ctx.beginPath(); ctx.arc(pB.x, pB.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); 
            }
            else if (c.type === ConstraintType.ROPE) {
                const dist = Vec2.dist(pA, pB);
                const slack = c.length - dist;
                if (slack > 1) {
                     ctx.strokeStyle = '#e2e8f0'; ctx.lineWidth = 2 / state.camera.zoom;
                     ctx.beginPath(); ctx.moveTo(pA.x, pA.y); 
                     const mid = Vec2.div(Vec2.add(pA, pB), 2);
                     mid.y += slack * 0.5; // Sag
                     ctx.quadraticCurveTo(mid.x, mid.y, pB.x, pB.y);
                     ctx.stroke();
                } else {
                     ctx.strokeStyle = '#e2e8f0'; ctx.lineWidth = 2 / state.camera.zoom; ctx.stroke();
                }
                ctx.fillStyle = '#e2e8f0'; ctx.beginPath(); ctx.arc(pA.x, pA.y, 3 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); ctx.beginPath(); ctx.arc(pB.x, pB.y, 3 / state.camera.zoom, 0, Math.PI*2); ctx.fill();
            }
            else if (c.type === ConstraintType.PIN) { ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(pA.x, pA.y, 6 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 2 / state.camera.zoom; ctx.stroke(); }
        }
      });

      // Constraint Builder Preview
      if (constraintBuilder) {
          const bodyA = state.bodies.find(b => b.id === constraintBuilder);
          if (bodyA) {
              ctx.save(); ctx.beginPath(); ctx.moveTo(bodyA.position.x, bodyA.position.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              ctx.strokeStyle = '#fbbf24'; ctx.lineWidth = 2 / state.camera.zoom; ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]); ctx.stroke();
              ctx.fillStyle = '#fbbf24'; ctx.beginPath(); ctx.arc(snappedMousePos.x, snappedMousePos.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); ctx.restore();
          }
      }

      // Interactive Vector Builder Preview (Initial Velocity etc)
      if (vectorBuilder) {
          ctx.save();
          // Draw arrow from body center to mouse
          const body = state.bodies.find(b => b.id === vectorBuilder.bodyId);
          if (body) {
              const start = body.position;
              const end = snappedMousePos;
              const vec = Vec2.sub(end, start);
              drawVector(ctx, vec, '#3b82f6', 'v', 1.0, false, start);
          }
          ctx.restore();
      }
      
      // Cut Tool Preview
      if (dragMode === 'tool_cut' && cutStart) {
          ctx.save();
          ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2 / state.camera.zoom; ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);
          ctx.beginPath(); ctx.moveTo(cutStart.x, cutStart.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y); ctx.stroke();
          ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(cutStart.x, cutStart.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill();
          ctx.restore();
      }
      
      // Ramp/Line Builder Preview
      if (rampCreation) {
          ctx.save();
          ctx.strokeStyle = '#f8fafc'; ctx.lineWidth = 2 / state.camera.zoom;
          ctx.beginPath(); ctx.moveTo(rampCreation.start.x, rampCreation.start.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
          ctx.stroke();
          ctx.fillStyle = '#f8fafc'; 
          ctx.beginPath(); ctx.arc(rampCreation.start.x, rampCreation.start.y, 3/state.camera.zoom, 0, Math.PI*2); ctx.fill();
          ctx.beginPath(); ctx.arc(snappedMousePos.x, snappedMousePos.y, 3/state.camera.zoom, 0, Math.PI*2); ctx.fill();
          ctx.restore();
      }

      // Field Builder Preview
      if (fieldBuilder) {
          ctx.save();
          ctx.strokeStyle = '#10b981'; ctx.lineWidth = 1.5 / state.camera.zoom; ctx.setLineDash([4/state.camera.zoom, 4/state.camera.zoom]);
          
          if (fieldBuilder.shape === FieldShape.BOX) {
             const w = snappedMousePos.x - fieldBuilder.start.x; 
             const h = snappedMousePos.y - fieldBuilder.start.y;
             ctx.strokeRect(fieldBuilder.start.x, fieldBuilder.start.y, w, h);
          } else if (fieldBuilder.shape === FieldShape.CIRCLE) {
             const r = Vec2.dist(fieldBuilder.start, snappedMousePos);
             ctx.beginPath(); ctx.arc(fieldBuilder.start.x, fieldBuilder.start.y, r, 0, Math.PI*2); ctx.stroke();
             ctx.beginPath(); ctx.moveTo(fieldBuilder.start.x, fieldBuilder.start.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y); ctx.stroke();
          } else if (fieldBuilder.shape === FieldShape.POLYGON && fieldBuilder.vertices) {
             ctx.beginPath();
             if (fieldBuilder.vertices.length > 0) {
                 ctx.moveTo(fieldBuilder.vertices[0].x, fieldBuilder.vertices[0].y);
                 for(let i=1; i<fieldBuilder.vertices.length; i++) ctx.lineTo(fieldBuilder.vertices[i].x, fieldBuilder.vertices[i].y);
                 ctx.lineTo(snappedMousePos.x, snappedMousePos.y); // Current drag line
             }
             ctx.stroke();
          }
          ctx.restore();
      }

      // Polygon Creation Hint
      if (dragMode === 'add_poly') {
          ctx.save();
          ctx.strokeStyle = '#14b8a6'; ctx.lineWidth = 1 / state.camera.zoom;
          ctx.beginPath(); ctx.moveTo(snappedMousePos.x - 10/state.camera.zoom, snappedMousePos.y); ctx.lineTo(snappedMousePos.x + 10/state.camera.zoom, snappedMousePos.y);
          ctx.moveTo(snappedMousePos.x, snappedMousePos.y - 10/state.camera.zoom); ctx.lineTo(snappedMousePos.x, snappedMousePos.y + 10/state.camera.zoom);
          ctx.stroke();
          ctx.restore();
      }

      // Arc Logic Visuals
      if (arcCreation) {
          ctx.save();
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2 / state.camera.zoom;
          ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);

          if (arcCreation.phase === 1) {
              // Dragging out radius
              const currentRadius = Vec2.dist(arcCreation.center, snappedMousePos);
              ctx.beginPath();
              ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI * 2);
              ctx.globalAlpha = 0.3; ctx.stroke(); ctx.globalAlpha = 1.0;
              ctx.beginPath(); ctx.moveTo(arcCreation.center.x, arcCreation.center.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.stroke();
          } else if (arcCreation.phase === 2) {
              const currentRadius = Vec2.dist(arcCreation.center, snappedMousePos);
              ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI * 2);
              ctx.globalAlpha = 0.3; ctx.stroke(); ctx.globalAlpha = 1.0;
              ctx.beginPath(); ctx.moveTo(arcCreation.center.x, arcCreation.center.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              ctx.stroke();

          } else if (arcCreation.phase === 3) {
              // Defining End Angle
              const currentRadius = arcCreation.radius;
              const start = arcCreation.startAngle;
              const currentPos = Vec2.sub(snappedMousePos, arcCreation.center);
              let end = Math.atan2(currentPos.y, currentPos.x);
              
              // Draw the arc
              drawHatchedArc(ctx, arcCreation.center.x, arcCreation.center.y, currentRadius, start, end, '#38bdf8');
              
              // Draw line to current mouse
              ctx.beginPath(); ctx.moveTo(arcCreation.center.x, arcCreation.center.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.stroke();
          }
          
          ctx.fillStyle = '#38bdf8';
          ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill();
          ctx.restore();
      }

      // Draw Snap Indicator (Crosshair)
      ctx.save();
      const snapToDraw = snapIndicator || snappedMousePos;
      const crossSize = 5 / state.camera.zoom;
      ctx.strokeStyle = snapIndicator ? '#22d3ee' : 'rgba(255, 255, 255, 0.5)';
      ctx.lineWidth = (snapIndicator ? 2 : 1) / state.camera.zoom;
      ctx.beginPath();
      ctx.moveTo(snapToDraw.x - crossSize, snapToDraw.y);
      ctx.lineTo(snapToDraw.x + crossSize, snapToDraw.y);
      ctx.moveTo(snapToDraw.x, snapToDraw.y - crossSize);
      ctx.lineTo(snapToDraw.x, snapToDraw.y + crossSize);
      ctx.stroke();
      if (snapIndicator) {
          ctx.beginPath(); ctx.arc(snapToDraw.x, snapToDraw.y, crossSize*1.5, 0, Math.PI*2); ctx.stroke();
      }
      ctx.restore();

      ctx.restore();
    };

    render();
  }, [state, mousePos, snappedMousePos, arcCreation, rampCreation, draggedBodyId, dragMode, constraintBuilder, vectorBuilder, fieldBuilder, draggedVector, cutStart, snapIndicator]);

  const drawHatchedArc = (ctx: CanvasRenderingContext2D, cx: number, cy: number, r: number, start: number, end: number, color: string) => {
      const zoom = state.camera.zoom;
      ctx.beginPath();
      ctx.arc(cx, cy, r, start, end);
      ctx.strokeStyle = color;
      ctx.lineWidth = 2 / zoom;
      ctx.stroke();

      // Hatches
      const hatchLen = 10 / zoom;
      const step = 0.2; // Radian step
      // Normalize angle range
      let range = end - start;
      if (range <= 0) range += Math.PI * 2; // Arcs usually go CCW
      // Special case: if range is effectively 0 (start~end), draw nothing or full circle?
      // For physics arc, usually valid range.
      
      const numSteps = Math.ceil(range / step);
      
      ctx.beginPath();
      ctx.strokeStyle = 'rgba(255,255,255,0.4)';
      ctx.lineWidth = 1 / zoom;
      for (let i = 0; i <= numSteps; i++) {
          const a = start + (i / numSteps) * range;
          const x1 = cx + Math.cos(a) * r;
          const y1 = cy + Math.sin(a) * r;
          const x2 = cx + Math.cos(a) * (r + hatchLen);
          const y2 = cy + Math.sin(a) * (r + hatchLen);
          ctx.moveTo(x1, y1);
          ctx.lineTo(x2, y2);
      }
      ctx.stroke();
  };

  const drawHatchedLine = (ctx: CanvasRenderingContext2D, x1: number, y1: number, x2: number, y2: number) => {
      const zoom = state.camera.zoom;
      ctx.beginPath();
      ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
      ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 2 / zoom; ctx.stroke();
      
      const len = Math.sqrt((x2-x1)**2 + (y2-y1)**2);
      const angle = Math.atan2(y2-y1, x2-x1);
      const normal = angle + Math.PI/2;
      const hatchLen = 8 / zoom;
      const spacing = 15 / zoom;
      const count = Math.ceil(len/spacing);

      ctx.beginPath();
      ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.lineWidth = 1 / zoom;
      for(let i=0; i<=count; i++) {
          const t = i/count;
          const px = x1 + (x2-x1)*t;
          const py = y1 + (y2-y1)*t;
          // Angled hatch
          const hx = px + Math.cos(normal - 0.5) * hatchLen;
          const hy = py + Math.sin(normal - 0.5) * hatchLen;
          ctx.moveTo(px, py); ctx.lineTo(hx, hy);
      }
      ctx.stroke();
  };

  const drawFieldGrid = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
      const spacing = 60 / state.camera.zoom; // World space spacing
      // Find start coordinates in world space
      const startX = Math.floor(((0 - state.camera.x) / state.camera.zoom) / spacing) * spacing;
      const startY = Math.floor(((0 - state.camera.y) / state.camera.zoom) / spacing) * spacing;
      const endX = startX + (width / state.camera.zoom);
      const endY = startY + (height / state.camera.zoom);

      for (let x = startX; x < endX; x += spacing) {
          for (let y = startY; y < endY; y += spacing) {
              let E = { x: 0, y: 0 };
              let B = 0;
              let G = { x: 0, y: 0 };

              // Sample Fields at (x, y)
              for (const field of state.fields) {
                  if (!field.visible) continue;
                  let inField = false;
                  if (field.shape === FieldShape.BOX) {
                      inField = x >= field.position.x && x <= field.position.x + field.size.x && y >= field.position.y && y <= field.position.y + field.size.y;
                  } else if (field.shape === FieldShape.CIRCLE) {
                      inField = (x - field.position.x)**2 + (y - field.position.y)**2 <= (field.radius || 100)**2;
                  } else if (field.shape === FieldShape.POLYGON && field.vertices) {
                      // Simple bounds check for perf
                      inField = true; // Optimization: skip full poly check for grid visual
                  }
                  
                  if (inField) {
                      if (field.type === FieldType.UNIFORM_ELECTRIC) { E = Vec2.add(E, field.strength as Vector2); }
                      else if (field.type === FieldType.UNIFORM_MAGNETIC) { B += field.strength as number; }
                      else if (field.type === FieldType.AREA_GRAVITY) { G = Vec2.add(G, field.strength as Vector2); }
                      else if (field.type === FieldType.CUSTOM && field.equations) {
                          const v1 = evaluateCustomField(field.equations.ex, x, y, state.time);
                          const v2 = evaluateCustomField(field.equations.ey, x, y, state.time);
                          if (field.customType === 'MAGNETIC') B += v1;
                          else E = Vec2.add(E, {x: v1, y: v2});
                      }
                  }
              }

              // Draw Indicators
              const scale = 0.5;
              if (Vec2.magSq(E) > 0.1) {
                  drawVector(ctx, E, 'rgba(239, 68, 68, 0.4)', '', scale, false, {x, y});
              }
              if (Vec2.magSq(G) > 0.1) {
                  drawVector(ctx, G, 'rgba(16, 185, 129, 0.4)', '', scale, false, {x, y});
              }
              if (Math.abs(B) > 0.1) {
                   ctx.fillStyle = 'rgba(59, 130, 246, 0.3)';
                   ctx.beginPath(); ctx.arc(x, y, 3/state.camera.zoom, 0, Math.PI*2); ctx.fill();
                   ctx.strokeStyle = 'rgba(59, 130, 246, 0.5)'; ctx.lineWidth = 1/state.camera.zoom;
                   if (B > 0) { // Out
                       ctx.beginPath(); ctx.arc(x, y, 3/state.camera.zoom, 0, Math.PI*2); ctx.stroke();
                       ctx.fillStyle = '#3b82f6'; ctx.beginPath(); ctx.arc(x, y, 1/state.camera.zoom, 0, Math.PI*2); ctx.fill();
                   } else { // In
                       ctx.beginPath(); ctx.moveTo(x-2/state.camera.zoom, y-2/state.camera.zoom); ctx.lineTo(x+2/state.camera.zoom, y+2/state.camera.zoom); 
                       ctx.moveTo(x+2/state.camera.zoom, y-2/state.camera.zoom); ctx.lineTo(x-2/state.camera.zoom, y+2/state.camera.zoom); ctx.stroke();
                   }
              }
          }
      }
  };

  const drawVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number, dashed: boolean = false, offset: Vector2 = {x:0, y:0}) => {
      const visualVec = Vec2.mul(vec, scale); 
      const len = Vec2.mag(visualVec);
      if (len * state.camera.zoom < 3) return; 

      ctx.save();
      ctx.translate(offset.x, offset.y);
      ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(visualVec.x, visualVec.y);
      ctx.strokeStyle = color; ctx.lineWidth = (dashed ? 1.5 : 2) / state.camera.zoom;
      if (dashed) ctx.setLineDash([4 / state.camera.zoom, 2 / state.camera.zoom]);
      ctx.stroke(); ctx.setLineDash([]);

      const angle = Math.atan2(visualVec.y, visualVec.x);
      const headLen = 6 / state.camera.zoom;
      ctx.beginPath(); ctx.moveTo(visualVec.x, visualVec.y);
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle - Math.PI / 6), visualVec.y - headLen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle + Math.PI / 6), visualVec.y - headLen * Math.sin(angle + Math.PI / 6));
      ctx.closePath(); ctx.fillStyle = color; ctx.fill();

      if (label) {
          ctx.fillStyle = color; ctx.font = `${10/state.camera.zoom}px monospace`;
          ctx.fillText(label, visualVec.x + 5/state.camera.zoom, visualVec.y + 5/state.camera.zoom);
      }
      ctx.restore();
  };
  
  const drawInteractiveVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number = 1.0, isInteractive: boolean) => {
      const visualVec = Vec2.mul(vec, scale);
      const px = visualVec.x; const py = visualVec.y;
      ctx.save();
      if (Math.abs(px) > 0.1 || Math.abs(py) > 0.1) {
          ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(px, py); ctx.strokeStyle = color; ctx.lineWidth = 2 / state.camera.zoom; ctx.stroke();
          const angle = Math.atan2(py, px); const headLen = 10 / state.camera.zoom;
          ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px - headLen * Math.cos(angle - Math.PI / 6), py - headLen * Math.sin(angle - Math.PI / 6));
          ctx.lineTo(px - headLen * Math.cos(angle + Math.PI / 6), py - headLen * Math.sin(angle + Math.PI / 6));
          ctx.closePath(); ctx.fillStyle = color; ctx.fill();
      }
      if (isInteractive) {
          ctx.beginPath(); const handleR = 3 / state.camera.zoom; 
          ctx.arc(px, py, handleR, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1 / state.camera.zoom; ctx.stroke();
      }
      if (Math.abs(px) > 0.1 || Math.abs(py) > 0.1) {
          ctx.fillStyle = color; ctx.font = `${10/state.camera.zoom}px monospace`;
          const mag = Vec2.mag(vec); ctx.fillText(`${label}: ${mag.toFixed(1)}`, px + 10/state.camera.zoom, py + 5/state.camera.zoom);
      }
      ctx.restore();
  };

  const drawGrid = (ctx: CanvasRenderingContext2D) => {
      const zoom = state.camera.zoom;
      const gridSize = getGridStep(zoom);
      const farLeft = ((-state.camera.x / zoom)); const farTop = ((-state.camera.y / zoom));
      const w = (ctx.canvas.width / window.devicePixelRatio) / zoom; const h = (ctx.canvas.height / window.devicePixelRatio) / zoom;
      ctx.strokeStyle = 'rgba(255,255,255,0.12)'; ctx.lineWidth = 1 / zoom;
      const startX = Math.floor(farLeft/gridSize)*gridSize; const endX = farLeft+w;
      const startY = Math.floor(farTop/gridSize)*gridSize; const endY = farTop+h;
      
      ctx.beginPath();
      for(let x = startX; x < endX + gridSize; x+=gridSize) { if (Math.abs(x) < 0.001) continue; ctx.moveTo(x, farTop); ctx.lineTo(x, farTop+h); }
      for(let y = startY; y < endY + gridSize; y+=gridSize) { if (Math.abs(y) < 0.001) continue; ctx.moveTo(farLeft, y); ctx.lineTo(farLeft+w, y); }
      ctx.stroke();
      
      // Axes
      ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 2 / zoom; 
      ctx.beginPath(); ctx.moveTo(farLeft, 0); ctx.lineTo(farLeft+w, 0); ctx.moveTo(0, farTop); ctx.lineTo(0, farTop+h); ctx.stroke(); 
      
      ctx.fillStyle = 'rgba(255,255,255,0.6)'; const fontSize = 10 / zoom; ctx.font = `${fontSize}px monospace`;
      
      // Fixed Coordinates Logic
      for(let x = startX; x < endX + gridSize; x+=gridSize) { 
          if (Math.abs(x) < 0.001) continue;
          // Ensure X label is visible even if Y=0 is off screen
          let labelY = 0 + 12/zoom;
          if (labelY < farTop + 12/zoom) labelY = farTop + 12/zoom;
          if (labelY > farTop + h - 5/zoom) labelY = farTop + h - 5/zoom;
          
          ctx.fillText((Math.round(x*100)/100).toString(), x + 2/zoom, labelY); 
      }
      for(let y = startY; y < endY + gridSize; y+=gridSize) { 
          if (Math.abs(y) < 0.001) continue;
          // Ensure Y label is visible even if X=0 is off screen
          let labelX = 0 + 5/zoom;
          if (labelX < farLeft + 5/zoom) labelX = farLeft + 5/zoom;
          if (labelX > farLeft + w - 20/zoom) labelX = farLeft + w - 20/zoom;

          ctx.fillText((Math.round(-y*100)/100).toString(), labelX, y - 2/zoom); 
      }
  };

  const drawSpring = (ctx: CanvasRenderingContext2D, p1: Vector2, p2: Vector2, width: number) => {
     const dist = Vec2.dist(p1, p2);
     const dir = Vec2.normalize(Vec2.sub(p2, p1));
     const perp = Vec2.perp(dir);
     const numCoils = 12;
     ctx.beginPath(); ctx.moveTo(p1.x, p1.y);
     for(let i=0; i<=numCoils; i++) {
        const t = i / numCoils;
        const p = Vec2.add(Vec2.mul(p1, 1-t), Vec2.mul(p2, t));
        let offset = 0; if (i > 0 && i < numCoils) offset = i % 2 === 0 ? width : -width;
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

    setDragStart({ x: e.clientX, y: e.clientY });
    setHasMoved(false);

    // 1. Vector Handles
    const HIT_RADIUS_SCREEN = 15;
    const hitRadiusWorld = HIT_RADIUS_SCREEN / state.camera.zoom;
    if (!dragMode.startsWith('add_') && !dragMode.startsWith('tool_')) {
        for (const body of state.bodies) {
             const checkHandle = (vec: Vector2, type: 'velocity' | 'force') => {
                 const tip = Vec2.add(body.position, vec); 
                 if (Vec2.dist(worldPos, tip) < hitRadiusWorld) { 
                     setDraggedVector({ bodyId: body.id, type });
                     onSelectBody(body.id); onSelectConstraint(null); onSelectField(null); onPauseToggle(true); 
                     return true;
                 }
                 return false;
             };
             if (Vec2.magSq(body.constantForce) > 0 && checkHandle(body.constantForce, 'force')) return;
             if (body.showVelocity && Vec2.magSq(body.velocity) > 0 && checkHandle(body.velocity, 'velocity')) return;
        }
    }

    // 2. Cut Tool Start
    if (dragMode === 'tool_cut') {
        setCutStart(snapped);
        return;
    }

    // 3. Modes
    if (dragMode.startsWith('add_field_')) {
        const shape = dragMode === 'add_field_circle' ? FieldShape.CIRCLE : dragMode === 'add_field_poly' ? FieldShape.POLYGON : FieldShape.BOX;
        if (dragMode === 'add_field_poly') {
            if (!fieldBuilder) { setFieldBuilder({ start: snapped, shape, vertices: [snapped] }); } else {
                if (fieldBuilder.vertices && fieldBuilder.vertices.length > 2) {
                    const start = fieldBuilder.vertices[0];
                    if (Vec2.dist(snapped, start) < 20 / state.camera.zoom) {
                        onAddField(fieldBuilder.start, FieldShape.POLYGON, fieldBuilder.vertices); setFieldBuilder(null); return;
                    }
                }
                setFieldBuilder({ ...fieldBuilder, vertices: [...(fieldBuilder.vertices || []), snapped] });
            }
        } else { if (!fieldBuilder) setFieldBuilder({ start: snapped, shape }); }
        return;
    }

    if (dragMode === 'tool_velocity' || dragMode === 'tool_force') {
        let clickedBodyId: string | null = null;
        for(let i=state.bodies.length-1; i>=0; i--) {
            const b = state.bodies[i]; const r = b.isParticle ? 10/state.camera.zoom : (b.radius || 20);
            if (Vec2.dist(worldPos, b.position) < r + 10) { clickedBodyId = b.id; break; }
        }
        if (clickedBodyId) { onPauseToggle(true); onSelectBody(clickedBodyId); setVectorBuilder({ bodyId: clickedBodyId, startPos: worldPos }); }
        return;
    }

    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin', 'add_rope', 'add_arc'].includes(dragMode)) {
        onAddBody(snapped); return;
    }
    if (dragMode === 'add_arc') { onAddBody(snapped); return; }

    // 4. Selection
    let clickedBodyId: string | null = null;
    for(let i=state.bodies.length-1; i>=0; i--) {
        const b = state.bodies[i]; let hit = false; const tol = hitRadiusWorld * 1.5; 
        if (b.type === BodyType.CIRCLE) { const r = b.isParticle ? 10/state.camera.zoom : (b.radius || 20); hit = Vec2.dist(worldPos, b.position) < Math.max(r, tol); } 
        else if (b.type === BodyType.ARC) { const r = b.radius || 100; const dist = Vec2.dist(worldPos, b.position); hit = Math.abs(dist - r) < tol * 2; } 
        else if (b.type === BodyType.BOX) { hit = Vec2.pointInRotatedBox(worldPos, b.position, b.width||40, b.height||40, b.angle); } 
        else if (b.type === BodyType.POLYGON && b.vertices) { hit = Vec2.dist(worldPos, b.position) < 40; }
        if (hit) { clickedBodyId = b.id; break; }
    }

    if (clickedBodyId) {
        if (['add_spring', 'add_rod', 'add_pin', 'add_rope'].includes(dragMode)) { 
            onSelectBody(clickedBodyId); 
            return; 
        }
        if (dragMode === 'tool_combine' || dragMode === 'tool_traj') {
            onSelectBody(clickedBodyId);
            return;
        }
        onSelectBody(clickedBodyId); setDraggedBodyId(clickedBodyId); return;
    }

    let clickedConstraintId: string | null = null;
    for (const c of state.constraints) {
        const bA = state.bodies.find(b => b.id === c.bodyAId); const bB = state.bodies.find(b => b.id === c.bodyBId);
        if (bA && bB) {
            const pA = Vec2.transform(c.localAnchorA, bA.position, bA.angle); const pB = Vec2.transform(c.localAnchorB, bB.position, bB.angle);
            if (Vec2.distToSegment(worldPos, pA, pB) < hitRadiusWorld * 2) { clickedConstraintId = c.id; break; }
        }
    }
    if (clickedConstraintId) { onSelectConstraint(clickedConstraintId); return; }

    let clickedFieldId: string | null = null;
    for(let i=state.fields.length-1; i>=0; i--) {
       const f = state.fields[i];
       if (f.shape === FieldShape.BOX) { if (worldPos.x >= f.position.x && worldPos.x <= f.position.x + f.size.x && worldPos.y >= f.position.y && worldPos.y <= f.position.y + f.size.y) { clickedFieldId = f.id; break; } } 
       else if (f.shape === FieldShape.CIRCLE) { if (Vec2.dist(worldPos, f.position) < (f.radius || 100)) { clickedFieldId = f.id; break; } }
       else if (f.shape === FieldShape.POLYGON && f.vertices) { if (Vec2.dist(worldPos, f.vertices[0]) < 100) { clickedFieldId = f.id; break; } }
    }
    if (clickedFieldId) { onSelectField(clickedFieldId); return; }

    if (dragMode === 'select' && !clickedBodyId && !clickedFieldId && !clickedConstraintId) { onSelectBody(null); }
    if (!dragMode.startsWith('add_')) { setIsDraggingCam(true); }
  };

  const handleMouseMove = (e: React.MouseEvent) => {
      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const worldPos = screenToWorld(sx, sy);
      setMousePos(worldPos); 
      
      const moveDist = Math.sqrt(Math.pow(e.clientX - dragStart.x, 2) + Math.pow(e.clientY - dragStart.y, 2));
      if (moveDist > 3) setHasMoved(true);

      if (rampCreation) {
          const start = rampCreation.start; const delta = Vec2.sub(worldPos, start); const dist = Vec2.mag(delta);
          if (dist > 5) {
              const angleRad = Math.atan2(delta.y, delta.x); const angleDeg = angleRad * 180 / Math.PI;
              const snapAngles = [0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -45, -60, -90, -120, -135, -150];
              let closest = angleDeg; let minDiff = 5; 
              for (const a of snapAngles) { let diff = Math.abs(a - angleDeg); if (diff > 180) diff = 360 - diff; if (diff < minDiff) { minDiff = diff; closest = a; } }
              const snapRad = closest * Math.PI / 180; const gridSize = getGridStep(state.camera.zoom); const snapDist = Math.round(dist/gridSize)*gridSize;
              const snappedEnd = { x: start.x + Math.cos(snapRad) * snapDist, y: start.y + Math.sin(snapRad) * snapDist }; setSnappedMousePos(snappedEnd);
          } else { setSnappedMousePos(snapToGrid(worldPos)); }
      } else {
          setSnappedMousePos(snapToGrid(worldPos));
      }
      
      if (draggedVector) {
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              const rawVec = Vec2.sub(worldPos, body.position); let snappedVec = rawVec;
              const angleRad = Math.atan2(rawVec.y, rawVec.x); const angleDeg = angleRad * 180 / Math.PI; const normDeg = (angleDeg % 360);
              if (Math.abs(normDeg) < 5 || Math.abs(Math.abs(normDeg) - 180) < 5) snappedVec = { x: rawVec.x, y: 0 };
              else if (Math.abs(Math.abs(normDeg) - 90) < 5) snappedVec = { x: 0, y: rawVec.y };
              onVectorEdit(draggedVector.bodyId, draggedVector.type, snappedVec);
          }
          return;
      }
      if (vectorBuilder) return;

      if (draggedBodyId && hasMoved) { 
          if (dragMode !== 'select_nodrag') {
              // Apply Snapping Logic when moving bodies
              const snapPos = getSnappingPosition(snappedMousePos, draggedBodyId);
              onMoveBody(draggedBodyId, snapPos); 
          }
          return; 
      }
      setSnapIndicator(null); // Reset if not dragging body

      if (isDraggingCam) { onZoom(0, sx + (dragStart.x - e.clientX), sy + (dragStart.y - e.clientY)); }
  };

  const handleMouseUp = (e: React.MouseEvent) => {
      if (isDraggingCam) setIsDraggingCam(false);
      if (draggedBodyId) { setDraggedBodyId(null); setSnapIndicator(null); }
      if (draggedVector) setDraggedVector(null);
      if (vectorBuilder) {
          const body = state.bodies.find(b => b.id === vectorBuilder.bodyId);
          if (body) { const rawVec = Vec2.sub(snappedMousePos, body.position); onVectorEdit(vectorBuilder.bodyId, dragMode === 'tool_velocity' ? 'velocity' : 'force', rawVec); }
          setVectorBuilder(null);
      }
      
      if (dragMode === 'tool_cut' && cutStart) {
          if (onAddBody && cutStart) {
              const event = new CustomEvent('canvas-cut', { detail: { p1: cutStart, p2: snappedMousePos } });
              window.dispatchEvent(event);
          }
          setCutStart(null);
      }

      if (fieldBuilder && dragMode !== 'add_field_poly') {
           if (dragMode === 'add_field_box') {
               const w = snappedMousePos.x - fieldBuilder.start.x; const h = snappedMousePos.y - fieldBuilder.start.y;
               if (Math.abs(w) > 5 && Math.abs(h) > 5) { const x = w > 0 ? fieldBuilder.start.x : snappedMousePos.x; const y = h > 0 ? fieldBuilder.start.y : snappedMousePos.y; onAddField({x, y}, FieldShape.BOX, [{x: Math.abs(w), y: Math.abs(h)}]); }
           } else if (dragMode === 'add_field_circle') {
               const r = Vec2.dist(fieldBuilder.start, snappedMousePos); if (r > 5) onAddField(fieldBuilder.start, FieldShape.CIRCLE, [{x: r, y: 0}]);
           }
           setFieldBuilder(null);
      } 
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault(); const rect = canvasRef.current!.getBoundingClientRect(); const sx = e.clientX - rect.left; const sy = e.clientY - rect.top; onZoom(e.deltaY, sx, sy);
  };

  // Touch Support
  const handleTouchStart = (e: React.TouchEvent) => {
      if (e.touches.length === 2) {
          // Pinch Zoom Start
          const dx = e.touches[0].clientX - e.touches[1].clientX;
          const dy = e.touches[0].clientY - e.touches[1].clientY;
          lastTouchRef.current = { dist: Math.sqrt(dx*dx + dy*dy), x: 0, y: 0 };
          setIsDraggingCam(false); // Stop panning
      } else if (e.touches.length === 1) {
          const touch = e.touches[0];
          // Simulate mouse down
          handleMouseDown({ 
              clientX: touch.clientX, 
              clientY: touch.clientY, 
              button: 0,
              preventDefault: () => {} 
          } as any);
      }
  };

  const handleTouchMove = (e: React.TouchEvent) => {
      e.preventDefault(); // Prevent scroll
      if (e.touches.length === 2 && lastTouchRef.current) {
          const dx = e.touches[0].clientX - e.touches[1].clientX;
          const dy = e.touches[0].clientY - e.touches[1].clientY;
          const dist = Math.sqrt(dx*dx + dy*dy);
          const delta = lastTouchRef.current.dist - dist;
          
          // Center of pinch
          const cx = (e.touches[0].clientX + e.touches[1].clientX) / 2;
          const cy = (e.touches[0].clientY + e.touches[1].clientY) / 2;
          const rect = canvasRef.current!.getBoundingClientRect();
          
          onZoom(delta * 5, cx - rect.left, cy - rect.top);
          lastTouchRef.current = { dist, x: 0, y: 0 };
      } else if (e.touches.length === 1) {
          const touch = e.touches[0];
          handleMouseMove({ 
              clientX: touch.clientX, 
              clientY: touch.clientY, 
              preventDefault: () => {}
          } as any);
      }
  };

  const handleTouchEnd = (e: React.TouchEvent) => {
      lastTouchRef.current = null;
      handleMouseUp({ clientX: 0, clientY: 0 } as any);
  };

  return (
    <canvas 
      ref={canvasRef}
      className={`w-full h-full block ${dragMode === 'select_nodrag' ? 'cursor-default' : 'cursor-crosshair'}`}
      onMouseDown={handleMouseDown}
      onWheel={handleWheel}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
      onTouchStart={handleTouchStart}
      onTouchMove={handleTouchMove}
      onTouchEnd={handleTouchEnd}
    />
  );
});

export default React.memo(SimulationCanvas);