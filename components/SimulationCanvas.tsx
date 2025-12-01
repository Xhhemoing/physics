import React, { useRef, useEffect, useState, useImperativeHandle, forwardRef } from 'react';
import { BodyType, PhysicsBody, SimulationState, FieldType, Vector2, ConstraintType, FieldShape } from '../types';
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
  arcCreation: { phase: number, center: Vector2, radius: number, endAngle: number } | null;
  rampCreation: { start: Vector2 } | null;
  constraintBuilder: string | null; // ID of the first body
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
  
  // Vector Interaction
  const [draggedVector, setDraggedVector] = useState<{ bodyId: string, type: 'velocity' | 'force' } | null>(null);

  // Vector Builder (Click Body -> Drag -> Release)
  const [vectorBuilder, setVectorBuilder] = useState<{bodyId: string, startPos: Vector2} | null>(null);

  // Field Builder
  const [fieldBuilder, setFieldBuilder] = useState<{start: Vector2, shape: FieldShape, vertices?: Vector2[]} | null>(null);
  
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

  // Helper to convert world to screen
  const worldToScreen = (wx: number, wy: number) => {
      return {
          x: wx * state.camera.zoom + state.camera.x,
          y: wy * state.camera.zoom + state.camera.y
      };
  };

  // Determine the dynamic grid step based on zoom level
  const getGridStep = (zoom: number): number => {
      // Base step logic
      const minScreenSpacing = 50; 
      // We want: step * zoom >= minScreenSpacing
      // step >= minScreenSpacing / zoom
      const rawStep = minScreenSpacing / zoom;
      
      const power = Math.floor(Math.log10(rawStep));
      const base = Math.pow(10, power);
      const multiplier = rawStep / base;

      let step;
      if (multiplier <= 1) step = 1 * base;
      else if (multiplier <= 2) step = 2 * base;
      else if (multiplier <= 5) step = 5 * base;
      else step = 10 * base;

      return step;
  };

  const snapToGrid = (v: Vector2): Vector2 => {
      const step = getGridStep(state.camera.zoom);
      // Strict snapping: Always snap to the nearest grid line
      const snappedX = Math.round(v.x / step) * step;
      const snappedY = Math.round(v.y / step) * step;
      return { x: snappedX, y: snappedY };
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
      
      // Grid & Axis (Draw BEFORE camera transform to handle text easily, or handle internally)
      // We'll apply camera inside drawGrid for lines, but handle text in screen space
      drawGrid(ctx, rect.width, rect.height);

      // Apply Camera for World Objects
      ctx.translate(state.camera.x, state.camera.y);
      ctx.scale(state.camera.zoom, state.camera.zoom);

      // Fields (Background)
      state.fields.forEach(field => {
        if (!field.visible) return;
        
        ctx.save();
        const isSelected = field.id === state.selectedFieldId;
        const alpha = isSelected ? 0.4 : 0.15;
        ctx.fillStyle = field.type === FieldType.UNIFORM_ELECTRIC ? `rgba(239, 68, 68, ${alpha})` 
                      : field.type === FieldType.UNIFORM_MAGNETIC ? `rgba(59, 130, 246, ${alpha})`
                      : field.type === FieldType.CUSTOM ? `rgba(217, 70, 239, ${alpha})`
                      : `rgba(16, 185, 129, ${alpha})`;
        ctx.strokeStyle = isSelected ? '#fff' : 
                        (field.type === FieldType.UNIFORM_ELECTRIC ? '#ef4444' 
                        : field.type === FieldType.UNIFORM_MAGNETIC ? '#3b82f6' 
                        : field.type === FieldType.CUSTOM ? '#d946ef'
                        : '#10b981');
        ctx.lineWidth = isSelected ? 2 / state.camera.zoom : 1 / state.camera.zoom;
        ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);

        if (field.shape === FieldShape.BOX) {
            ctx.fillRect(field.position.x, field.position.y, field.size.x, field.size.y);
            ctx.strokeRect(field.position.x, field.position.y, field.size.x, field.size.y);
            
            // Label
            ctx.fillStyle = ctx.strokeStyle;
            ctx.font = `${12/state.camera.zoom}px sans-serif`;
            ctx.fillText(getFieldLabel(field.type), field.position.x + 5/state.camera.zoom, field.position.y + 15/state.camera.zoom);

        } else if (field.shape === FieldShape.CIRCLE) {
            ctx.beginPath();
            ctx.arc(field.position.x, field.position.y, field.radius || 100, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();
            
            ctx.fillStyle = ctx.strokeStyle;
            ctx.font = `${12/state.camera.zoom}px sans-serif`;
            ctx.fillText(getFieldLabel(field.type), field.position.x - 20/state.camera.zoom, field.position.y);

        } else if (field.shape === FieldShape.POLYGON && field.vertices) {
            ctx.beginPath();
            if (field.vertices.length > 0) {
              ctx.moveTo(field.vertices[0].x, field.vertices[0].y);
              for(let i=1; i<field.vertices.length; i++) ctx.lineTo(field.vertices[i].x, field.vertices[i].y);
            }
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            
            ctx.fillStyle = ctx.strokeStyle;
            ctx.font = `${12/state.camera.zoom}px sans-serif`;
            if (field.vertices.length > 0) ctx.fillText(getFieldLabel(field.type), field.vertices[0].x, field.vertices[0].y);
        }
        
        ctx.restore();
      });

      // Field Builder Preview
      if (fieldBuilder) {
          ctx.save();
          ctx.strokeStyle = '#fff';
          ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);
          ctx.lineWidth = 1 / state.camera.zoom;
          
          if (fieldBuilder.shape === FieldShape.BOX) {
              const w = snappedMousePos.x - fieldBuilder.start.x;
              const h = snappedMousePos.y - fieldBuilder.start.y;
              ctx.strokeRect(fieldBuilder.start.x, fieldBuilder.start.y, w, h);
          } else if (fieldBuilder.shape === FieldShape.CIRCLE) {
              const r = Vec2.dist(fieldBuilder.start, snappedMousePos);
              ctx.beginPath();
              ctx.arc(fieldBuilder.start.x, fieldBuilder.start.y, r, 0, Math.PI * 2);
              ctx.stroke();
          } else if (fieldBuilder.shape === FieldShape.POLYGON && fieldBuilder.vertices) {
              ctx.beginPath();
              if (fieldBuilder.vertices.length > 0) {
                  ctx.moveTo(fieldBuilder.vertices[0].x, fieldBuilder.vertices[0].y);
                  for (let i=1; i<fieldBuilder.vertices.length; i++) ctx.lineTo(fieldBuilder.vertices[i].x, fieldBuilder.vertices[i].y);
                  ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              } else {
                   ctx.moveTo(snappedMousePos.x, snappedMousePos.y);
              }
              ctx.stroke();
              // Vertices
              ctx.fillStyle = '#fff';
              fieldBuilder.vertices.forEach(v => {
                  ctx.beginPath(); ctx.arc(v.x, v.y, 3/state.camera.zoom, 0, Math.PI*2); ctx.fill();
              });
          }
          ctx.restore();
      }

      // Bodies
      state.bodies.forEach(body => {
        ctx.save();
        ctx.translate(body.position.x, body.position.y);
        ctx.save();
        ctx.rotate(body.angle);
        
        // Shadow/Glow logic
        ctx.fillStyle = body.color;
        const baseLineWidth = 2 / state.camera.zoom;
        ctx.lineWidth = body.id === state.selectedBodyId ? baseLineWidth * 1.5 : baseLineWidth;
        ctx.strokeStyle = body.id === state.selectedBodyId ? '#ffffff' : '#000000';
        ctx.shadowColor = 'rgba(0,0,0,0.5)';
        ctx.shadowBlur = 10;
        ctx.shadowOffsetX = 2;
        ctx.shadowOffsetY = 2;

        if (body.type === BodyType.CIRCLE) {
          ctx.beginPath();
          const r = body.isParticle ? 4 : (body.radius || 20);
          ctx.arc(0, 0, r, 0, Math.PI * 2);
          ctx.fill();
          ctx.shadowColor = 'transparent'; 
          ctx.stroke();
          if (!body.isParticle) {
              ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(r * 0.8, 0);
              ctx.strokeStyle = 'rgba(0,0,0,0.3)'; ctx.lineWidth = baseLineWidth; ctx.stroke();
          }
        } else if (body.type === BodyType.BOX) {
          const w = body.width || 40;
          const h = body.height || 40;
          const isRamp = h < 12 && w > h * 2; 
          if (isRamp && body.mass === 0) {
              ctx.shadowColor = 'transparent'; ctx.beginPath(); ctx.rect(-w/2, -h/2, w, h);
              ctx.fillStyle = body.color; ctx.fill();
              ctx.beginPath(); ctx.moveTo(-w/2, -h/2); ctx.lineTo(w/2, -h/2);
              ctx.strokeStyle = '#fff'; ctx.lineWidth = baseLineWidth; ctx.stroke();
          } else {
              ctx.fillRect(-w/2, -h/2, w, h); ctx.shadowColor = 'transparent'; ctx.strokeRect(-w/2, -h/2, w, h);
          }
          // Conveyor marks
           if (body.surfaceSpeed && body.surfaceSpeed !== 0) {
             const dir = Math.sign(body.surfaceSpeed);
             ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth = baseLineWidth;
             ctx.setLineDash([5, 5]); ctx.lineDashOffset = -state.time * 20 * dir; 
             ctx.beginPath(); ctx.moveTo(-w/2, -h/2); ctx.lineTo(w/2, -h/2); ctx.moveTo(-w/2, h/2); ctx.lineTo(w/2, h/2); ctx.stroke();
             ctx.setLineDash([]);
             ctx.fillStyle = 'rgba(255,255,255,0.4)';
             for(let x = -w/2 + 10; x < w/2; x+=20) {
                 ctx.beginPath(); ctx.moveTo(x, -h/2 - 5); ctx.lineTo(x + 5 * dir, -h/2 - 5); ctx.lineTo(x, -h/2 - 8); ctx.fill();
             }
          }
        } else if (body.type === BodyType.POLYGON) {
           if (body.vertices && body.vertices.length > 0) {
              ctx.beginPath(); ctx.moveTo(body.vertices[0].x, body.vertices[0].y);
              for (let i = 1; i < body.vertices.length; i++) ctx.lineTo(body.vertices[i].x, body.vertices[i].y);
              ctx.closePath(); ctx.fill(); ctx.shadowColor = 'transparent'; ctx.stroke();
          }
        } else if (body.type === BodyType.PLANE) {
          ctx.shadowColor = 'transparent'; ctx.beginPath(); ctx.moveTo(-10000, 0); ctx.lineTo(10000, 0);
          ctx.strokeStyle = body.color; ctx.lineWidth = baseLineWidth; ctx.stroke();
          for(let i=-2000; i<2000; i+=25) {
              ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i - 10, 15);
              ctx.strokeStyle = 'rgba(255,255,255,0.1)'; ctx.lineWidth = baseLineWidth/2; ctx.stroke();
          }
        } else if (body.type === BodyType.ARC) {
             const r = body.radius || 100;
             const start = body.arcStartAngle || 0;
             const end = body.arcEndAngle || Math.PI;
             ctx.shadowColor = 'transparent'; ctx.beginPath(); ctx.arc(0, 0, r, start, end);
             ctx.strokeStyle = body.color; ctx.lineWidth = 5 / state.camera.zoom; ctx.stroke();
             ctx.strokeStyle = 'rgba(255,255,255,0.2)'; ctx.lineWidth = baseLineWidth;
             const steps = 20;
             for (let i=0; i<=steps; i++) {
                 const a = start + (end-start)*(i/steps);
                 ctx.beginPath(); ctx.moveTo(Math.cos(a)*r, Math.sin(a)*r); ctx.lineTo(Math.cos(a)*(r+10), Math.sin(a)*(r+10)); ctx.stroke();
             }
        }
        
        if (body.showCharge && body.charge !== 0) {
             const offset = body.isParticle ? -15 : 0;
             ctx.fillStyle = body.charge > 0 ? 'rgba(239, 68, 68, 0.8)' : 'rgba(59, 130, 246, 0.8)';
             // Scale font with zoom roughly but clamp
             ctx.font = `bold ${20/state.camera.zoom}px monospace`; 
             ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
             ctx.fillText(body.charge > 0 ? '+' : '-', 0, offset/state.camera.zoom);
        }

        ctx.restore(); // End Body Rotation

        // Draw Analysis Vectors (World Space)
        if (body.showVelocity && Vec2.magSq(body.velocity) > 0.01) drawVector(ctx, body.velocity, '#3b82f6', 'v', 1.0);
        if (body.showAcceleration && Vec2.magSq(body.acceleration) > 0.01) drawVector(ctx, body.acceleration, '#a855f7', 'a', 5.0);
        if (body.showForce) {
            if (Vec2.magSq(body.force) > 0.01) drawVector(ctx, body.force, '#ef4444', 'F_net', 1.0);
            if (body.forceComponents) {
                Object.entries(body.forceComponents).forEach(([name, val]) => {
                     const vec = val as Vector2;
                     if (Vec2.magSq(vec) > 0.1) {
                         let color = '#fff';
                         if (name === 'Gravity') color = '#22c55e'; if (name === 'Normal') color = '#eab308';
                         if (name === 'Friction') color = '#f97316'; if (name === 'Electric') color = '#f472b6';
                         if (name === 'Magnetic') color = '#3b82f6'; if (name === 'Spring') color = '#06b6d4';
                         if (name === 'Custom') color = '#d946ef';
                         drawVector(ctx, vec, color, name, 1.0, true);
                     }
                });
            }
        }

        // Draw Interactive Vector Handles (Velocity & Constant Force)
        const isToolMode = dragMode === 'tool_velocity' || dragMode === 'tool_force';
        
        if (Vec2.magSq(body.constantForce) > 0 || draggedVector?.type === 'force' || (dragMode === 'tool_force' && body.selected)) {
            drawInteractiveVector(ctx, body.constantForce, '#ef4444', 'F_app', 1.0, true);
        }
        
        if (body.showVelocity || draggedVector?.type === 'velocity' || (dragMode === 'tool_velocity' && body.selected)) {
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
              ctx.strokeStyle = body.color; ctx.globalAlpha = 0.5; ctx.lineWidth = 1 / state.camera.zoom; ctx.stroke();
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
            
            const isSelected = c.id === state.selectedConstraintId;
            ctx.shadowBlur = isSelected ? 8 : 0;
            ctx.shadowColor = '#fff';

            ctx.beginPath(); ctx.moveTo(pA.x, pA.y); ctx.lineTo(pB.x, pB.y);
            const w = (c.type === ConstraintType.SPRING ? 3 : 5) / state.camera.zoom;
            ctx.lineWidth = w;
            ctx.strokeStyle = isSelected ? '#fff' : (c.type === ConstraintType.SPRING ? '#fbbf24' : '#94a3b8'); 
            
            if (c.type === ConstraintType.SPRING) drawSpring(ctx, pA, pB, 6 / state.camera.zoom);
            else if (c.type === ConstraintType.ROD) { 
                ctx.stroke(); 
                ctx.fillStyle = '#94a3b8'; 
                // Anchors
                ctx.beginPath(); ctx.arc(pA.x, pA.y, w, 0, Math.PI*2); ctx.fill(); 
                ctx.beginPath(); ctx.arc(pB.x, pB.y, w, 0, Math.PI*2); ctx.fill(); 
            }
            else if (c.type === ConstraintType.PIN) { 
                ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(pA.x, pA.y, 6/state.camera.zoom, 0, Math.PI*2); ctx.fill(); 
                ctx.strokeStyle = '#fff'; ctx.lineWidth = 2/state.camera.zoom; ctx.stroke(); 
            }
            
            ctx.shadowBlur = 0;
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

      // Vector Builder Tool Visualization
      if (vectorBuilder) {
          const body = state.bodies.find(b => b.id === vectorBuilder.bodyId);
          if (body) {
              ctx.save();
              ctx.translate(body.position.x, body.position.y);
              
              // Vector from body center to mouse
              const targetPos = Vec2.sub(snappedMousePos, body.position);
              const isForce = dragMode === 'tool_force';

              ctx.beginPath();
              ctx.moveTo(0, 0);
              ctx.lineTo(targetPos.x, targetPos.y);
              ctx.strokeStyle = isForce ? '#ef4444' : '#3b82f6';
              ctx.lineWidth = 3 / state.camera.zoom;
              ctx.setLineDash([4 / state.camera.zoom, 4 / state.camera.zoom]);
              ctx.stroke();
              ctx.setLineDash([]);
              
              const angle = Math.atan2(targetPos.y, targetPos.x);
              const headLen = 10 / state.camera.zoom;
              ctx.beginPath();
              ctx.moveTo(targetPos.x, targetPos.y);
              ctx.lineTo(targetPos.x - headLen * Math.cos(angle - Math.PI / 6), targetPos.y - headLen * Math.sin(angle - Math.PI / 6));
              ctx.lineTo(targetPos.x - headLen * Math.cos(angle + Math.PI / 6), targetPos.y - headLen * Math.sin(angle + Math.PI / 6));
              ctx.lineTo(targetPos.x, targetPos.y);
              ctx.fillStyle = isForce ? '#ef4444' : '#3b82f6';
              ctx.fill();

              const mag = Vec2.mag(targetPos); 
              ctx.fillStyle = '#fff';
              ctx.font = `${12/state.camera.zoom}px monospace`;
              const label = isForce ? `F: ${mag.toFixed(0)} N` : `v: ${mag.toFixed(1)} m/s`;
              ctx.fillText(label, targetPos.x + 10/state.camera.zoom, targetPos.y);

              ctx.restore();
          }
      }

      // Arc Creation Overlay
      if (arcCreation) {
          ctx.save();
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2 / state.camera.zoom;
          ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);

          if (arcCreation.phase >= 1) {
              const currentRadius = arcCreation.phase === 1 
                  ? Vec2.dist(arcCreation.center, snappedMousePos) 
                  : arcCreation.radius;
                  
              ctx.beginPath();
              // In phase 1, allow full circle visualization
              // In phase 2, we show the arc being built
              if (arcCreation.phase === 1) {
                  ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI * 2);
                  ctx.stroke();
              } else {
                  // Draw from startAngle to current mouse angle
                  const dx = snappedMousePos.x - arcCreation.center.x;
                  const dy = snappedMousePos.y - arcCreation.center.y;
                  const mouseAngle = Math.atan2(dy, dx);
                  // Normalize for proper drawing direction logic if needed, but simple arc works
                  ctx.beginPath();
                  ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, arcCreation.endAngle, mouseAngle);
                  ctx.stroke();
              }
              
              ctx.fillStyle = '#38bdf8';
              ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill();

              // Draw radius line
              ctx.beginPath(); ctx.moveTo(arcCreation.center.x, arcCreation.center.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y); 
              ctx.strokeStyle = 'rgba(56, 189, 248, 0.4)'; ctx.stroke();
          }
          ctx.restore();
      }

      // Ramp Overlay
      if (rampCreation) {
          ctx.save();
          ctx.beginPath();
          ctx.strokeStyle = 'rgba(255,255,255,0.2)';
          ctx.lineWidth = 1 / state.camera.zoom;
          ctx.setLineDash([2 / state.camera.zoom, 4 / state.camera.zoom]);
          ctx.moveTo(rampCreation.start.x - 500, rampCreation.start.y); ctx.lineTo(rampCreation.start.x + 500, rampCreation.start.y);
          ctx.moveTo(rampCreation.start.x, rampCreation.start.y - 500); ctx.lineTo(rampCreation.start.x, rampCreation.start.y + 500);
          ctx.stroke();

          ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 2 / state.camera.zoom; ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);
          ctx.beginPath(); ctx.moveTo(rampCreation.start.x, rampCreation.start.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y); ctx.stroke();
          
          ctx.fillStyle = '#38bdf8'; 
          ctx.beginPath(); ctx.arc(rampCreation.start.x, rampCreation.start.y, 4/state.camera.zoom, 0, Math.PI*2); ctx.fill(); 
          ctx.beginPath(); ctx.arc(snappedMousePos.x, snappedMousePos.y, 4/state.camera.zoom, 0, Math.PI*2); ctx.fill();
          
          const delta = Vec2.sub(snappedMousePos, rampCreation.start);
          const angleRad = Math.atan2(delta.y, delta.x);
          const angleDeg = (angleRad * 180 / Math.PI).toFixed(1);
          ctx.font = `${12/state.camera.zoom}px monospace`; ctx.fillStyle = '#fff';
          ctx.fillText(`${angleDeg}°`, snappedMousePos.x + 10/state.camera.zoom, snappedMousePos.y - 10/state.camera.zoom);
          ctx.restore();
      }
      
      if (dragMode.startsWith('add_') || draggedBodyId) {
          ctx.save(); ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1 / state.camera.zoom;
          const r = 5 / state.camera.zoom;
          ctx.beginPath(); ctx.moveTo(snappedMousePos.x - r, snappedMousePos.y); ctx.lineTo(snappedMousePos.x + r, snappedMousePos.y);
          ctx.moveTo(snappedMousePos.x, snappedMousePos.y - r); ctx.lineTo(snappedMousePos.x, snappedMousePos.y + r);
          ctx.stroke(); ctx.restore();
      }

      ctx.restore();
    };

    render();
  }, [state, mousePos, snappedMousePos, arcCreation, rampCreation, draggedBodyId, dragMode, constraintBuilder, vectorBuilder, fieldBuilder, draggedVector]);

  const getFieldLabel = (type: FieldType) => {
      switch(type) {
          case FieldType.UNIFORM_ELECTRIC: return "Electric Field";
          case FieldType.UNIFORM_MAGNETIC: return "Magnetic Field";
          case FieldType.AREA_GRAVITY: return "Gravity Zone";
          case FieldType.CUSTOM: return "Custom Field";
          default: return "Field";
      }
  };

  const drawVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number, dashed: boolean = false) => {
      const visualVec = Vec2.mul(vec, scale); 
      const len = Vec2.mag(visualVec);
      const zoom = state.camera.zoom;
      if (len < 3 / zoom) return; 

      ctx.save();
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(visualVec.x, visualVec.y);
      ctx.strokeStyle = color;
      ctx.lineWidth = (dashed ? 1.5 : 2) / zoom;
      if (dashed) ctx.setLineDash([4/zoom, 2/zoom]);
      ctx.stroke();
      ctx.setLineDash([]);

      const angle = Math.atan2(visualVec.y, visualVec.x);
      const headLen = 8 / zoom;
      ctx.beginPath();
      ctx.moveTo(visualVec.x, visualVec.y);
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle - Math.PI / 6), visualVec.y - headLen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle + Math.PI / 6), visualVec.y - headLen * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();

      ctx.fillStyle = color;
      ctx.font = `${10/zoom}px monospace`;
      ctx.fillText(label, visualVec.x + 5/zoom, visualVec.y + 5/zoom);
      ctx.restore();
  };

  const drawInteractiveVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number = 1.0, isInteractive: boolean) => {
      const zoom = state.camera.zoom;
      const visualVec = Vec2.mul(vec, scale);
      const px = visualVec.x;
      const py = visualVec.y;
      
      ctx.save();
      
      if (Math.abs(px) > 0.1/zoom || Math.abs(py) > 0.1/zoom) {
          ctx.beginPath();
          ctx.moveTo(0, 0);
          ctx.lineTo(px, py);
          ctx.strokeStyle = color;
          ctx.lineWidth = 2 / zoom;
          ctx.stroke();

          // Arrow Head
          const angle = Math.atan2(py, px);
          const headLen = 10 / zoom;
          ctx.beginPath();
          ctx.moveTo(px, py);
          ctx.lineTo(px - headLen * Math.cos(angle - Math.PI / 6), py - headLen * Math.sin(angle - Math.PI / 6));
          ctx.lineTo(px - headLen * Math.cos(angle + Math.PI / 6), py - headLen * Math.sin(angle + Math.PI / 6));
          ctx.closePath();
          ctx.fillStyle = color;
          ctx.fill();
      }

      // Interaction Handle (Circle at tip)
      if (isInteractive) {
          ctx.beginPath();
          // Constant screen size (~3px radius)
          const handleR = 3 / zoom; 
          ctx.arc(px, py, handleR, 0, Math.PI * 2); 
          ctx.fillStyle = color;
          ctx.fill();
          ctx.strokeStyle = '#fff';
          ctx.lineWidth = 1 / zoom;
          ctx.stroke();
      }
      
      if (Math.abs(px) > 0.1 || Math.abs(py) > 0.1) {
          ctx.fillStyle = color;
          ctx.font = `${10/zoom}px monospace`;
          const mag = Vec2.mag(vec);
          ctx.fillText(`${label}: ${mag.toFixed(1)}`, px + 10/zoom, py + 5/zoom);
      }

      ctx.restore();
  };

  const drawGrid = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
    const zoom = state.camera.zoom;
    const gridSize = getGridStep(zoom);
    
    // Viewport bounds in world space
    const farLeft = ((-state.camera.x) / zoom);
    const farTop = ((-state.camera.y) / zoom);
    const farRight = farLeft + (width) / zoom;
    const farBottom = farTop + (height) / zoom;
    
    ctx.save();
    // Apply camera for lines
    ctx.translate(state.camera.x, state.camera.y);
    ctx.scale(zoom, zoom);

    // Main Grid
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'; 
    ctx.lineWidth = 1 / zoom;

    const startX = Math.floor(farLeft/gridSize)*gridSize;
    const endX = farRight + gridSize;
    const startY = Math.floor(farTop/gridSize)*gridSize;
    const endY = farBottom + gridSize;

    ctx.beginPath();
    for(let x = startX; x < endX; x+=gridSize) {
        if (Math.abs(x) < 0.1) continue; 
        ctx.moveTo(x, farTop); ctx.lineTo(x, farBottom);
    }
    for(let y = startY; y < endY; y+=gridSize) {
        if (Math.abs(y) < 0.1) continue; 
        ctx.moveTo(farLeft, y); ctx.lineTo(farRight, y);
    }
    ctx.stroke();
    
    // Axis
    ctx.strokeStyle = '#94a3b8'; 
    ctx.lineWidth = 2 / zoom; 
    ctx.beginPath(); 
    ctx.moveTo(farLeft, 0); ctx.lineTo(farRight, 0); 
    ctx.moveTo(0, farTop); ctx.lineTo(0, farBottom); 
    ctx.stroke(); 
    
    // Restore transform to draw text in screen space for clarity/positioning
    ctx.restore();

    // Text rendering in screen space
    ctx.fillStyle = 'rgba(255,255,255,0.4)'; 
    ctx.font = '10px monospace';
    
    // Function to map world to screen
    const w2sX = (x: number) => x * zoom + state.camera.x;
    const w2sY = (y: number) => y * zoom + state.camera.y;

    // Draw X Axis numbers
    for(let x = startX; x < endX; x+=gridSize) {
        if (Math.abs(x) < gridSize/2) continue;
        const sx = w2sX(x);
        // Clamp text Y to screen edges
        const sy = Math.max(15, Math.min(height - 15, w2sY(0) + 15));
        ctx.fillText(parseFloat(x.toFixed(2)).toString(), sx + 2, sy);
    }
    // Draw Y Axis numbers
    for(let y = startY; y < endY; y+=gridSize) {
         if (Math.abs(y) < gridSize/2) continue;
         const sy = w2sY(y);
         // Clamp text X to screen edges
         const sx = Math.max(5, Math.min(width - 30, w2sX(0) + 5));
         ctx.fillText(parseFloat((-y).toFixed(2)).toString(), sx, sy - 2); 
    }
    
    // Origin
    ctx.fillStyle = '#94a3b8';
    ctx.font = '12px monospace';
    const originX = Math.max(5, Math.min(width - 40, w2sX(0) + 5));
    const originY = Math.max(15, Math.min(height - 5, w2sY(0) + 15));
    ctx.fillText("(0,0)", originX, originY);
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

  // Improved Hit Testing Function
  type HitResult = 
    | { type: 'handle'; bodyId: string; handleType: 'velocity' | 'force' }
    | { type: 'body'; id: string }
    | { type: 'constraint'; id: string }
    | { type: 'field'; id: string };

  const hitTest = (worldPos: Vector2, hitRadius: number): HitResult | null => {
      // Priority 1: Handles (Vector Interactions)
      if (!dragMode.startsWith('add_') && !dragMode.startsWith('tool_')) {
        for (const body of state.bodies) {
            const checkHandle = (vec: Vector2, type: 'velocity' | 'force'): HitResult | null => {
                const tip = Vec2.add(body.position, vec);
                // Adjusted hit radius for handles
                if (Vec2.dist(worldPos, tip) < Math.max(5 / state.camera.zoom, hitRadius * 1.5)) {
                    return { type: 'handle', bodyId: body.id, handleType: type };
                }
                return null;
            };
            if (Vec2.magSq(body.constantForce) > 0) {
                const h = checkHandle(body.constantForce, 'force');
                if (h) return h;
            }
            if (body.showVelocity) {
                const h = checkHandle(body.velocity, 'velocity');
                if (h) return h;
            }
        }
      }

      // Priority 2: Constraints (Lines are thin, harder to hit, so check before bodies if they overlay)
      // Iterate reverse to select top-most if multiple
      for (let i = state.constraints.length - 1; i >= 0; i--) {
          const c = state.constraints[i];
          const bodyA = state.bodies.find(b => b.id === c.bodyAId);
          const bodyB = state.bodies.find(b => b.id === c.bodyBId);
          if (bodyA && bodyB) {
              const pA = Vec2.transform(c.localAnchorA, bodyA.position, bodyA.angle);
              const pB = Vec2.transform(c.localAnchorB, bodyB.position, bodyB.angle);
              
              const l2 = Vec2.magSq(Vec2.sub(pA, pB));
              if (l2 === 0) continue;
              const t = Math.max(0, Math.min(1, Vec2.dot(Vec2.sub(worldPos, pA), Vec2.sub(pB, pA)) / l2));
              const proj = Vec2.add(pA, Vec2.mul(Vec2.sub(pB, pA), t));
              const dist = Vec2.dist(worldPos, proj);
              
              // Use screen-space constant threshold (e.g., 6px)
              if (dist < 6 / state.camera.zoom) {
                  return { type: 'constraint', id: c.id };
              }
          }
      }

      // Priority 3: Bodies (Top to Bottom)
      for(let i=state.bodies.length-1; i>=0; i--) {
        const b = state.bodies[i];
        
        if (b.type === BodyType.BOX || b.type === BodyType.POLYGON) {
             const localPos = Vec2.rotate(Vec2.sub(worldPos, b.position), -b.angle);
             
             if (b.type === BodyType.BOX) {
                 const w = b.width || 40; 
                 const h = b.height || 40;
                 // Add hitRadius to dimensions to make selection easier
                 if (Math.abs(localPos.x) <= (w/2 + hitRadius) && Math.abs(localPos.y) <= (h/2 + hitRadius)) {
                     return { type: 'body', id: b.id };
                 }
             } else if (b.vertices) {
                 // Simple AABB for Poly
                 let minX=Infinity, maxX=-Infinity, minY=Infinity, maxY=-Infinity;
                 b.vertices.forEach(v => {
                     minX = Math.min(minX, v.x); maxX = Math.max(maxX, v.x);
                     minY = Math.min(minY, v.y); maxY = Math.max(maxY, v.y);
                 });
                 if (localPos.x >= minX - hitRadius && localPos.x <= maxX + hitRadius &&
                     localPos.y >= minY - hitRadius && localPos.y <= maxY + hitRadius) {
                         return { type: 'body', id: b.id };
                 }
             }
        } 
        else if (b.type === BodyType.CIRCLE || b.type === BodyType.ARC) {
            const dist = Vec2.dist(worldPos, b.position);
            const r = b.radius || 20;
            if (b.type === BodyType.ARC) {
                 // Select Arc if near the ring
                 if (Math.abs(dist - r) < Math.max(10 / state.camera.zoom, hitRadius)) {
                     return { type: 'body', id: b.id };
                 }
            } else {
                 if (dist < r + hitRadius) {
                     return { type: 'body', id: b.id };
                 }
            }
        }
      }

      // Priority 4: Fields
      // Iterate reverse (top rendered first)
      for (let i = state.fields.length - 1; i >= 0; i--) {
          const f = state.fields[i];
          if (f.shape === FieldShape.BOX) {
              if (worldPos.x >= f.position.x && worldPos.x <= f.position.x + f.size.x &&
                  worldPos.y >= f.position.y && worldPos.y <= f.position.y + f.size.y) {
                      return { type: 'field', id: f.id };
              }
          } else if (f.shape === FieldShape.CIRCLE) {
              if (Vec2.dist(worldPos, f.position) < (f.radius || 100)) {
                  return { type: 'field', id: f.id };
              }
          } else if (f.shape === FieldShape.POLYGON && f.vertices && f.vertices.length > 0) {
              // Basic Poly check (Vertex proximity or AABB)
              // For full poly selection, we check if mouse is near any vertex or centroid for simplicity
              let inPoly = false;
              // A simple hit test for polygon fields (Ray casting would be better but expensive)
              // Let's use bounding box + proximity
              let minX=Infinity, maxX=-Infinity, minY=Infinity, maxY=-Infinity;
              f.vertices.forEach(v => {
                  minX = Math.min(minX, v.x); maxX = Math.max(maxX, v.x);
                  minY = Math.min(minY, v.y); maxY = Math.max(maxY, v.y);
              });
              if (worldPos.x >= minX && worldPos.x <= maxX && worldPos.y >= minY && worldPos.y <= maxY) {
                   return { type: 'field', id: f.id };
              }
          }
      }

      return null;
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    const rect = canvasRef.current!.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const worldPos = screenToWorld(sx, sy);
    const snapped = snapToGrid(worldPos);

    setDragStart({ x: e.clientX, y: e.clientY });
    setHasMoved(false);

    // Constant screen hit radius converted to world
    const hitRadiusWorld = 8 / state.camera.zoom;

    // --- Field Builder Logic ---
    if (dragMode.startsWith('add_field_')) {
        const shape = dragMode === 'add_field_circle' ? FieldShape.CIRCLE : 
                      dragMode === 'add_field_poly' ? FieldShape.POLYGON : FieldShape.BOX;
        
        if (dragMode === 'add_field_poly') {
            if (!fieldBuilder) {
                setFieldBuilder({ start: snapped, shape, vertices: [snapped] });
            } else {
                setFieldBuilder({ ...fieldBuilder, vertices: [...(fieldBuilder.vertices || []), snapped] });
            }
        } else {
            if (!fieldBuilder) {
                setFieldBuilder({ start: snapped, shape });
            }
        }
        return;
    }

    // --- Vector Tool Logic ---
    if (dragMode === 'tool_velocity' || dragMode === 'tool_force') {
        const hit = hitTest(worldPos, hitRadiusWorld);
        if (hit && hit.type === 'body') {
            onPauseToggle(true);
            onSelectBody(hit.id);
            setVectorBuilder({ bodyId: hit.id, startPos: worldPos }); 
        }
        return;
    }

    // --- Add Objects Logic ---
    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin', 'add_field_box', 'add_field_circle', 'add_field_poly'].includes(dragMode)) {
        onAddBody(snapped);
        return;
    }

    // --- Selection Logic ---
    const hit = hitTest(worldPos, hitRadiusWorld);

    if (hit) {
        if (hit.type === 'handle') {
             setDraggedVector({ bodyId: hit.bodyId!, type: hit.handleType as any });
             onSelectBody(hit.bodyId!);
             onPauseToggle(true);
        } else if (hit.type === 'body') {
             if (['add_spring', 'add_rod', 'add_pin'].includes(dragMode)) { 
                 onAddBody(worldPos); 
                 return; 
             }
             setDraggedBodyId(hit.id!);
             onSelectBody(hit.id!);
             onSelectConstraint(null);
             onSelectField(null);
        } else if (hit.type === 'constraint') {
             onSelectConstraint(hit.id!);
             onSelectBody(null);
             onSelectField(null);
        } else if (hit.type === 'field') {
             onSelectField(hit.id!);
             onSelectBody(null);
             onSelectConstraint(null);
        }
    } else {
        // Pan
        setIsDraggingCam(true);
    }
  };

  const handleMouseMove = (e: React.MouseEvent) => {
      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const worldPos = screenToWorld(sx, sy);
      setMousePos(worldPos); 
      
      const moveDist = Math.sqrt(Math.pow(e.clientX - dragStart.x, 2) + Math.pow(e.clientY - dragStart.y, 2));
      if (moveDist > 3) setHasMoved(true);

      // --- Snapping Logic ---
      setSnappedMousePos(snapToGrid(worldPos));
      
      // --- Handle Vector Dragging (Interaction) ---
      if (draggedVector) {
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              const rawVec = Vec2.sub(worldPos, body.position); // Calculate vector based on mouse relative to body center
              
              // Angle Snapping for Dragging
              let snappedVec = rawVec;
              const angleRad = Math.atan2(rawVec.y, rawVec.x);
              const angleDeg = angleRad * 180 / Math.PI;
              const normDeg = (angleDeg % 360);
              
              if (Math.abs(normDeg) < 5 || Math.abs(Math.abs(normDeg) - 180) < 5) snappedVec = { x: rawVec.x, y: 0 };
              else if (Math.abs(Math.abs(normDeg) - 90) < 5) snappedVec = { x: 0, y: rawVec.y };

              onVectorEdit(draggedVector.bodyId, draggedVector.type, snappedVec);
          }
          return;
      }

      // --- Handle Vector Builder Dragging ---
      if (vectorBuilder) {
          return; // Visual only
      }

      if (draggedBodyId && hasMoved) {
          if (dragMode !== 'select_nodrag') {
              onMoveBody(draggedBodyId, snappedMousePos); 
          }
          return;
      }
      if (isDraggingCam) {
           onZoom(0, sx + (dragStart.x - e.clientX), sy + (dragStart.y - e.clientY)); 
      }
  };

  const handleMouseUp = (e: React.MouseEvent) => {
      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const worldPos = screenToWorld(sx, sy);
      
      if (isDraggingCam) setIsDraggingCam(false);
      setDraggedBodyId(null);
      
      if (draggedVector) {
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              const vec = draggedVector.type === 'velocity' ? body.velocity : body.constantForce;
              // Threshold logic: 10 pixels visually
              if (Vec2.mag(vec) * state.camera.zoom < 10) { 
                  onVectorEdit(draggedVector.bodyId, draggedVector.type, {x:0, y:0});
              }
          }
          setDraggedVector(null);
      }
      
      // Finish Vector Building (Tool Mode)
      if (vectorBuilder) {
          const body = state.bodies.find(b => b.id === vectorBuilder.bodyId);
          if (body) {
              const rawVec = Vec2.sub(snappedMousePos, body.position);
              
              let finalVec = rawVec;
              const angleRad = Math.atan2(rawVec.y, rawVec.x);
              const angleDeg = angleRad * 180 / Math.PI;
              const normDeg = (angleDeg % 360);

              if (Math.abs(normDeg) < 5 || Math.abs(Math.abs(normDeg) - 180) < 5) finalVec = { x: rawVec.x, y: 0 };
              else if (Math.abs(Math.abs(normDeg) - 90) < 5) finalVec = { x: 0, y: rawVec.y };
              
              onVectorEdit(vectorBuilder.bodyId, dragMode === 'tool_velocity' ? 'velocity' : 'force', finalVec);
          }
          setVectorBuilder(null);
      }

      // Finish Field Building (Box/Circle)
      if (fieldBuilder && dragMode !== 'add_field_poly') {
           if (dragMode === 'add_field_box') {
               const w = snappedMousePos.x - fieldBuilder.start.x;
               const h = snappedMousePos.y - fieldBuilder.start.y;
               if (Math.abs(w) > 5 && Math.abs(h) > 5) {
                   const x = w > 0 ? fieldBuilder.start.x : snappedMousePos.x;
                   const y = h > 0 ? fieldBuilder.start.y : snappedMousePos.y;
                   onAddField({x, y}, FieldShape.BOX, [{x: Math.abs(w), y: Math.abs(h)}]);
               }
           } else if (dragMode === 'add_field_circle') {
               const r = Vec2.dist(fieldBuilder.start, snappedMousePos);
               if (r > 5) {
                   onAddField(fieldBuilder.start, FieldShape.CIRCLE, [{x: r, y: 0}]);
               }
           }
           setFieldBuilder(null);
      } else if (!hasMoved && !draggedBodyId && !isDraggingCam && !draggedVector && !dragMode.startsWith('add_') && !dragMode.startsWith('tool_')) {
          // Pure Click for Selection (if MouseDown didn't catch dragging)
          const hit = hitTest(worldPos, 10 / state.camera.zoom);
          if (!hit) {
              onSelectBody(null);
              onSelectConstraint(null);
              onSelectField(null);
          }
      }
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
      className={`w-full h-full block ${dragMode === 'select_nodrag' ? 'cursor-default' : 'cursor-crosshair'}`}
      onMouseDown={handleMouseDown}
      onWheel={handleWheel}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
    />
  );
});

export default SimulationCanvas;