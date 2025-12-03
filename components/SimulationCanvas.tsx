
import React, { useRef, useEffect, useState, useImperativeHandle, forwardRef } from 'react';
import { BodyType, SimulationState, FieldType, Vector2, FieldShape, ConstraintType } from '../types';
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
  booleanOpBuilder: { op: 'combine' | 'cut', subjectId: string } | null;
  knifeBuilder: { start: Vector2 } | null;
  onKnifeCut: (start: Vector2, end: Vector2) => void;
}

export interface CanvasRef {
  exportImage: () => void;
}

const SimulationCanvas = forwardRef<CanvasRef, Props>(({ state, onSelectBody, onSelectField, onSelectConstraint, onAddBody, onAddField, onMoveBody, onVectorEdit, dragMode, onZoom, onPauseToggle, arcCreation, rampCreation, constraintBuilder, booleanOpBuilder, knifeBuilder, onKnifeCut }, ref) => {
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

  const getGridStep = (zoom: number) => {
    // 1-2-5 scaling step
    const targetPx = 80; 
    const rawVal = targetPx / zoom;
    const power = Math.floor(Math.log10(rawVal));
    const base = Math.pow(10, power);
    const mult = rawVal / base;
    
    let step;
    if (mult < 2) step = 1 * base;
    else if (mult < 5) step = 2 * base;
    else step = 5 * base;
    
    return step;
  };

  const snapToGrid = (v: Vector2): Vector2 => {
      const gridSize = getGridStep(state.camera.zoom);
      
      // Strict snapping
      const snappedX = Math.round(v.x / gridSize) * gridSize;
      const snappedY = Math.round(v.y / gridSize) * gridSize;

      // Calculate pixel distance to snap point
      const distSq = Math.pow((v.x - snappedX) * state.camera.zoom, 2) + Math.pow((v.y - snappedY) * state.camera.zoom, 2);
      const SNAP_THRESHOLD_PX_SQ = 15 * 15;

      if (distSq < SNAP_THRESHOLD_PX_SQ) {
          return { x: snappedX, y: snappedY };
      }
      return { x: v.x, y: v.y }; 
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
      drawGrid(ctx);

      // Fields (Background)
      state.fields.forEach(field => {
        if (!field.visible) return;
        
        ctx.save();
        const isSelected = field.id === state.selectedFieldId;
        const alpha = isSelected ? 0.3 : 0.15;
        const strokeColor = field.type === FieldType.UNIFORM_ELECTRIC ? '#ef4444' 
                        : field.type === FieldType.UNIFORM_MAGNETIC ? '#3b82f6' 
                        : field.type === FieldType.CUSTOM ? '#d946ef'
                        : '#10b981';
        
        ctx.fillStyle = field.type === FieldType.UNIFORM_ELECTRIC ? `rgba(239, 68, 68, ${alpha})` 
                      : field.type === FieldType.UNIFORM_MAGNETIC ? `rgba(59, 130, 246, ${alpha})`
                      : field.type === FieldType.CUSTOM ? `rgba(217, 70, 239, ${alpha})`
                      : `rgba(16, 185, 129, ${alpha})`;
        
        ctx.strokeStyle = strokeColor;
        ctx.lineWidth = isSelected ? 2 / state.camera.zoom : 1 / state.camera.zoom;
        ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);

        // Pattern Drawing
        const drawFieldPattern = (cx: number, cy: number, w: number, h: number) => {
            if (field.type === FieldType.UNIFORM_MAGNETIC) {
                const strength = field.strength as number;
                const densityFactor = Math.min(2, Math.max(0.5, Math.sqrt(Math.abs(strength)) / 5));
                const baseSpacing = 50 / (state.camera.zoom * densityFactor);
                
                const startX = cx; const startY = cy;
                const size = (6 / state.camera.zoom) * densityFactor;

                for (let x = startX + baseSpacing/2; x < startX + w; x += baseSpacing) {
                    for (let y = startY + baseSpacing/2; y < startY + h; y += baseSpacing) {
                        ctx.strokeStyle = strokeColor;
                        ctx.fillStyle = strokeColor;
                        ctx.lineWidth = 1.5 / state.camera.zoom;
                        ctx.setLineDash([]);

                        if (strength > 0) {
                            // Out of page (Dot/Circle)
                            ctx.beginPath();
                            ctx.arc(x, y, size/2, 0, Math.PI*2);
                            ctx.fill();
                            ctx.beginPath();
                            ctx.arc(x, y, size, 0, Math.PI*2);
                            ctx.stroke();
                        } else {
                            // Into page (X)
                            ctx.beginPath();
                            ctx.moveTo(x - size, y - size);
                            ctx.lineTo(x + size, y + size);
                            ctx.moveTo(x + size, y - size);
                            ctx.lineTo(x - size, y + size);
                            ctx.stroke();
                        }
                    }
                }
            } else if (field.type === FieldType.UNIFORM_ELECTRIC) {
                const E = field.strength as Vector2;
                const mag = Vec2.mag(E);
                const densityFactor = Math.min(2, Math.max(0.5, Math.sqrt(mag) / 3));
                const baseSpacing = 60 / (state.camera.zoom * densityFactor);
                const angle = Math.atan2(E.y, E.x);
                
                const startX = cx; const startY = cy;
                const arrowLen = 15 / state.camera.zoom;

                for (let x = startX + baseSpacing/2; x < startX + w; x += baseSpacing) {
                    for (let y = startY + baseSpacing/2; y < startY + h; y += baseSpacing) {
                         ctx.save();
                         ctx.translate(x, y);
                         ctx.rotate(angle);
                         ctx.beginPath();
                         ctx.moveTo(-arrowLen/2, 0);
                         ctx.lineTo(arrowLen/2, 0);
                         ctx.lineTo(arrowLen/2 - 4/state.camera.zoom, -4/state.camera.zoom);
                         ctx.moveTo(arrowLen/2, 0);
                         ctx.lineTo(arrowLen/2 - 4/state.camera.zoom, 4/state.camera.zoom);
                         ctx.strokeStyle = 'rgba(239, 68, 68, 0.6)';
                         ctx.lineWidth = 1.5 / state.camera.zoom;
                         ctx.setLineDash([]);
                         ctx.stroke();
                         ctx.restore();
                    }
                }
            } else if (field.type === FieldType.CUSTOM) {
                // Draw a function f(x,y) visualization - simplified grid points
                const baseSpacing = 60 / state.camera.zoom;
                ctx.fillStyle = 'rgba(217, 70, 239, 0.5)';
                for (let x = cx + baseSpacing/2; x < cx + w; x += baseSpacing) {
                    for (let y = cy + baseSpacing/2; y < cy + h; y += baseSpacing) {
                        ctx.beginPath();
                        ctx.arc(x, y, 2 / state.camera.zoom, 0, Math.PI*2);
                        ctx.fill();
                    }
                }
            }
        };

        if (field.shape === FieldShape.BOX) {
            ctx.fillRect(field.position.x, field.position.y, field.size.x, field.size.y);
            ctx.strokeRect(field.position.x, field.position.y, field.size.x, field.size.y);
            ctx.save();
            ctx.beginPath(); ctx.rect(field.position.x, field.position.y, field.size.x, field.size.y); ctx.clip();
            drawFieldPattern(field.position.x, field.position.y, field.size.x, field.size.y);
            ctx.restore();
        } else if (field.shape === FieldShape.CIRCLE) {
            const r = field.radius || 100;
            ctx.beginPath();
            ctx.arc(field.position.x, field.position.y, r, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();
            
            ctx.save();
            ctx.beginPath(); ctx.arc(field.position.x, field.position.y, r, 0, Math.PI*2); ctx.clip();
            drawFieldPattern(field.position.x - r, field.position.y - r, r*2, r*2);
            ctx.restore();
        } else if (field.shape === FieldShape.POLYGON && field.vertices) {
            ctx.beginPath();
            if (field.vertices.length > 0) {
                ctx.moveTo(field.vertices[0].x, field.vertices[0].y);
                for(let i=1; i<field.vertices.length; i++) ctx.lineTo(field.vertices[i].x, field.vertices[i].y);
            }
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            
            if (field.vertices.length > 0) {
                 let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
                 field.vertices.forEach(v => {
                     if(v.x<minX) minX=v.x; if(v.y<minY) minY=v.y;
                     if(v.x>maxX) maxX=v.x; if(v.y>maxY) maxY=v.y;
                 });
                 ctx.save();
                 ctx.beginPath();
                 ctx.moveTo(field.vertices[0].x, field.vertices[0].y);
                 for(let i=1; i<field.vertices.length; i++) ctx.lineTo(field.vertices[i].x, field.vertices[i].y);
                 ctx.closePath();
                 ctx.clip();
                 drawFieldPattern(minX, minY, maxX-minX, maxY-minY);
                 ctx.restore();
            }
        }
        
        // Label
        const labelPos = field.shape === FieldShape.BOX ? {x: field.position.x, y: field.position.y - 10/state.camera.zoom} :
                         field.shape === FieldShape.CIRCLE ? {x: field.position.x - (field.radius||0), y: field.position.y - (field.radius||0) - 10/state.camera.zoom} :
                         field.vertices && field.vertices.length > 0 ? {x: field.vertices[0].x, y: field.vertices[0].y - 10/state.camera.zoom} : field.position;
        
        ctx.fillStyle = ctx.strokeStyle;
        ctx.font = `${12/state.camera.zoom}px sans-serif`;
        ctx.fillText(getFieldLabel(field.type), labelPos.x, labelPos.y);

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
                  ctx.beginPath(); ctx.arc(v.x, v.y, 3 / state.camera.zoom, 0, Math.PI*2); ctx.fill();
              });
              // Close hint
              if (fieldBuilder.vertices.length > 2) {
                   const start = fieldBuilder.vertices[0];
                   if (Vec2.dist(snappedMousePos, start) < 20 / state.camera.zoom) {
                       ctx.fillStyle = '#4ade80';
                       ctx.beginPath(); ctx.arc(start.x, start.y, 6/state.camera.zoom, 0, Math.PI*2); ctx.fill();
                   }
              }
          }
          ctx.restore();
      }

      // Bodies
      state.bodies.forEach(body => {
        ctx.save();
        ctx.translate(body.position.x, body.position.y);
        ctx.save();
        ctx.rotate(body.angle);
        
        ctx.fillStyle = body.color;
        const isSelected = body.id === state.selectedBodyId;
        const isSubject = booleanOpBuilder?.subjectId === body.id;

        ctx.lineWidth = isSelected || isSubject ? 3 / state.camera.zoom : 2 / state.camera.zoom;
        ctx.strokeStyle = isSelected ? '#ffffff' : isSubject ? '#fbbf24' : '#000000';
        
        if (isSelected) {
            ctx.shadowColor = 'rgba(255,255,255,0.8)';
            ctx.shadowBlur = 10;
        } else if (isSubject) {
            ctx.shadowColor = 'rgba(251, 191, 36, 0.8)';
            ctx.shadowBlur = 10;
        } else {
            ctx.shadowColor = 'rgba(0,0,0,0.5)';
            ctx.shadowBlur = 5;
            ctx.shadowOffsetX = 2;
            ctx.shadowOffsetY = 2;
        }

        if (body.type === BodyType.CIRCLE) {
          ctx.beginPath();
          const r = body.isParticle ? 6 / state.camera.zoom : (body.radius || 20);
          ctx.arc(0, 0, r, 0, Math.PI * 2);
          ctx.fill();
          ctx.shadowColor = 'transparent'; 
          ctx.stroke();
          if (!body.isParticle) {
              ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(r * 0.8, 0);
              ctx.strokeStyle = 'rgba(0,0,0,0.3)'; ctx.lineWidth = 2 / state.camera.zoom; ctx.stroke();
          }
        } else if (body.type === BodyType.BOX) {
          const w = body.width || 40;
          const h = body.height || 40;
          const isRamp = h < 12 && w > h * 2; 
          if (isRamp && body.mass === 0) {
              ctx.shadowColor = 'transparent'; ctx.beginPath(); ctx.rect(-w/2, -h/2, w, h);
              ctx.fillStyle = body.color; ctx.fill();
              ctx.beginPath(); ctx.moveTo(-w/2, -h/2); ctx.lineTo(w/2, -h/2);
              ctx.strokeStyle = '#fff'; ctx.lineWidth = 2 / state.camera.zoom; ctx.stroke();
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
        } else if (body.type === BodyType.POLYGON) {
           if (body.vertices && body.vertices.length > 0) {
              ctx.beginPath(); ctx.moveTo(body.vertices[0].x, body.vertices[0].y);
              for (let i = 1; i < body.vertices.length; i++) ctx.lineTo(body.vertices[i].x, body.vertices[i].y);
              ctx.closePath(); ctx.fill(); ctx.shadowColor = 'transparent'; ctx.stroke();
          }
        } else if (body.type === BodyType.ARC) {
             const r = body.radius || 100;
             const start = body.arcStartAngle || 0;
             const end = body.arcEndAngle || Math.PI;
             ctx.shadowColor = 'transparent'; 
             ctx.beginPath(); ctx.arc(0, 0, r, start, end);
             ctx.strokeStyle = body.color; ctx.lineWidth = 5 / state.camera.zoom; ctx.stroke();
             // Draw Ticks
             ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.lineWidth = 1 / state.camera.zoom;
             const steps = 12;
             const range = end - start;
             // Avoid drawing if full circle
             if (Math.abs(range) < Math.PI * 1.99) {
                 for (let i=0; i<=steps; i++) {
                     const a = start + range*(i/steps);
                     ctx.beginPath(); ctx.moveTo(Math.cos(a)*r, Math.sin(a)*r); ctx.lineTo(Math.cos(a)*(r+10), Math.sin(a)*(r+10)); ctx.stroke();
                 }
             }
        }
        
        if (body.showCharge && body.charge !== 0) {
             const offset = body.isParticle ? -15 : 0;
             ctx.fillStyle = body.charge > 0 ? 'rgba(239, 68, 68, 0.8)' : 'rgba(59, 130, 246, 0.8)';
             ctx.font = `bold ${20/state.camera.zoom}px monospace`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
             ctx.fillText(body.charge > 0 ? '+' : '-', 0, offset / state.camera.zoom);
        }

        ctx.restore();

        // Draw Vectors
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
            ctx.beginPath(); ctx.moveTo(pA.x, pA.y); ctx.lineTo(pB.x, pB.y);
            const isSelected = state.selectedConstraintId === c.id;
            ctx.lineWidth = (c.type === ConstraintType.SPRING ? 3 : 5) / state.camera.zoom;
            ctx.strokeStyle = isSelected ? '#ffffff' : (c.type === ConstraintType.SPRING ? '#fbbf24' : '#94a3b8'); 
            
            if (c.type === ConstraintType.SPRING) drawSpring(ctx, pA, pB, 6 / state.camera.zoom);
            else if (c.type === ConstraintType.ROD) { ctx.stroke(); ctx.fillStyle = '#94a3b8'; ctx.beginPath(); ctx.arc(pA.x, pA.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); ctx.beginPath(); ctx.arc(pB.x, pB.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); }
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

      // Boolean Op Builder Line
      if (booleanOpBuilder) {
          const bodyA = state.bodies.find(b => b.id === booleanOpBuilder.subjectId);
          if (bodyA) {
              ctx.save();
              ctx.beginPath(); ctx.moveTo(bodyA.position.x, bodyA.position.y); ctx.lineTo(mousePos.x, mousePos.y);
              ctx.strokeStyle = booleanOpBuilder.op === 'combine' ? '#fbbf24' : '#ef4444';
              ctx.lineWidth = 2 / state.camera.zoom; ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]); ctx.stroke();
              ctx.restore();
          }
      }

      // Knife Builder
      if (knifeBuilder) {
          ctx.save();
          ctx.beginPath();
          ctx.moveTo(knifeBuilder.start.x, knifeBuilder.start.y);
          ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
          ctx.strokeStyle = '#f87171'; // Red-400
          ctx.lineWidth = 2 / state.camera.zoom;
          ctx.setLineDash([8 / state.camera.zoom, 4 / state.camera.zoom]);
          ctx.stroke();
          ctx.restore();
      }

      // Vector Builder Tool Visualization
      if (vectorBuilder) {
          const body = state.bodies.find(b => b.id === vectorBuilder.bodyId);
          if (body) {
              ctx.save();
              ctx.translate(body.position.x, body.position.y);
              
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
              ctx.fillText(label, targetPos.x + 10 / state.camera.zoom, targetPos.y);

              ctx.restore();
          }
      }

      // Arc Creation Overlay
      if (arcCreation) {
          ctx.save();
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2 / state.camera.zoom;
          ctx.setLineDash([5 / state.camera.zoom, 5 / state.camera.zoom]);

          if (arcCreation.phase === 1) {
              // Center chosen, creating radius
              const currentRadius = Vec2.dist(arcCreation.center, snappedMousePos);
              ctx.beginPath();
              ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI * 2);
              ctx.globalAlpha = 0.3; ctx.stroke(); ctx.globalAlpha = 1.0;
              
              ctx.beginPath(); ctx.moveTo(arcCreation.center.x, arcCreation.center.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.stroke();
          } else if (arcCreation.phase === 2) {
              // Radius chosen, creating end angle
              const currentRadius = arcCreation.radius;
              const start = arcCreation.startAngle;
              const currentPos = Vec2.sub(snappedMousePos, arcCreation.center);
              let end = Math.atan2(currentPos.y, currentPos.x);
              
              // Draw full circle hint if close
              const endPt = {x: arcCreation.center.x + Math.cos(end)*currentRadius, y: arcCreation.center.y + Math.sin(end)*currentRadius};
              const startPt = {x: arcCreation.center.x + Math.cos(start)*currentRadius, y: arcCreation.center.y + Math.sin(start)*currentRadius};
              
              if (Vec2.dist(startPt, endPt) < 20 / state.camera.zoom) {
                  ctx.strokeStyle = '#4ade80'; // Green hint for circle
                  ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI*2); ctx.stroke();
              } else {
                  ctx.strokeStyle = '#38bdf8';
                  ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, start, end, false); ctx.stroke();
              }
          }

          ctx.fillStyle = '#38bdf8';
          ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill();
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
          ctx.beginPath(); ctx.arc(rampCreation.start.x, rampCreation.start.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill(); 
          ctx.beginPath(); ctx.arc(snappedMousePos.x, snappedMousePos.y, 4 / state.camera.zoom, 0, Math.PI*2); ctx.fill();
          
          const delta = Vec2.sub(snappedMousePos, rampCreation.start);
          const angleRad = Math.atan2(delta.y, delta.x);
          const angleDeg = (angleRad * 180 / Math.PI).toFixed(1);
          ctx.font = `${12/state.camera.zoom}px monospace`; ctx.fillStyle = '#fff';
          ctx.fillText(`${angleDeg}°`, snappedMousePos.x + 10 / state.camera.zoom, snappedMousePos.y - 10 / state.camera.zoom);
          ctx.restore();
      }
      
      if (dragMode.startsWith('add_') && dragMode !== 'add_field_poly') {
          ctx.save(); ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1 / state.camera.zoom;
          ctx.beginPath(); ctx.moveTo(snappedMousePos.x - 5/state.camera.zoom, snappedMousePos.y); ctx.lineTo(snappedMousePos.x + 5/state.camera.zoom, snappedMousePos.y);
          ctx.moveTo(snappedMousePos.x, snappedMousePos.y - 5/state.camera.zoom); ctx.lineTo(snappedMousePos.x, snappedMousePos.y + 5/state.camera.zoom);
          ctx.stroke(); ctx.restore();
      }

      ctx.restore();
    };

    render();
  }, [state, mousePos, snappedMousePos, arcCreation, rampCreation, draggedBodyId, dragMode, constraintBuilder, vectorBuilder, fieldBuilder, draggedVector, booleanOpBuilder, knifeBuilder]);

  const getFieldLabel = (type: FieldType) => {
      switch(type) {
          case FieldType.UNIFORM_ELECTRIC: return "Electric (E)";
          case FieldType.UNIFORM_MAGNETIC: return "Magnetic (B)";
          case FieldType.AREA_GRAVITY: return "Gravity (g)";
          case FieldType.CUSTOM: return "Custom Field";
          default: return "Field";
      }
  };

  const drawVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number, dashed: boolean = false) => {
      const visualVec = Vec2.mul(vec, scale); 
      const len = Vec2.mag(visualVec);
      if (len * state.camera.zoom < 3) return; 

      ctx.save();
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(visualVec.x, visualVec.y);
      ctx.strokeStyle = color;
      ctx.lineWidth = (dashed ? 1.5 : 2) / state.camera.zoom;
      if (dashed) ctx.setLineDash([4 / state.camera.zoom, 2 / state.camera.zoom]);
      ctx.stroke();
      ctx.setLineDash([]);

      const angle = Math.atan2(visualVec.y, visualVec.x);
      const headLen = 8 / state.camera.zoom;
      ctx.beginPath();
      ctx.moveTo(visualVec.x, visualVec.y);
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle - Math.PI / 6), visualVec.y - headLen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle + Math.PI / 6), visualVec.y - headLen * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();

      ctx.fillStyle = color;
      ctx.font = `${10/state.camera.zoom}px monospace`;
      ctx.fillText(label, visualVec.x + 5/state.camera.zoom, visualVec.y + 5/state.camera.zoom);
      ctx.restore();
  };

  const drawInteractiveVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number = 1.0, isInteractive: boolean) => {
      const visualVec = Vec2.mul(vec, scale);
      const px = visualVec.x;
      const py = visualVec.y;
      
      ctx.save();
      
      if (Math.abs(px) > 0.1 || Math.abs(py) > 0.1) {
          ctx.beginPath();
          ctx.moveTo(0, 0);
          ctx.lineTo(px, py);
          ctx.strokeStyle = color;
          ctx.lineWidth = 2 / state.camera.zoom;
          ctx.stroke();

          // Arrow Head
          const angle = Math.atan2(py, px);
          const headLen = 10 / state.camera.zoom;
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
          // Constant screen size ~6px dia
          const handleR = 3 / state.camera.zoom; 
          ctx.arc(px, py, handleR, 0, Math.PI * 2); 
          ctx.fillStyle = color;
          ctx.fill();
          ctx.strokeStyle = '#fff';
          ctx.lineWidth = 1 / state.camera.zoom;
          ctx.stroke();
      }
      
      if (Math.abs(px) > 0.1 || Math.abs(py) > 0.1) {
          ctx.fillStyle = color;
          ctx.font = `${10/state.camera.zoom}px monospace`;
          const mag = Vec2.mag(vec);
          ctx.fillText(`${label}: ${mag.toFixed(1)}`, px + 10/state.camera.zoom, py + 5/state.camera.zoom);
      }

      ctx.restore();
  };

  const drawGrid = (ctx: CanvasRenderingContext2D) => {
    const zoom = state.camera.zoom;
    const gridSize = getGridStep(zoom);
    
    // Bounds in World Coords
    const farLeft = ((-state.camera.x / zoom));
    const farTop = ((-state.camera.y / zoom));
    const w = (ctx.canvas.width / window.devicePixelRatio) / zoom;
    const h = (ctx.canvas.height / window.devicePixelRatio) / zoom;
    
    // Major Lines
    ctx.strokeStyle = 'rgba(255,255,255,0.12)'; 
    ctx.lineWidth = 1 / zoom;

    const startX = Math.floor(farLeft/gridSize)*gridSize;
    const endX = farLeft+w;
    const startY = Math.floor(farTop/gridSize)*gridSize;
    const endY = farTop+h;

    // Draw Grid Lines
    ctx.beginPath();
    for(let x = startX; x < endX + gridSize; x+=gridSize) {
        if (Math.abs(x) < 0.001) continue; 
        ctx.moveTo(x, farTop); ctx.lineTo(x, farTop+h);
    }
    for(let y = startY; y < endY + gridSize; y+=gridSize) {
        if (Math.abs(y) < 0.001) continue; 
        ctx.moveTo(farLeft, y); ctx.lineTo(farLeft+w, y);
    }
    ctx.stroke();
    
    // Draw Axis (Distinct)
    ctx.strokeStyle = '#94a3b8'; // Lighter Slate
    ctx.lineWidth = 2 / zoom; 
    ctx.beginPath(); 
    ctx.moveTo(farLeft, 0); ctx.lineTo(farLeft+w, 0); 
    ctx.moveTo(0, farTop); ctx.lineTo(0, farTop+h); 
    ctx.stroke(); 

    // Text Logic
    ctx.fillStyle = 'rgba(255,255,255,0.6)'; 
    const fontSize = 10 / zoom;
    ctx.font = `${fontSize}px monospace`;

    for(let x = startX; x < endX + gridSize; x+=gridSize) {
        if (Math.abs(x) < 0.001) continue;
        // Draw coordinate text near grid line
        if (x > farLeft && x < farLeft + w) {
             const val = Math.round(x * 100) / 100;
             let drawY = farTop + h - 20/zoom;
             if (0 > farTop && 0 < farTop + h) drawY = 0 + 12/zoom; 
             ctx.fillText(val.toString(), x + 2/zoom, drawY);
        }
    }
    for(let y = startY; y < endY + gridSize; y+=gridSize) {
        if (Math.abs(y) < 0.001) continue;
        if (y > farTop && y < farTop + h) {
             const val = Math.round(-y * 100) / 100;
             let drawX = farLeft + 5/zoom;
             if (0 > farLeft && 0 < farLeft + w) drawX = 0 + 5/zoom;
             ctx.fillText(val.toString(), drawX, y - 2/zoom); 
        }
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

    // Knife Mode
    if (dragMode === 'tool_knife') {
        setMousePos(worldPos);
        return; // Don't trigger other selections
    }

    // Dynamic Hit Radius
    const HIT_RADIUS_SCREEN = 15;
    const hitRadiusWorld = HIT_RADIUS_SCREEN / state.camera.zoom;

    // --- 1. Vector Handles (Priority) ---
    if (!dragMode.startsWith('add_') && !dragMode.startsWith('tool_')) {
        for (const body of state.bodies) {
             const checkHandle = (vec: Vector2, type: 'velocity' | 'force') => {
                 const tip = Vec2.add(body.position, vec); 
                 if (Vec2.dist(worldPos, tip) < hitRadiusWorld) { 
                     setDraggedVector({ bodyId: body.id, type });
                     onSelectBody(body.id);
                     onSelectConstraint(null);
                     onSelectField(null);
                     onPauseToggle(true); 
                     return true;
                 }
                 return false;
             };
             if (Vec2.magSq(body.constantForce) > 0 && checkHandle(body.constantForce, 'force')) return;
             if (body.showVelocity && Vec2.magSq(body.velocity) > 0 && checkHandle(body.velocity, 'velocity')) return;
        }
    }

    // --- 2. Modes ---
    if (dragMode.startsWith('add_field_')) {
        const shape = dragMode === 'add_field_circle' ? FieldShape.CIRCLE : 
                      dragMode === 'add_field_poly' ? FieldShape.POLYGON : FieldShape.BOX;
        if (dragMode === 'add_field_poly') {
            if (!fieldBuilder) {
                setFieldBuilder({ start: snapped, shape, vertices: [snapped] });
            } else {
                // Check closing
                if (fieldBuilder.vertices && fieldBuilder.vertices.length > 2) {
                    const start = fieldBuilder.vertices[0];
                    if (Vec2.dist(snapped, start) < 20 / state.camera.zoom) {
                        onAddField(fieldBuilder.start, FieldShape.POLYGON, fieldBuilder.vertices);
                        setFieldBuilder(null);
                        return;
                    }
                }
                setFieldBuilder({ ...fieldBuilder, vertices: [...(fieldBuilder.vertices || []), snapped] });
            }
        } else {
            if (!fieldBuilder) setFieldBuilder({ start: snapped, shape });
        }
        return;
    }

    if (dragMode === 'tool_velocity' || dragMode === 'tool_force') {
        let clickedBodyId: string | null = null;
        for(let i=state.bodies.length-1; i>=0; i--) {
            const b = state.bodies[i];
            const r = b.isParticle ? 10/state.camera.zoom : (b.radius || 20);
            if (Vec2.dist(worldPos, b.position) < r + 10) { clickedBodyId = b.id; break; }
        }
        if (clickedBodyId) {
            onPauseToggle(true); onSelectBody(clickedBodyId); setVectorBuilder({ bodyId: clickedBodyId, startPos: worldPos }); 
        }
        return;
    }

    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin', 'add_arc'].includes(dragMode)) {
        onAddBody(snapped);
        return;
    }
    
    if (dragMode === 'add_arc') {
        onAddBody(snapped);
        return;
    }

    // --- 3. Robust Selection Hit Testing (Prioritize selection if mode is 'select') ---
    
    // A. Bodies
    let clickedBodyId: string | null = null;
    for(let i=state.bodies.length-1; i>=0; i--) {
        const b = state.bodies[i];
        let hit = false;
        // Increase hit radius for particles and thin objects
        const tol = hitRadiusWorld * 1.5; 
        
        if (b.type === BodyType.CIRCLE) {
             const r = b.isParticle ? 10/state.camera.zoom : (b.radius || 20);
             hit = Vec2.dist(worldPos, b.position) < Math.max(r, tol);
        } else if (b.type === BodyType.ARC) {
             const r = b.radius || 100;
             const dist = Vec2.dist(worldPos, b.position);
             hit = Math.abs(dist - r) < tol * 2; 
        } else if (b.type === BodyType.BOX) {
             hit = Vec2.pointInRotatedBox(worldPos, b.position, b.width||40, b.height||40, b.angle);
        } else if (b.type === BodyType.POLYGON && b.vertices) {
             hit = Vec2.dist(worldPos, b.position) < 40; // Approx
        }
        if (hit) { clickedBodyId = b.id; break; }
    }

    if (clickedBodyId) {
        if (['add_spring', 'add_rod', 'add_pin'].includes(dragMode)) { onAddBody(worldPos); return; }
        // Select
        onSelectBody(clickedBodyId);
        setDraggedBodyId(clickedBodyId);
        return;
    }

    // B. Constraints
    let clickedConstraintId: string | null = null;
    for (const c of state.constraints) {
        const bA = state.bodies.find(b => b.id === c.bodyAId);
        const bB = state.bodies.find(b => b.id === c.bodyBId);
        if (bA && bB) {
            const pA = Vec2.transform(c.localAnchorA, bA.position, bA.angle);
            const pB = Vec2.transform(c.localAnchorB, bB.position, bB.angle);
            if (Vec2.distToSegment(worldPos, pA, pB) < hitRadiusWorld * 2) {
                clickedConstraintId = c.id;
                break;
            }
        }
    }

    if (clickedConstraintId) {
        onSelectConstraint(clickedConstraintId);
        return;
    }

    // C. Fields
    let clickedFieldId: string | null = null;
    for(let i=state.fields.length-1; i>=0; i--) {
       const f = state.fields[i];
       if (f.shape === FieldShape.BOX) {
           if (worldPos.x >= f.position.x && worldPos.x <= f.position.x + f.size.x &&
               worldPos.y >= f.position.y && worldPos.y <= f.position.y + f.size.y) {
                   clickedFieldId = f.id; break;
           }
       } else if (f.shape === FieldShape.CIRCLE) {
           if (Vec2.dist(worldPos, f.position) < (f.radius || 100)) {
               clickedFieldId = f.id; break;
           }
       } else if (f.shape === FieldShape.POLYGON && f.vertices) {
           // Simple AABB check for now for selection
           // Proper point in poly is better but expensive without caching
           if (Vec2.dist(worldPos, f.vertices[0]) < 100) { // lazy fallback
                clickedFieldId = f.id; break;
           }
       }
    }

    if (clickedFieldId) {
         onSelectField(clickedFieldId);
         return;
    }

    // Deselect if clicking empty space and not panning
    if (dragMode === 'select' && !clickedBodyId && !clickedFieldId && !clickedConstraintId) {
        // We will start panning, but also clear selection? 
        // Typically physics apps keep selection unless explicit deselect.
        // Let's keep selection to avoid annoyance, unless explicitly clicking away?
        // Actually, clicking background usually deselects.
        onSelectBody(null);
    }

    // Nothing clicked -> Pan
    if (!dragMode.startsWith('add_')) {
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
      if (rampCreation) {
          const start = rampCreation.start; 
          const delta = Vec2.sub(worldPos, start); 
          const dist = Vec2.mag(delta);
          
          if (dist > 5) {
              const angleRad = Math.atan2(delta.y, delta.x); 
              const angleDeg = angleRad * 180 / Math.PI;
              const snapAngles = [0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -45, -60, -90, -120, -135, -150];
              let closest = angleDeg; 
              let minDiff = 5; 
              for (const a of snapAngles) { 
                  let diff = Math.abs(a - angleDeg); 
                  if (diff > 180) diff = 360 - diff; 
                  if (diff < minDiff) { minDiff = diff; closest = a; } 
              }
              const snapRad = closest * Math.PI / 180;
              const gridSize = getGridStep(state.camera.zoom);
              const snapDist = Math.round(dist/gridSize)*gridSize;
              const snappedEnd = { x: start.x + Math.cos(snapRad) * snapDist, y: start.y + Math.sin(snapRad) * snapDist };
              setSnappedMousePos(snappedEnd);
          } else { 
              setSnappedMousePos(snapToGrid(worldPos)); 
          }
      } else {
          setSnappedMousePos(snapToGrid(worldPos));
      }
      
      if (draggedVector) {
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              const rawVec = Vec2.sub(worldPos, body.position); 
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
      if (vectorBuilder) return;

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
      if (isDraggingCam) setIsDraggingCam(false);
      
      if (draggedBodyId) {
          setDraggedBodyId(null);
      }
      
      if (draggedVector) {
          setDraggedVector(null);
      }
      
      if (dragMode === 'tool_knife' && knifeBuilder) {
          // Finish cut
          const rect = canvasRef.current!.getBoundingClientRect();
          const sx = e.clientX - rect.left;
          const sy = e.clientY - rect.top;
          const worldPos = screenToWorld(sx, sy);
          onKnifeCut(knifeBuilder.start, worldPos);
          return; // Let App handle clearing builder state
      }
      
      if (vectorBuilder) {
          const body = state.bodies.find(b => b.id === vectorBuilder.bodyId);
          if (body) {
              const rawVec = Vec2.sub(snappedMousePos, body.position);
              onVectorEdit(vectorBuilder.bodyId, dragMode === 'tool_velocity' ? 'velocity' : 'force', rawVec);
          }
          setVectorBuilder(null);
      }

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
               if (r > 5) onAddField(fieldBuilder.start, FieldShape.CIRCLE, [{x: r, y: 0}]);
           }
           setFieldBuilder(null);
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
