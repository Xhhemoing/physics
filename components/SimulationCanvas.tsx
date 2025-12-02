
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
  const [dragStartWorld, setDragStartWorld] = useState<Vector2>({ x: 0, y: 0 }); // World coordinates for body logic
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
    const logZoom = Math.log10(zoom);
    const order = Math.floor(logZoom);
    // 1, 10, 100 based on zoom
    return 100 / Math.pow(10, order);
  };

  const snapToGrid = (v: Vector2): Vector2 => {
      // Calculate dynamic grid size based on zoom
      const gridSize = getGridStep(state.camera.zoom);
      
      // Strict snapping to the visible grid
      const snappedX = Math.round(v.x / gridSize) * gridSize;
      const snappedY = Math.round(v.y / gridSize) * gridSize;

      const SNAP_THRESHOLD_SCREEN = 15;
      const SNAP_THRESHOLD = SNAP_THRESHOLD_SCREEN / state.camera.zoom;
      
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
      drawGrid(ctx);

      // Fields (Background)
      state.fields.forEach(field => {
        if (!field.visible) return;
        
        ctx.save();
        const alpha = field.id === state.selectedFieldId ? 0.3 : 0.15;
        const strokeColor = field.type === FieldType.UNIFORM_ELECTRIC ? '#ef4444' 
                        : field.type === FieldType.UNIFORM_MAGNETIC ? '#3b82f6' 
                        : field.type === FieldType.CUSTOM ? '#d946ef'
                        : '#10b981';
        
        ctx.fillStyle = field.type === FieldType.UNIFORM_ELECTRIC ? `rgba(239, 68, 68, ${alpha})` 
                      : field.type === FieldType.UNIFORM_MAGNETIC ? `rgba(59, 130, 246, ${alpha})`
                      : field.type === FieldType.CUSTOM ? `rgba(217, 70, 239, ${alpha})`
                      : `rgba(16, 185, 129, ${alpha})`;
        
        ctx.strokeStyle = strokeColor;
        ctx.lineWidth = field.id === state.selectedFieldId ? 2 : 1;
        ctx.setLineDash([5, 5]);

        if (field.shape === FieldShape.BOX) {
            ctx.fillRect(field.position.x, field.position.y, field.size.x, field.size.y);
            ctx.strokeRect(field.position.x, field.position.y, field.size.x, field.size.y);
            // Label
            ctx.fillStyle = ctx.strokeStyle;
            ctx.font = '12px sans-serif';
            ctx.fillText(getFieldLabel(field.type), field.position.x + 5, field.position.y + 15);
        } else if (field.shape === FieldShape.CIRCLE) {
            ctx.beginPath();
            ctx.arc(field.position.x, field.position.y, field.radius || 100, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();
            ctx.fillStyle = ctx.strokeStyle;
            ctx.font = '12px sans-serif';
            ctx.fillText(getFieldLabel(field.type), field.position.x - 20, field.position.y);
        } else if (field.shape === FieldShape.POLYGON && field.vertices) {
            ctx.beginPath();
            ctx.moveTo(field.vertices[0].x, field.vertices[0].y);
            for(let i=1; i<field.vertices.length; i++) ctx.lineTo(field.vertices[i].x, field.vertices[i].y);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            ctx.fillStyle = ctx.strokeStyle;
            ctx.font = '12px sans-serif';
            ctx.fillText(getFieldLabel(field.type), field.vertices[0].x, field.vertices[0].y);
        }
        
        ctx.restore();
      });

      // Field Builder Preview
      if (fieldBuilder) {
          ctx.save();
          ctx.strokeStyle = '#fff';
          ctx.setLineDash([5, 5]);
          ctx.lineWidth = 1;
          
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
                  ctx.beginPath(); ctx.arc(v.x, v.y, 3, 0, Math.PI*2); ctx.fill();
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
        ctx.lineWidth = body.id === state.selectedBodyId ? 3 : 2;
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
              ctx.strokeStyle = 'rgba(0,0,0,0.3)'; ctx.lineWidth = 2; ctx.stroke();
          }
        } else if (body.type === BodyType.BOX) {
          const w = body.width || 40;
          const h = body.height || 40;
          const isRamp = h < 12 && w > h * 2; 
          if (isRamp && body.mass === 0) {
              ctx.shadowColor = 'transparent'; ctx.beginPath(); ctx.rect(-w/2, -h/2, w, h);
              ctx.fillStyle = body.color; ctx.fill();
              ctx.beginPath(); ctx.moveTo(-w/2, -h/2); ctx.lineTo(w/2, -h/2);
              ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
          } else {
              ctx.fillRect(-w/2, -h/2, w, h); ctx.shadowColor = 'transparent'; ctx.strokeRect(-w/2, -h/2, w, h);
          }
          // Conveyor marks
           if (body.surfaceSpeed && body.surfaceSpeed !== 0) {
             const dir = Math.sign(body.surfaceSpeed);
             ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth = 2;
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
          ctx.strokeStyle = body.color; ctx.lineWidth = 2; ctx.stroke();
          for(let i=-2000; i<2000; i+=25) {
              ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i - 10, 15);
              ctx.strokeStyle = 'rgba(255,255,255,0.1)'; ctx.lineWidth = 1; ctx.stroke();
          }
        } else if (body.type === BodyType.ARC) {
             const r = body.radius || 100;
             const start = body.arcStartAngle || 0;
             const end = body.arcEndAngle || Math.PI;
             ctx.shadowColor = 'transparent'; ctx.beginPath(); ctx.arc(0, 0, r, start, end);
             ctx.strokeStyle = body.color; ctx.lineWidth = 5; ctx.stroke();
             ctx.strokeStyle = 'rgba(255,255,255,0.2)'; ctx.lineWidth = 1;
             const steps = 20;
             for (let i=0; i<=steps; i++) {
                 const a = start + (end-start)*(i/steps);
                 ctx.beginPath(); ctx.moveTo(Math.cos(a)*r, Math.sin(a)*r); ctx.lineTo(Math.cos(a)*(r+10), Math.sin(a)*(r+10)); ctx.stroke();
             }
        }
        
        if (body.showCharge && body.charge !== 0) {
             const offset = body.isParticle ? -15 : 0;
             ctx.fillStyle = body.charge > 0 ? 'rgba(239, 68, 68, 0.8)' : 'rgba(59, 130, 246, 0.8)';
             ctx.font = 'bold 20px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
             ctx.fillText(body.charge > 0 ? '+' : '-', 0, offset);
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
        if (Vec2.magSq(body.constantForce) > 0 || draggedVector?.type === 'force') {
            drawInteractiveVector(ctx, body.constantForce, '#ef4444', 'F_app', 1.0, true);
        }
        // ONLY show velocity handle if explicitly enabled or currently dragging it
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
              ctx.strokeStyle = body.color; ctx.globalAlpha = 0.5; ctx.lineWidth = 1; ctx.stroke();
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
            ctx.lineWidth = c.type === ConstraintType.SPRING ? 3 : 5;
            ctx.strokeStyle = isSelected ? '#fff' : (c.type === ConstraintType.SPRING ? '#fbbf24' : '#94a3b8'); 
            if (c.type === ConstraintType.SPRING) drawSpring(ctx, pA, pB, 6);
            else if (c.type === ConstraintType.ROD) { ctx.stroke(); ctx.fillStyle = '#94a3b8'; ctx.beginPath(); ctx.arc(pA.x, pA.y, 4, 0, Math.PI*2); ctx.fill(); ctx.beginPath(); ctx.arc(pB.x, pB.y, 4, 0, Math.PI*2); ctx.fill(); }
            else if (c.type === ConstraintType.PIN) { ctx.fillStyle = '#f59e0b'; ctx.beginPath(); ctx.arc(pA.x, pA.y, 6, 0, Math.PI*2); ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke(); }
        }
      });

      // Constraint Builder Preview
      if (constraintBuilder) {
          const bodyA = state.bodies.find(b => b.id === constraintBuilder);
          if (bodyA) {
              ctx.save(); ctx.beginPath(); ctx.moveTo(bodyA.position.x, bodyA.position.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
              ctx.strokeStyle = '#fbbf24'; ctx.lineWidth = 2; ctx.setLineDash([5, 5]); ctx.stroke();
              ctx.fillStyle = '#fbbf24'; ctx.beginPath(); ctx.arc(snappedMousePos.x, snappedMousePos.y, 4, 0, Math.PI*2); ctx.fill(); ctx.restore();
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
              ctx.lineWidth = 3;
              ctx.setLineDash([4, 4]);
              ctx.stroke();
              ctx.setLineDash([]);
              
              const angle = Math.atan2(targetPos.y, targetPos.x);
              const headLen = 10;
              ctx.beginPath();
              ctx.moveTo(targetPos.x, targetPos.y);
              ctx.lineTo(targetPos.x - headLen * Math.cos(angle - Math.PI / 6), targetPos.y - headLen * Math.sin(angle - Math.PI / 6));
              ctx.lineTo(targetPos.x - headLen * Math.cos(angle + Math.PI / 6), targetPos.y - headLen * Math.sin(angle + Math.PI / 6));
              ctx.lineTo(targetPos.x, targetPos.y);
              ctx.fillStyle = isForce ? '#ef4444' : '#3b82f6';
              ctx.fill();

              const mag = Vec2.mag(targetPos); 
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
                  
              ctx.beginPath();
              // Always draw full circle hint
              ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI * 2);
              ctx.stroke();
              
              ctx.fillStyle = '#38bdf8';
              ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, 4, 0, Math.PI*2); ctx.fill();

              if (arcCreation.phase === 2) {
                   // Drawing the Arc segment from 0 to mouse angle
                   const dx = snappedMousePos.x - arcCreation.center.x;
                   const dy = snappedMousePos.y - arcCreation.center.y;
                   const mouseAngle = Math.atan2(dy, dx);
                   ctx.fillStyle = 'rgba(56, 189, 248, 0.2)';
                   ctx.beginPath(); ctx.moveTo(arcCreation.center.x, arcCreation.center.y);
                   // Default start is 0
                   ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, mouseAngle);
                   ctx.lineTo(arcCreation.center.x, arcCreation.center.y); ctx.fill();
              }
          }
          ctx.restore();
      }

      // Ramp Overlay
      if (rampCreation) {
          ctx.save();
          ctx.beginPath();
          ctx.strokeStyle = 'rgba(255,255,255,0.2)';
          ctx.lineWidth = 1;
          ctx.setLineDash([2, 4]);
          ctx.moveTo(rampCreation.start.x - 500, rampCreation.start.y); ctx.lineTo(rampCreation.start.x + 500, rampCreation.start.y);
          ctx.moveTo(rampCreation.start.x, rampCreation.start.y - 500); ctx.lineTo(rampCreation.start.x, rampCreation.start.y + 500);
          ctx.stroke();

          ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 2; ctx.setLineDash([5, 5]);
          ctx.beginPath(); ctx.moveTo(rampCreation.start.x, rampCreation.start.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y); ctx.stroke();
          ctx.fillStyle = '#38bdf8'; ctx.beginPath(); ctx.arc(rampCreation.start.x, rampCreation.start.y, 4, 0, Math.PI*2); ctx.fill(); ctx.beginPath(); ctx.arc(snappedMousePos.x, snappedMousePos.y, 4, 0, Math.PI*2); ctx.fill();
          
          const delta = Vec2.sub(snappedMousePos, rampCreation.start);
          const angleRad = Math.atan2(delta.y, delta.x);
          const angleDeg = (angleRad * 180 / Math.PI).toFixed(1);
          ctx.font = '12px monospace'; ctx.fillStyle = '#fff';
          ctx.fillText(`${angleDeg}°`, snappedMousePos.x + 10, snappedMousePos.y - 10);
          ctx.restore();
      }
      
      if (dragMode.startsWith('add_') || draggedBodyId) {
          ctx.save(); ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1;
          ctx.beginPath(); ctx.moveTo(snappedMousePos.x - 5, snappedMousePos.y); ctx.lineTo(snappedMousePos.x + 5, snappedMousePos.y);
          ctx.moveTo(snappedMousePos.x, snappedMousePos.y - 5); ctx.lineTo(snappedMousePos.x, snappedMousePos.y + 5);
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
      if (len < 3) return; 

      ctx.save();
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(visualVec.x, visualVec.y);
      ctx.strokeStyle = color;
      ctx.lineWidth = dashed ? 1.5 : 2;
      if (dashed) ctx.setLineDash([4, 2]);
      ctx.stroke();
      ctx.setLineDash([]);

      const angle = Math.atan2(visualVec.y, visualVec.x);
      const headLen = 8;
      ctx.beginPath();
      ctx.moveTo(visualVec.x, visualVec.y);
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle - Math.PI / 6), visualVec.y - headLen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle + Math.PI / 6), visualVec.y - headLen * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();

      ctx.fillStyle = color;
      ctx.font = '10px monospace';
      ctx.fillText(label, visualVec.x + 5, visualVec.y + 5);
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
          ctx.lineWidth = 2;
          ctx.stroke();

          // Arrow Head
          const angle = Math.atan2(py, px);
          const headLen = 10;
          ctx.beginPath();
          ctx.moveTo(px, py);
          ctx.lineTo(px - headLen * Math.cos(angle - Math.PI / 6), py - headLen * Math.sin(angle - Math.PI / 6));
          ctx.lineTo(px - headLen * Math.cos(angle + Math.PI / 6), py - headLen * Math.sin(angle + Math.PI / 6));
          ctx.closePath();
          ctx.fillStyle = color;
          ctx.fill();
      }

      // Interaction Handle (Circle at tip)
      // FIX: Make it visually consistent but not too large
      if (isInteractive) {
          ctx.beginPath();
          // Constant screen size ~3.5px
          const handleR = 3.5 / state.camera.zoom; 
          ctx.arc(px, py, handleR, 0, Math.PI * 2); 
          ctx.fillStyle = color;
          ctx.fill();
          ctx.strokeStyle = '#fff';
          ctx.lineWidth = 1.5 / state.camera.zoom;
          ctx.stroke();
      }
      
      if (Math.abs(px) > 0.1 || Math.abs(py) > 0.1) {
          ctx.fillStyle = color;
          ctx.font = '10px monospace';
          const mag = Vec2.mag(vec);
          ctx.fillText(`${label}: ${mag.toFixed(1)}`, px + 10, py + 5);
      }

      ctx.restore();
  };

  const drawGrid = (ctx: CanvasRenderingContext2D) => {
    const zoom = state.camera.zoom;
    const gridSize = getGridStep(zoom);
    
    const farLeft = ((-state.camera.x / zoom) - 2000);
    const farTop = ((-state.camera.y / zoom) - 2000);
    const w = 6000 / zoom;
    const h = 6000 / zoom;
    
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'; 
    ctx.lineWidth = 1 / zoom;

    const startX = Math.round(farLeft/gridSize)*gridSize;
    const endX = farLeft+w;
    const startY = Math.round(farTop/gridSize)*gridSize;
    const endY = farTop+h;

    // Draw Grid Lines
    ctx.beginPath();
    for(let x = startX; x < endX; x+=gridSize) {
        if (Math.abs(x) < 0.1) continue; // Skip axis
        ctx.moveTo(x, farTop); ctx.lineTo(x, farTop+h);
    }
    for(let y = startY; y < endY; y+=gridSize) {
        if (Math.abs(y) < 0.1) continue; // Skip axis
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

    // Axis Labels - Draw relative to camera so they stay on screen if desired, 
    // or draw at axis lines. Drawing at axis lines is better for physics context.
    
    // Font scale fix to keep text readable
    ctx.font = '12px monospace';
    ctx.fillStyle = '#94a3b8'; 
    
    // Draw coordinates near the lines
    ctx.fillStyle = 'rgba(255,255,255,0.3)'; 
    
    // Determine screen text size
    const fontSize = 10 / zoom;
    ctx.font = `${fontSize}px monospace`;

    // Only draw every Nth label to avoid clutter if zoomed out fast
    for(let x = startX; x < endX; x+=gridSize) {
        if (Math.abs(x) < 0.1) continue;
        // Check if visible
        if (x > (-state.camera.x/zoom) && x < ((-state.camera.x + ctx.canvas.width/window.devicePixelRatio)/zoom)) {
             ctx.fillText(parseFloat(x.toFixed(2)).toString(), x + 2/zoom, 12/zoom);
        }
    }
    for(let y = startY; y < endY; y+=gridSize) {
        if (Math.abs(y) < 0.1) continue;
        if (y > (-state.camera.y/zoom) && y < ((-state.camera.y + ctx.canvas.height/window.devicePixelRatio)/zoom)) {
             ctx.fillText(parseFloat((-y).toFixed(2)).toString(), 5/zoom, y - 2/zoom); 
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
    setDragStartWorld(worldPos);
    setHasMoved(false);

    const HIT_RADIUS_SCREEN = 8;
    const hitRadiusWorld = HIT_RADIUS_SCREEN / state.camera.zoom;

    // --- 1. Vector Handles (Priority) ---
    if (!dragMode.startsWith('add_') && !dragMode.startsWith('tool_')) {
        for (const body of state.bodies) {
             const checkHandle = (vec: Vector2, type: 'velocity' | 'force') => {
                 const tip = Vec2.add(body.position, vec); 
                 if (Vec2.dist(worldPos, tip) < hitRadiusWorld * 2) {
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
             // STRICT: Only check velocity handle if showVelocity is true or velocity is being dragged
             if (body.showVelocity && Vec2.magSq(body.velocity) > 0 && checkHandle(body.velocity, 'velocity')) return;
        }
    }

    // --- 2. Modes (Field, Tool, Add) ---
    if (dragMode.startsWith('add_field_')) {
        const shape = dragMode === 'add_field_circle' ? FieldShape.CIRCLE : 
                      dragMode === 'add_field_poly' ? FieldShape.POLYGON : FieldShape.BOX;
        if (dragMode === 'add_field_poly') {
            if (!fieldBuilder) setFieldBuilder({ start: snapped, shape, vertices: [snapped] });
            else setFieldBuilder({ ...fieldBuilder, vertices: [...(fieldBuilder.vertices || []), snapped] });
        } else {
            if (!fieldBuilder) setFieldBuilder({ start: snapped, shape });
        }
        return;
    }

    if (dragMode === 'tool_velocity' || dragMode === 'tool_force') {
        // Hit Test for Tool
        let clickedBodyId: string | null = null;
        for(let i=state.bodies.length-1; i>=0; i--) {
            const b = state.bodies[i];
            const dist = Vec2.dist(worldPos, b.position);
            const radius = b.type === BodyType.CIRCLE ? (b.radius || 20) : (Math.max(b.width||40, b.height||40)/2);
            if (dist < radius) { clickedBodyId = b.id; break; }
        }
        if (clickedBodyId) {
            onPauseToggle(true); onSelectBody(clickedBodyId); setVectorBuilder({ bodyId: clickedBodyId, startPos: worldPos }); 
        }
        return;
    }

    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin'].includes(dragMode)) {
        onAddBody(snapped);
        return;
    }

    // --- 3. General Selection Hit Testing ---
    // Reverse iterate to pick top-most
    
    // A. Bodies
    let clickedBodyId: string | null = null;
    for(let i=state.bodies.length-1; i>=0; i--) {
        const b = state.bodies[i];
        
        if (b.type === BodyType.CIRCLE || b.type === BodyType.ARC) {
             const r = b.isParticle ? 5 : (b.radius || 20);
             // Use dist check
             if (Vec2.dist(worldPos, b.position) < Math.max(r, hitRadiusWorld)) { clickedBodyId = b.id; break; }
        } else if (b.type === BodyType.BOX) {
             // Use Rotated Box Check
             if (Vec2.pointInRotatedBox(worldPos, b.position, b.width||40, b.height||40, b.angle)) {
                 clickedBodyId = b.id; break;
             }
        } else if (b.type === BodyType.POLYGON && b.vertices) {
             // Simple AABB approximation for now or Point in Poly
             // ... keeping it simple for polygon (centroid check)
             if (Vec2.dist(worldPos, b.position) < 30) { clickedBodyId = b.id; break; }
        }
    }

    if (clickedBodyId) {
        if (['add_spring', 'add_rod', 'add_pin'].includes(dragMode)) { onAddBody(worldPos); return; }
        onSelectBody(clickedBodyId);
        onSelectField(null);
        onSelectConstraint(null);
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
            const dist = Vec2.distToSegment(worldPos, pA, pB);
            if (dist < hitRadiusWorld) {
                clickedConstraintId = c.id;
                break;
            }
        }
    }

    if (clickedConstraintId) {
        onSelectConstraint(clickedConstraintId);
        onSelectBody(null);
        onSelectField(null);
        return;
    }

    // C. Fields
    let clickedFieldId: string | null = null;
    for(const f of state.fields) {
       if (f.shape === FieldShape.BOX) {
           if (worldPos.x >= f.position.x && worldPos.x <= f.position.x + f.size.x &&
               worldPos.y >= f.position.y && worldPos.y <= f.position.y + f.size.y) {
                   clickedFieldId = f.id; break;
           }
       } else if (f.shape === FieldShape.CIRCLE) {
           if (Vec2.dist(worldPos, f.position) < (f.radius || 100)) {
               clickedFieldId = f.id; break;
           }
       }
    }

    if (clickedFieldId) {
         onSelectField(clickedFieldId);
         onSelectBody(null);
         onSelectConstraint(null);
         return;
    }

    // Nothing clicked -> Pan or Deselect
    if (!dragMode.startsWith('add_')) {
        setIsDraggingCam(true);
        // Don't deselect immediately on mouse down to allow dragging pan
        // Deselect happens on mouse up if no move
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
          // Snap Ramp end point
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
              
              // Also snap distance to grid if close
              let snapDist = dist;
              // Simple dist snap check
              const gridSize = getGridStep(state.camera.zoom);
              if (Math.abs(dist % gridSize) < gridSize * 0.2) snapDist = Math.round(dist/gridSize)*gridSize;
              
              const snappedEnd = { x: start.x + Math.cos(snapRad) * snapDist, y: start.y + Math.sin(snapRad) * snapDist };
              
              // IMPORTANT: Update snappedMousePos to the calculated ramp end
              setSnappedMousePos(snappedEnd);
          } else { 
              setSnappedMousePos(snapToGrid(worldPos)); 
          }
      } else {
          setSnappedMousePos(snapToGrid(worldPos));
      }
      
      // ... (Vector Dragging logic same as before)
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
      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      
      if (isDraggingCam) setIsDraggingCam(false);
      
      if (draggedBodyId) {
          if (!hasMoved) {
               onSelectBody(draggedBodyId);
               onSelectField(null);
               onSelectConstraint(null);
          }
          setDraggedBodyId(null);
      }
      
      if (draggedVector) {
          // cleanup zero vectors
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              const vec = draggedVector.type === 'velocity' ? body.velocity : body.constantForce;
              if (Vec2.mag(vec) * state.camera.zoom < 10) { 
                  onVectorEdit(draggedVector.bodyId, draggedVector.type, {x:0, y:0});
              }
          }
          setDraggedVector(null);
      }
      
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
      } else if (!hasMoved && !draggedBodyId && !isDraggingCam && !draggedVector) {
          // Deselect if clicked empty space (checked hit tests in MouseDown)
          // We can just rely on MouseDown to set selection. 
          // If MouseDown didn't find anything, it set draggingCam=true.
          // If we are here and not moved, we can clear.
          // However, MouseDown hit logic handles selection now.
          // To support "click empty space deselects", we need to check if MouseDown hit anything.
          // Since we don't store "didHit" state, let's re-run a simple check or just clear if nothing selected.
          // Actually, MouseDown logic handles selection. If we clicked something, it's selected.
          // If we are here, we might have clicked nothing.
          
          // Re-implement deselect logic cleanly:
          // If we clicked a valid object, it was handled in MouseDown.
          // If we started dragging cam (clicked empty space) and didn't move, it's a click on empty space.
          if (!dragMode.startsWith('add_') && !constraintBuilder) {
              onSelectBody(null);
              onSelectField(null);
              onSelectConstraint(null);
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
