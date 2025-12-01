
import React, { useRef, useEffect, useState, useImperativeHandle, forwardRef } from 'react';
import { BodyType, PhysicsBody, SimulationState, FieldType, Vector2, ConstraintType, FieldShape } from '../types';
import { Vec2 } from '../services/vectorMath';

interface Props {
  state: SimulationState;
  onSelectBody: (id: string | null) => void;
  onSelectField: (id: string | null) => void;
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

const SimulationCanvas = forwardRef<CanvasRef, Props>(({ state, onSelectBody, onSelectField, onAddBody, onAddField, onMoveBody, onVectorEdit, dragMode, onZoom, onPauseToggle, arcCreation, rampCreation, constraintBuilder }, ref) => {
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

  const snapToGrid = (v: Vector2): Vector2 => {
      // Calculate dynamic grid size based on zoom
      const baseGrid = 20;
      // Adjust grid snapping logic if needed, for now constant grid is okay or we can make it dynamic
      // Making it dynamic might confuse placement if grid lines shift. 
      // User asked for "grid size varies with view", let's keep snapping constant for consistency or match visual grid?
      // Matching visual grid is better.
      const logZoom = Math.log10(state.camera.zoom);
      const scale = Math.pow(10, Math.floor(logZoom));
      const gridSize = 20 / scale; // Simple adaptive snap

      const SNAP_THRESHOLD = 8 / state.camera.zoom; // Screen pixels converted to world
      
      // Use a fixed reasonable grid for object placement stability (e.g., 20)
      const STABLE_GRID = 20;
      const snappedX = Math.round(v.x / STABLE_GRID) * STABLE_GRID;
      const snappedY = Math.round(v.y / STABLE_GRID) * STABLE_GRID;
      
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
        ctx.fillStyle = field.type === FieldType.UNIFORM_ELECTRIC ? `rgba(239, 68, 68, ${alpha})` 
                      : field.type === FieldType.UNIFORM_MAGNETIC ? `rgba(59, 130, 246, ${alpha})`
                      : field.type === FieldType.CUSTOM ? `rgba(217, 70, 239, ${alpha})`
                      : `rgba(16, 185, 129, ${alpha})`;
        ctx.strokeStyle = field.type === FieldType.UNIFORM_ELECTRIC ? '#ef4444' 
                        : field.type === FieldType.UNIFORM_MAGNETIC ? '#3b82f6' 
                        : field.type === FieldType.CUSTOM ? '#d946ef'
                        : '#10b981';
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
        // ... (Body rendering code remains mostly same, just ensuring correct order)
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
                Object.entries(body.forceComponents).forEach(([name, vec]) => {
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
        // These are persistent if set, or if we are editing
        if (Vec2.magSq(body.constantForce) > 0) {
            drawInteractiveVector(ctx, body.constantForce, '#ef4444', 'F_app', 10.0, true);
        }
        if (body.selected && (Vec2.magSq(body.velocity) > 0 || draggedVector?.type === 'velocity')) {
            drawInteractiveVector(ctx, body.velocity, '#3b82f6', 'v0', 1.0, true);
        }

        ctx.restore();
      });
      
       // Trails (Rendered separately to be behind bodies if needed, but here is fine)
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
            ctx.lineWidth = c.type === ConstraintType.SPRING ? 3 : 5;
            ctx.strokeStyle = c.type === ConstraintType.SPRING ? '#fbbf24' : '#94a3b8'; 
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

      // Vector Builder Visualization (Ramp-like Logic)
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

              const mag = isForce ? Vec2.mag(targetPos) * 10 : Vec2.mag(targetPos); 
              ctx.fillStyle = '#fff';
              ctx.font = '12px monospace';
              const label = isForce ? `F: ${mag.toFixed(0)} N` : `v: ${mag.toFixed(1)} m/s`;
              ctx.fillText(label, targetPos.x + 10, targetPos.y);

              ctx.restore();
          }
      }

      // Arc Creation Overlay
      if (arcCreation) {
          // ... (Arc creation overlay code)
          ctx.save();
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2;
          ctx.setLineDash([5, 5]);

          if (arcCreation.phase >= 1) {
              const currentRadius = arcCreation.phase === 1 
                  ? Vec2.dist(arcCreation.center, snappedMousePos) 
                  : arcCreation.radius;
                  
              ctx.beginPath();
              ctx.moveTo(arcCreation.center.x, arcCreation.center.y);
              ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, Math.PI * 2);
              ctx.stroke();
              
              ctx.fillStyle = '#38bdf8';
              ctx.beginPath(); ctx.arc(arcCreation.center.x, arcCreation.center.y, 4, 0, Math.PI*2); ctx.fill();

              if (arcCreation.phase === 2) {
                   const dx = snappedMousePos.x - arcCreation.center.x;
                   const dy = snappedMousePos.y - arcCreation.center.y;
                   const mouseAngle = Math.atan2(dy, dx);
                   ctx.fillStyle = 'rgba(56, 189, 248, 0.2)';
                   ctx.beginPath(); ctx.moveTo(arcCreation.center.x, arcCreation.center.y);
                   ctx.arc(arcCreation.center.x, arcCreation.center.y, currentRadius, 0, mouseAngle);
                   ctx.lineTo(arcCreation.center.x, arcCreation.center.y); ctx.fill();
              }
          }
          ctx.restore();
      }

      // Ramp Overlay
      if (rampCreation) {
           // ... (Ramp creation overlay code)
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
      // Force scaling can be huge, so visual scaling for force is needed. 
      // Velocity is usually 1:1, Force might be 1:10 depending on units. 
      // Here we assume vec passed in is RAW. We apply scale for display.
      // NOTE: scale param here acts as a display multiplier. 
      
      // Special check: If this vector is currently being dragged, use the mouse position to draw it dynamically?
      // No, state updates trigger re-render.
      
      const visualVec = Vec2.mul(vec, scale === 10.0 ? 1 : 1); // Logic for force scaling is handled by caller usually or implicit
      // Actually, 'tool_force' logic uses 10x multiplier. Let's stick to standard visualization.
      
      const px = vec.x;
      const py = vec.y;
      
      if (Math.abs(px) < 0.1 && Math.abs(py) < 0.1) return;

      ctx.save();
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

      // Interaction Handle
      if (isInteractive) {
          ctx.beginPath();
          ctx.arc(px, py, 5 / state.camera.zoom + 2, 0, Math.PI * 2); // Handle size scales slightly with zoom to remain clickable
          ctx.fillStyle = color;
          ctx.fill();
          ctx.strokeStyle = '#fff';
          ctx.lineWidth = 1;
          ctx.stroke();
      }
      
      ctx.fillStyle = color;
      ctx.font = '10px monospace';
      const mag = Vec2.mag(vec);
      ctx.fillText(`${label}: ${mag.toFixed(1)}`, px + 5, py + 5);

      ctx.restore();
  };

  const drawGrid = (ctx: CanvasRenderingContext2D) => {
    // Adaptive Grid
    const zoom = state.camera.zoom;
    // Calculate exponent of zoom base 10
    const logZoom = Math.log10(zoom);
    const order = Math.floor(logZoom);
    // Grid size shrinks/grows by powers of 10 relative to zoom
    // Base grid 100 at zoom 1.
    // At zoom 10, grid should be 10. At zoom 0.1, grid should be 1000.
    const baseGrid = 100;
    const gridSize = baseGrid / Math.pow(10, order);
    
    // Fade factor for smooth transition (optional, but nice)
    // const fade = logZoom - order; 
    
    const farLeft = ((-state.camera.x / zoom) - 2000);
    const farTop = ((-state.camera.y / zoom) - 2000);
    const w = 6000 / zoom;
    const h = 6000 / zoom;
    
    // Subgrid
    ctx.strokeStyle = 'rgba(255,255,255,0.03)';
    ctx.lineWidth = 1;
    // ... skipping subgrid for performance/simplicity if adaptive main grid works well

    // Main Grid
    ctx.strokeStyle = 'rgba(255,255,255,0.1)'; 
    ctx.lineWidth = 1;

    const startX = Math.floor(farLeft/gridSize)*gridSize;
    const endX = farLeft+w;
    const startY = Math.floor(farTop/gridSize)*gridSize;
    const endY = farTop+h;

    for(let x = startX; x < endX; x+=gridSize) {
        ctx.beginPath(); ctx.moveTo(x, farTop); ctx.lineTo(x, farTop+h); ctx.stroke();
        if (zoom > 0.2) {
             ctx.fillStyle = 'rgba(255,255,255,0.3)'; ctx.font = '10px monospace'; 
             ctx.fillText(parseFloat(x.toFixed(2)).toString(), x + 2, farTop + h/2 + 10);
        }
    }
    for(let y = startY; y < endY; y+=gridSize) {
        ctx.beginPath(); ctx.moveTo(farLeft, y); ctx.lineTo(farLeft+w, y); ctx.stroke();
        if (zoom > 0.2) {
            ctx.fillStyle = 'rgba(255,255,255,0.3)'; ctx.font = '10px monospace'; 
            ctx.fillText(parseFloat((-y).toFixed(2)).toString(), farLeft + w/2 + 5, y - 2); 
        }
    }
    
    // Axes
    ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(farLeft, 0); ctx.lineTo(farLeft+w, 0); ctx.stroke(); 
    ctx.beginPath(); ctx.moveTo(0, farTop); ctx.lineTo(0, farTop+h); ctx.stroke(); 
    
    ctx.fillStyle = '#fff'; ctx.font = '12px monospace';
    ctx.fillText("0,0", 5, -5);
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

    // --- 1. Check for Vector Handle Interactions (Priority) ---
    // Check if clicking near a velocity or force tip of SELECTED bodies
    if (!dragMode.startsWith('add_') && !dragMode.startsWith('tool_')) {
        for (const body of state.bodies) {
            if (body.selected || body.showVelocity || Vec2.magSq(body.constantForce) > 0) {
                 const checkHandle = (vec: Vector2, type: 'velocity' | 'force') => {
                     const tip = Vec2.add(body.position, vec); // World pos of tip
                     const dist = Vec2.dist(worldPos, tip);
                     // Hit radius: 10px screen size
                     if (dist * state.camera.zoom < 10) {
                         setDraggedVector({ bodyId: body.id, type });
                         onSelectBody(body.id);
                         onPauseToggle(true); // Pause when editing vectors
                         return true;
                     }
                     return false;
                 };
                 
                 // Check Force (Priority if both exist)
                 if (Vec2.magSq(body.constantForce) > 0) {
                     if (checkHandle(body.constantForce, 'force')) return;
                 }
                 // Check Velocity
                 if (body.selected || body.showVelocity) {
                     if (checkHandle(body.velocity, 'velocity')) return;
                 }
            }
        }
    }

    // --- 2. Field Creation Logic ---
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

    // --- 3. Vector Builder Tool (Click Body -> Drag) ---
    if (dragMode === 'tool_velocity' || dragMode === 'tool_force') {
        let clickedBodyId: string | null = null;
        for(let i=state.bodies.length-1; i>=0; i--) {
            const b = state.bodies[i];
            const hitR = b.type === BodyType.CIRCLE && b.isParticle ? 10 : (b.radius || 20);
            const dist = b.type === BodyType.CIRCLE || b.type === BodyType.ARC ? Vec2.dist(worldPos, b.position) : 9999;
            if (dist < hitR) { clickedBodyId = b.id; break; }
            if (b.type === BodyType.BOX && Math.abs(worldPos.x-b.position.x)<(b.width||40)/2 && Math.abs(worldPos.y-b.position.y)<(b.height||40)/2) { clickedBodyId = b.id; break; }
        }

        if (clickedBodyId) {
            onPauseToggle(true);
            onSelectBody(clickedBodyId);
            setVectorBuilder({ bodyId: clickedBodyId, startPos: worldPos }); 
        }
        return;
    }

    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin', 'add_field_box', 'add_field_circle', 'add_field_poly'].includes(dragMode)) {
        onAddBody(snapped);
        return;
    }

    // --- 4. Body Selection ---
    let clickedBodyId: string | null = null;
    for(let i=state.bodies.length-1; i>=0; i--) {
        const b = state.bodies[i];
        if (b.type === BodyType.CIRCLE) {
            const hitR = b.isParticle ? 10 : (b.radius || 20);
            if (Vec2.dist(worldPos, b.position) < hitR) { clickedBodyId = b.id; break; }
        } else if (b.type === BodyType.BOX || b.type === BodyType.POLYGON) {
            const w = b.width || 40; const h = b.height || 40;
             if (worldPos.x > b.position.x - w/2 && worldPos.x < b.position.x + w/2 &&
                worldPos.y > b.position.y - h/2 && worldPos.y < b.position.y + h/2) { clickedBodyId = b.id; break; }
        } else if (b.type === BodyType.ARC) {
             const dist = Vec2.dist(worldPos, b.position); const r = b.radius || 100;
             if (Math.abs(dist - r) < 10) { clickedBodyId = b.id; break; }
        }
    }

    // --- 5. Field Selection ---
    let clickedFieldId: string | null = null;
    if (!clickedBodyId) {
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
    }

    if (clickedBodyId) {
        if (['add_spring', 'add_rod', 'add_pin'].includes(dragMode)) { onAddBody(worldPos); return; }
        setDraggedBodyId(clickedBodyId);
    } else if (clickedFieldId) {
        // Selection happens in MouseUp to distinguish from pan
        // setDraggedFieldId(clickedFieldId); // If we implement field dragging
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
      if (moveDist > 5) setHasMoved(true);

      // --- Snapping Logic ---
      if (rampCreation) {
          const start = rampCreation.start; const delta = Vec2.sub(worldPos, start); const dist = Vec2.mag(delta);
          if (dist > 5) {
              const angleRad = Math.atan2(delta.y, delta.x); const angleDeg = angleRad * 180 / Math.PI;
              const snapAngles = [0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -45, -60, -90, -120, -135, -150];
              let closest = angleDeg; let minDiff = 3; 
              for (const a of snapAngles) { let diff = Math.abs(a - angleDeg); if (diff > 180) diff = 360 - diff; if (diff < minDiff) { minDiff = diff; closest = a; } }
              const snapRad = closest * Math.PI / 180;
              const snappedEnd = { x: start.x + Math.cos(snapRad) * dist, y: start.y + Math.sin(snapRad) * dist };
              setSnappedMousePos(snappedEnd);
          } else { setSnappedMousePos(worldPos); }
      } else {
          setSnappedMousePos(snapToGrid(worldPos));
      }
      
      // --- Handle Vector Dragging (Existing Vector Handle) ---
      if (draggedVector) {
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              const rawVec = Vec2.sub(worldPos, body.position); // Calculate vector based on mouse
              
              // Apply Snapping (Horizontal/Vertical)
              let snappedVec = rawVec;
              const angleRad = Math.atan2(rawVec.y, rawVec.x);
              const angleDeg = angleRad * 180 / Math.PI;
              const dist = Vec2.mag(rawVec);
              
              // Strong snap to 0, 90, 180, -90 within 5 degrees
              if (Math.abs(angleDeg) < 5 || Math.abs(angleDeg - 180) < 5 || Math.abs(angleDeg + 180) < 5) snappedVec = { x: rawVec.x, y: 0 };
              else if (Math.abs(angleDeg - 90) < 5 || Math.abs(angleDeg + 90) < 5) snappedVec = { x: 0, y: rawVec.y };

              onVectorEdit(draggedVector.bodyId, draggedVector.type, snappedVec);
          }
          return;
      }

      // --- Handle Vector Builder Dragging ---
      if (vectorBuilder) {
          return; // Visual only
      }

      if (draggedBodyId && hasMoved) {
          onMoveBody(draggedBodyId, snappedMousePos); 
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
      if (draggedBodyId) {
          // If simply clicked without moving much, treat as Selection
          if (!hasMoved) {
               onSelectBody(draggedBodyId);
               onSelectField(null);
          }
          setDraggedBodyId(null);
      }
      if (draggedVector) {
          // Check if vector is near zero, if so, delete it
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              const vec = draggedVector.type === 'velocity' ? body.velocity : body.constantForce;
              if (Vec2.magSq(vec) < 4) { // Threshold
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
              
              // Angle Snapping for Tool Builder
              let finalVec = rawVec;
              const angleRad = Math.atan2(rawVec.y, rawVec.x);
              const angleDeg = angleRad * 180 / Math.PI;
               // Strong snap
              if (Math.abs(angleDeg) < 5 || Math.abs(angleDeg - 180) < 5 || Math.abs(angleDeg + 180) < 5) finalVec = { x: rawVec.x, y: 0 };
              else if (Math.abs(angleDeg - 90) < 5 || Math.abs(angleDeg + 90) < 5) finalVec = { x: 0, y: rawVec.y };
              
              if (dragMode === 'tool_force') {
                  finalVec = Vec2.mul(finalVec, 1); // No multiplier, 1:1 map pixel to Newton for simplicity or allow scaling
              }
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
                   const fw = Math.abs(w);
                   const fh = Math.abs(h);
                   onAddField({x, y}, FieldShape.BOX, [{x: fw, y: fh}]);
               }
           } else if (dragMode === 'add_field_circle') {
               const r = Vec2.dist(fieldBuilder.start, snappedMousePos);
               if (r > 5) {
                   onAddField(fieldBuilder.start, FieldShape.CIRCLE, [{x: r, y: 0}]);
               }
           }
           setFieldBuilder(null);
      } else if (!hasMoved && !draggedBodyId && !isDraggingCam && !draggedVector) {
          // Check for Field Selection if not dragging
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
          } else {
               // Deselect all
               onSelectBody(null);
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
