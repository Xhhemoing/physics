
import React, { useRef, useEffect, useState, useImperativeHandle, forwardRef, useCallback } from 'react';
import { BodyType, SimulationState, FieldType, Vector2, FieldShape, ConstraintType, PhysicsField, PhysicsBody } from '../types';
import { Vec2 } from '../services/vectorMath';

interface Props {
  state: SimulationState;
  onSelectBody: (id: string | null) => void;
  onSelectField: (id: string | null) => void;
  onSelectConstraint: (id: string | null) => void;
  onAddBody: (position: Vector2) => void;
  onAddField: (position: Vector2, size: Vector2, shape: FieldShape, vertices?: Vector2[]) => void;
  onMoveBody: (id: string, position: Vector2) => void;
  onVectorEdit: (id: string, vectorType: 'velocity' | 'force', vector: Vector2) => void;
  dragMode: string;
  onZoom: (delta: number, mouseX: number, mouseY: number) => void;
  onPan: (dx: number, dy: number) => void;
  onPauseToggle: (paused: boolean) => void;
  arcCreation: { phase: number, center: Vector2, radius: number, startAngle: number, endAngle: number } | null;
  rampCreation: { start: Vector2 } | null;
  constraintBuilder: string | null;
  polyBuilder: Vector2[];
}

export interface CanvasRef {
  exportImage: () => void;
  getStream: () => MediaStream | null;
}

const SimulationCanvas = forwardRef<CanvasRef, Props>(({ 
    state, onSelectBody, onSelectField, onSelectConstraint, onAddBody, onAddField, 
    onMoveBody, onVectorEdit, dragMode, onZoom, onPan, onPauseToggle, 
    arcCreation, rampCreation, constraintBuilder, polyBuilder 
}, ref) => {
  
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isDraggingCam, setIsDraggingCam] = useState(false);
  const [draggedBodyId, setDraggedBodyId] = useState<string | null>(null);
  const [draggedVector, setDraggedVector] = useState<{ bodyId: string, type: 'velocity' | 'force' } | null>(null);
  const [dragStart, setDragStart] = useState<Vector2>({ x: 0, y: 0 }); // Screen coordinates
  const [hasMoved, setHasMoved] = useState(false);
  
  // Touch Handling State
  const [lastTouchDist, setLastTouchDist] = useState<number | null>(null);
  
  // Field Builder
  const [fieldBuilder, setFieldBuilder] = useState<{start: Vector2, shape: FieldShape, vertices?: Vector2[]} | null>(null);
  
  // Cut Tool Line
  const [cutStart, setCutStart] = useState<Vector2 | null>(null);

  const [mousePos, setMousePos] = useState<Vector2>({ x: 0, y: 0 }); // World Pos
  const [snappedMousePos, setSnappedMousePos] = useState<Vector2>({ x: 0, y: 0 });
  
  // Image Cache
  const imageCache = useRef<Record<string, HTMLImageElement>>({});

  useImperativeHandle(ref, () => ({
    exportImage: () => {
        if (!canvasRef.current) return;
        const link = document.createElement('a');
        link.download = `${state.canvasName || 'physlab-snapshot'}-${Date.now()}.png`;
        link.href = canvasRef.current.toDataURL();
        link.click();
    },
    getStream: () => {
        if (!canvasRef.current) return null;
        return canvasRef.current.captureStream(60);
    }
  }));

  const screenToWorld = useCallback((sx: number, sy: number) => {
     return {
         x: (sx - state.camera.x) / state.camera.zoom,
         y: (sy - state.camera.y) / state.camera.zoom
     };
  }, [state.camera]);

  const getGridStep = (zoom: number) => {
    const targetPx = 80; 
    const rawVal = targetPx / zoom;
    const power = Math.floor(Math.log10(rawVal));
    const base = Math.pow(10, power);
    const mult = rawVal / base;
    return mult < 2 ? 1 * base : mult < 5 ? 2 * base : 5 * base;
  };

  const snapToGrid = (v: Vector2): Vector2 => {
      // 1. Snap to Axis
      const axisSnapThreshold = 10 / state.camera.zoom;
      if (Math.abs(v.x) < axisSnapThreshold) v.x = 0;
      if (Math.abs(v.y) < axisSnapThreshold) v.y = 0;

      // 2. Snap to Grid
      const gridSize = getGridStep(state.camera.zoom);
      const snappedX = Math.round(v.x / gridSize) * gridSize;
      const snappedY = Math.round(v.y / gridSize) * gridSize;
      const distSq = Math.pow((v.x - snappedX) * state.camera.zoom, 2) + Math.pow((v.y - snappedY) * state.camera.zoom, 2);
      
      if (distSq < 225) return { x: snappedX, y: snappedY };
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

      // Fields
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

        // --- BODY RENDERING ---
        
        // Image Handling
        if (body.image) {
            if (!imageCache.current[body.image]) {
                const img = new Image();
                img.src = body.image;
                img.onload = () => { imageCache.current[body.image!] = img; }; // Force re-render not needed as tick handles it
                imageCache.current[body.image] = img;
            }
            const img = imageCache.current[body.image];
            if (img && img.complete) {
                // Draw Image instead of shape fill
                let w=0, h=0;
                if (body.type === BodyType.CIRCLE) { w = (body.radius||20)*2; h = w; }
                else if (body.type === BodyType.BOX) { w = body.width||40; h = body.height||40; }
                else { w = 40; h = 40; } // Default
                
                ctx.drawImage(img, -w/2, -h/2, w, h);
                ctx.strokeRect(-w/2, -h/2, w, h); // Outline
            } else {
                // Fallback shape
                renderShape(ctx, body, state.camera.zoom, isSelected);
            }
        } else {
            renderShape(ctx, body, state.camera.zoom, isSelected);
        }
        
        // LABEL
        if (!body.isLine) {
            ctx.fillStyle = '#fff';
            ctx.font = `bold ${12/state.camera.zoom}px sans-serif`;
            ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
            ctx.shadowColor='black'; ctx.shadowBlur=4;
            ctx.fillText(body.label || '', 0, 0);
            ctx.shadowColor='transparent';
        }

        ctx.restore();

        // Vectors
        const drawVec = (vec: Vector2, color: string, label: string, scale: number = 1.0, dashed: boolean = false) => {
            if (Vec2.magSq(vec) < 0.0001) return;
            drawVector(ctx, vec, color, label, scale, dashed, {x:0, y:0});
        };

        if (body.showVelocity) drawVec(body.velocity, '#3b82f6', 'v');
        if (body.showAcceleration) drawVec(body.acceleration, '#a855f7', 'a', 5.0);
        if (body.showForce) {
            if (body.forceComponents) {
                Object.entries(body.forceComponents).forEach(([name, val]) => {
                     const vec = val as Vector2;
                     if (Vec2.magSq(vec) > 0.1) {
                         let color = '#fff';
                         if (name.includes('Gravity')) color = '#22c55e'; if (name.includes('Normal')) color = '#eab308';
                         if (name.includes('Friction')) color = '#f97316'; if (name.includes('Electric')) color = '#f472b6';
                         drawVec(vec, color, name, 1.0, true);
                     }
                });
            }
            drawVec(body.force, '#ef4444', 'F');
        }
        
        // Interactive Vectors (Always shown when selected)
        if (isSelected || draggedVector?.bodyId === body.id) {
             drawInteractiveVector(ctx, body.velocity, '#3b82f6', 1.0, draggedVector?.bodyId === body.id && draggedVector?.type === 'velocity');
             drawInteractiveVector(ctx, body.constantForce, '#ef4444', 1.0, draggedVector?.bodyId === body.id && draggedVector?.type === 'force');
        }

        ctx.restore();
        
        // Custom Graph Path
        if (body.customGraph && body.customGraph.show && body.customGraph.data && body.customGraph.data.length > 1) {
             ctx.save();
             ctx.beginPath();
             const start = body.customGraph.data[0];
             ctx.moveTo(start.x, -start.y); 
             for(let i=1; i<body.customGraph.data.length; i++) {
                 const p = body.customGraph.data[i];
                 ctx.lineTo(p.x, -p.y);
             }
             ctx.strokeStyle = body.customGraph.color;
             ctx.lineWidth = 2 / state.camera.zoom;
             ctx.stroke();
             ctx.restore();
        }

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
              ctx.lineJoin = 'round'; 
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
            else ctx.stroke();

            // Constraint Anchors
            ctx.fillStyle = '#fff';
            const r = 3 / state.camera.zoom;
            ctx.beginPath(); ctx.arc(pA.x, pA.y, r, 0, Math.PI*2); ctx.fill();
            ctx.beginPath(); ctx.arc(pB.x, pB.y, r, 0, Math.PI*2); ctx.fill();
        }
      });

      // BUILDER GUIDES
      drawBuilderGuides(ctx);

      // COORDINATE SYSTEM
      drawCoordinateSystem(ctx, rect.width/dpr, rect.height/dpr);

      ctx.restore();
    };

    render();
  }, [state, mousePos, snappedMousePos, arcCreation, rampCreation, draggedBodyId, dragMode, constraintBuilder, polyBuilder, cutStart, fieldBuilder, draggedVector]);

  // --- Drawing Helpers ---
  
  const renderShape = (ctx: CanvasRenderingContext2D, body: PhysicsBody, zoom: number, isSelected: boolean) => {
        if (body.type === BodyType.CIRCLE) {
          ctx.beginPath();
          const r = body.isParticle ? 6 / zoom : (body.radius || 20);
          ctx.arc(0, 0, r, 0, Math.PI * 2);
          if (body.isHollow) {
              ctx.strokeStyle = body.color; ctx.lineWidth = 4/zoom; ctx.stroke();
          } else {
              ctx.fill(); ctx.shadowColor = 'transparent'; ctx.stroke();
          }
          if (!body.isParticle && !body.isHollow) {
              ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(r * 0.8, 0);
              ctx.strokeStyle = 'rgba(0,0,0,0.3)'; ctx.lineWidth = 2 / zoom; ctx.stroke();
          }
        } else if (body.type === BodyType.BOX) {
          const w = body.width || 40;
          const h = body.height || 40;
          if (h <= 6 && body.inverseMass === 0) {
              drawHatchedLine(ctx, -w/2, 0, w/2, 0);
          } else {
              if (body.isHollow) {
                  ctx.strokeStyle = body.color; ctx.lineWidth = 4/zoom; ctx.strokeRect(-w/2, -h/2, w, h);
              } else {
                  ctx.fillRect(-w/2, -h/2, w, h); ctx.shadowColor = 'transparent'; ctx.strokeRect(-w/2, -h/2, w, h);
              }
          }
        } else if (body.type === BodyType.POLYGON) {
           if (body.vertices && body.vertices.length > 0) {
              ctx.beginPath(); ctx.moveTo(body.vertices[0].x, body.vertices[0].y);
              for (let i = 1; i < body.vertices.length; i++) ctx.lineTo(body.vertices[i].x, body.vertices[i].y);
              ctx.closePath();
              if (body.isHollow) {
                   ctx.strokeStyle = body.color; ctx.lineWidth = 4/zoom; ctx.stroke();
              } else {
                   ctx.fill(); ctx.shadowColor = 'transparent'; ctx.stroke();
              }
          }
        } else if (body.type === BodyType.ARC) {
             const r = body.radius || 100;
             const start = body.arcStartAngle || 0;
             const end = body.arcEndAngle || Math.PI;
             drawHatchedArc(ctx, 0, 0, r, start, end, isSelected ? '#ffffff' : body.color);
        } else if (body.type === BodyType.LINE) {
             const l = body.length || 100;
             ctx.beginPath(); ctx.moveTo(-l/2, 0); ctx.lineTo(l/2, 0);
             ctx.strokeStyle = body.color; ctx.lineWidth = 4 / zoom;
             ctx.stroke();
        }
  };

  const drawBuilderGuides = (ctx: CanvasRenderingContext2D) => {
      const zoom = state.camera.zoom;
      ctx.lineWidth = 1.5 / zoom;
      ctx.font = `${12/zoom}px monospace`;
      
      // Arc Builder
      if (arcCreation) {
          const { center, radius, startAngle, phase } = arcCreation;
          ctx.strokeStyle = '#3b82f6';
          
          // Phase 1: Center
          ctx.beginPath(); ctx.arc(center.x, center.y, 4/zoom, 0, Math.PI*2); ctx.fillStyle='#3b82f6'; ctx.fill();

          if (phase >= 2) {
              // Phase 2: Radius
              ctx.beginPath(); ctx.arc(center.x, center.y, radius, 0, Math.PI*2); 
              ctx.setLineDash([5/zoom, 5/zoom]); ctx.stroke(); ctx.setLineDash([]);
              
              const startPos = { x: center.x + Math.cos(startAngle)*radius, y: center.y + Math.sin(startAngle)*radius };
              ctx.beginPath(); ctx.moveTo(center.x, center.y); ctx.lineTo(startPos.x, startPos.y); ctx.stroke();
              
              // Text
              ctx.fillStyle = '#fff';
              ctx.fillText(`R: ${radius.toFixed(2)}`, center.x + 10/zoom, center.y - 10/zoom);
          }
          
          if (phase === 3) {
             // Phase 3: Arc Sweep
             const currentAngle = Math.atan2(mousePos.y - center.y, mousePos.x - center.x);
             ctx.beginPath(); 
             ctx.arc(center.x, center.y, radius, startAngle, currentAngle); 
             ctx.strokeStyle = '#fff'; ctx.lineWidth = 3/zoom; ctx.stroke();
          }
      }

      // Ramp Builder
      if (rampCreation) {
          ctx.strokeStyle = '#f59e0b';
          ctx.beginPath(); ctx.moveTo(rampCreation.start.x, rampCreation.start.y); ctx.lineTo(mousePos.x, mousePos.y); ctx.stroke();
          
          const delta = Vec2.sub(mousePos, rampCreation.start);
          const len = Vec2.mag(delta);
          let angle = Math.atan2(delta.y, delta.x) * (180/Math.PI);
          
          // Angle Snap - Conditional (Relaxed)
          const rad = Math.atan2(delta.y, delta.x);
          let displayAngle = rad * 180 / Math.PI;
          
          // Snap if close to 15 degrees
          const step = 15;
          const snapped = Math.round(displayAngle / step) * step;
          if (Math.abs(displayAngle - snapped) < 5) {
              displayAngle = snapped;
          }
          
          const finalRad = displayAngle * Math.PI / 180;
          const snappedEnd = { x: rampCreation.start.x + len * Math.cos(finalRad), y: rampCreation.start.y + len * Math.sin(finalRad) };

          // Redraw line to snapped
          ctx.beginPath(); ctx.moveTo(rampCreation.start.x, rampCreation.start.y); ctx.lineTo(snappedEnd.x, snappedEnd.y); ctx.stroke();

          ctx.fillStyle = '#fff';
          ctx.fillText(`L: ${len.toFixed(2)}`, rampCreation.start.x, rampCreation.start.y - 10/zoom);
          ctx.fillText(`A: ${displayAngle.toFixed(0)}°`, rampCreation.start.x, rampCreation.start.y - 25/zoom);
      }
      
      // Field Builder
      if (fieldBuilder) {
          ctx.strokeStyle = '#10b981';
          if (fieldBuilder.shape === FieldShape.BOX) {
               ctx.strokeRect(fieldBuilder.start.x, fieldBuilder.start.y, snappedMousePos.x - fieldBuilder.start.x, snappedMousePos.y - fieldBuilder.start.y);
          } else if (fieldBuilder.shape === FieldShape.CIRCLE) {
               const r = Vec2.dist(fieldBuilder.start, snappedMousePos);
               ctx.beginPath(); ctx.arc(fieldBuilder.start.x, fieldBuilder.start.y, r, 0, Math.PI*2); ctx.stroke();
          }
      }

      // Poly Builder
      if (polyBuilder.length > 0) {
          ctx.strokeStyle = '#10b981';
          ctx.beginPath();
          ctx.moveTo(polyBuilder[0].x, polyBuilder[0].y);
          for(let i=1; i<polyBuilder.length; i++) ctx.lineTo(polyBuilder[i].x, polyBuilder[i].y);
          ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
          ctx.stroke();

          polyBuilder.forEach(p => {
              ctx.beginPath(); ctx.arc(p.x, p.y, 3/zoom, 0, Math.PI*2); ctx.fillStyle='#10b981'; ctx.fill();
          });
      }

      // Cut Line
      if (dragMode === 'tool_cut' && cutStart) {
          ctx.strokeStyle = '#ef4444';
          ctx.setLineDash([5/zoom, 3/zoom]);
          ctx.beginPath(); ctx.moveTo(cutStart.x, cutStart.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y);
          ctx.stroke(); ctx.setLineDash([]);
      }
      
      // Constraint Builder
      if (constraintBuilder) {
          const body = state.bodies.find(b => b.id === constraintBuilder);
          if (body) {
              ctx.strokeStyle = '#fbbf24'; ctx.setLineDash([5/zoom, 5/zoom]);
              ctx.beginPath(); ctx.moveTo(body.position.x, body.position.y); ctx.lineTo(snappedMousePos.x, snappedMousePos.y); ctx.stroke(); ctx.setLineDash([]);
          }
      }
  };

  const drawCoordinateSystem = (ctx: CanvasRenderingContext2D, w: number, h: number) => {
      const zoom = state.camera.zoom;
      const camX = state.camera.x;
      const camY = state.camera.y;
      
      // Visible World Bounds
      const left = -camX / zoom;
      const top = -camY / zoom;
      const right = left + w / zoom;
      const bottom = top + h / zoom;
      
      const step = getGridStep(zoom) * 2; 

      ctx.fillStyle = 'rgba(255,255,255,0.7)';
      ctx.font = `${10/zoom}px monospace`;
      
      // Ruler Positions
      // We want the ruler numbers to be readable. 
      // X Ruler is at y = bottom of screen (in world coord)
      const rulerY = bottom - 20/zoom;
      ctx.textAlign = 'center';
      
      for (let x = Math.ceil(left/step)*step; x < right; x += step) {
          ctx.fillText(x.toFixed(0), x, rulerY);
          ctx.fillRect(x - 0.5/zoom, rulerY - 5/zoom, 1/zoom, 5/zoom);
      }
      
      // Y Ruler is at x = left of screen (in world coord)
      const rulerX = left + 20/zoom;
      ctx.textAlign = 'left';
      
      for (let y = Math.ceil(top/step)*step; y < bottom; y += step) {
          ctx.fillText((-y).toFixed(0), rulerX, y + 3/zoom);
          ctx.fillRect(rulerX - 5/zoom, y - 0.5/zoom, 5/zoom, 1/zoom);
      }
  };

  const drawHatchedArc = (ctx: CanvasRenderingContext2D, cx: number, cy: number, r: number, start: number, end: number, color: string) => {
      const zoom = state.camera.zoom;
      ctx.beginPath();
      ctx.arc(cx, cy, r, start, end);
      ctx.strokeStyle = color;
      ctx.lineWidth = 2 / zoom;
      ctx.stroke();
      
      const hatchLen = 10 / zoom;
      const step = 0.2; 
      let range = end - start;
      if (range <= 0) range += Math.PI * 2; 
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
      ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
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
          const hx = px + Math.cos(normal - 0.5) * hatchLen;
          const hy = py + Math.sin(normal - 0.5) * hatchLen;
          ctx.moveTo(px, py); ctx.lineTo(hx, hy);
      }
      ctx.stroke();
  };

  const drawFieldGrid = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
      const spacing = 100 / state.camera.zoom; 
      const startX = Math.floor(((0 - state.camera.x) / state.camera.zoom) / spacing) * spacing;
      const startY = Math.floor(((0 - state.camera.y) / state.camera.zoom) / spacing) * spacing;
      const endX = startX + (width / state.camera.zoom);
      const endY = startY + (height / state.camera.zoom);

      for (let x = startX; x < endX; x += spacing) {
          for (let y = startY; y < endY; y += spacing) {
              let hasField = false;
              for (const field of state.fields) {
                  if (!field.visible) continue;
                  if (field.shape === FieldShape.BOX && x >= field.position.x && x <= field.position.x + field.size.x && y >= field.position.y && y <= field.position.y + field.size.y) hasField=true;
                  else if (field.shape === FieldShape.CIRCLE && (x - field.position.x)**2 + (y - field.position.y)**2 <= (field.radius || 100)**2) hasField=true;
                  else if (field.type === FieldType.CUSTOM) hasField = true;
                  if (hasField) break;
              }
              if (hasField) {
                  ctx.fillStyle = 'rgba(255,255,255,0.1)';
                  ctx.fillRect(x, y, 2/state.camera.zoom, 2/state.camera.zoom);
              }
          }
      }
  };

  const drawVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, label: string, scale: number, dashed: boolean, offset: Vector2) => {
      const visualVec = Vec2.mul(vec, scale); 
      const len = Vec2.mag(visualVec);
      if (len * state.camera.zoom < 3) return; 

      ctx.save();
      ctx.translate(offset.x, offset.y);
      ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(visualVec.x, visualVec.y);
      ctx.strokeStyle = color; ctx.lineWidth = (dashed ? 1.5 : 2) / state.camera.zoom;
      if (dashed) ctx.setLineDash([4 / state.camera.zoom, 2 / state.camera.zoom]);
      ctx.stroke(); ctx.setLineDash([]);
      
      // Arrow
      const angle = Math.atan2(visualVec.y, visualVec.x);
      const headLen = 8 / state.camera.zoom;
      ctx.beginPath();
      ctx.moveTo(visualVec.x, visualVec.y);
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle - Math.PI / 6), visualVec.y - headLen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(visualVec.x - headLen * Math.cos(angle + Math.PI / 6), visualVec.y - headLen * Math.sin(angle + Math.PI / 6));
      ctx.fillStyle = color;
      ctx.fill();

      // Label
      ctx.fillStyle = '#fff';
      ctx.font = `${10/state.camera.zoom}px sans-serif`;
      ctx.fillText(label, visualVec.x + 5/state.camera.zoom, visualVec.y + 5/state.camera.zoom);

      ctx.restore();
  };
  
  const drawInteractiveVector = (ctx: CanvasRenderingContext2D, vec: Vector2, color: string, scale: number = 1.0, isInteractive: boolean) => {
      const visualVec = Vec2.mul(vec, scale);
      drawVector(ctx, visualVec, color, "", 1.0, false, {x:0, y:0});
      
      // Draw Handle
      const handlePos = visualVec;
      ctx.fillStyle = isInteractive ? '#fff' : color;
      ctx.beginPath(); 
      ctx.arc(handlePos.x, handlePos.y, 4 / state.camera.zoom, 0, Math.PI*2);
      ctx.fill();
      if (isInteractive) {
          ctx.strokeStyle = color; ctx.lineWidth = 1/state.camera.zoom; ctx.stroke();
      }
  };

  const drawGrid = (ctx: CanvasRenderingContext2D) => {
      const zoom = state.camera.zoom;
      const gridSize = getGridStep(zoom);
      const farLeft = ((-state.camera.x / zoom)); const farTop = ((-state.camera.y / zoom));
      const w = (ctx.canvas.width / window.devicePixelRatio) / zoom; const h = (ctx.canvas.height / window.devicePixelRatio) / zoom;
      
      ctx.strokeStyle = 'rgba(255,255,255,0.08)'; ctx.lineWidth = 1 / zoom;
      const startX = Math.floor(farLeft/gridSize)*gridSize; const endX = farLeft+w;
      const startY = Math.floor(farTop/gridSize)*gridSize; const endY = farTop+h;
      
      ctx.beginPath();
      for(let x = startX; x < endX + gridSize; x+=gridSize) { 
          if (Math.abs(x) < 0.1) continue; 
          ctx.moveTo(x, farTop); ctx.lineTo(x, farTop+h); 
      }
      for(let y = startY; y < endY + gridSize; y+=gridSize) { 
          if (Math.abs(y) < 0.1) continue; 
          ctx.moveTo(farLeft, y); ctx.lineTo(farLeft+w, y); 
      }
      ctx.stroke();

      // Axis Lines
      ctx.strokeStyle = '#64748b'; ctx.lineWidth = 2 / zoom; 
      ctx.beginPath(); ctx.moveTo(farLeft, 0); ctx.lineTo(farLeft+w, 0); ctx.moveTo(0, farTop); ctx.lineTo(0, farTop+h); ctx.stroke(); 
  };

  const drawSpring = (ctx: CanvasRenderingContext2D, p1: Vector2, p2: Vector2, width: number) => {
      const dist = Vec2.dist(p1, p2);
      const coils = 10;
      const dx = p2.x - p1.x; const dy = p2.y - p1.y;
      const angle = Math.atan2(dy, dx);
      
      ctx.save();
      ctx.translate(p1.x, p1.y);
      ctx.rotate(angle);
      
      ctx.beginPath();
      ctx.moveTo(0, 0);
      for(let i=0; i<=coils; i++) {
          const x = (i/coils) * dist;
          const y = (i%2===0 ? -1 : 1) * width * 0.5;
          ctx.lineTo(x, i===0 || i===coils ? 0 : y);
      }
      ctx.stroke();
      ctx.restore();
  };

  // --- Interaction Handlers ---

  const handleMouseDown = (e: React.MouseEvent | React.TouchEvent) => {
    e.preventDefault(); // Prevent scroll on touch
    let cx, cy;
    if ('touches' in e) {
        cx = e.touches[0].clientX; cy = e.touches[0].clientY;
    } else {
        cx = (e as React.MouseEvent).clientX; cy = (e as React.MouseEvent).clientY;
    }

    const rect = canvasRef.current!.getBoundingClientRect();
    const sx = cx - rect.left;
    const sy = cy - rect.top;
    const worldPos = screenToWorld(sx, sy);
    const snapped = snapToGrid(worldPos);

    setDragStart({ x: cx, y: cy });
    setHasMoved(false);

    // 1. Vector Handles (Hit Check)
    const HIT_RADIUS_SCREEN = 20;
    const hitRadiusWorld = HIT_RADIUS_SCREEN / state.camera.zoom;
    
    if (state.selectedBodyId) {
         const body = state.bodies.find(b => b.id === state.selectedBodyId);
         if (body) {
             const checkHandle = (vec: Vector2, type: 'velocity' | 'force') => {
                 const tip = Vec2.add(body.position, vec); 
                 if (Vec2.dist(worldPos, tip) < hitRadiusWorld) { 
                     setDraggedVector({ bodyId: body.id, type });
                     onPauseToggle(true); 
                     return true;
                 }
                 return false;
             };
             if (checkHandle(body.constantForce, 'force')) return;
             if (checkHandle(body.velocity, 'velocity')) return;
         }
    }

    // 2. Cut Tool Start
    if (dragMode === 'tool_cut') {
        setCutStart(snapped);
        return;
    }

    // 3. Creation Modes
    if (dragMode.startsWith('add_field_')) {
        const shape = dragMode === 'add_field_circle' ? FieldShape.CIRCLE : dragMode === 'add_field_poly' ? FieldShape.POLYGON : FieldShape.BOX;
        if (!fieldBuilder) setFieldBuilder({ start: snapped, shape });
        return;
    }
    
    if (dragMode.startsWith('add_') && !['add_spring', 'add_rod', 'add_pin', 'add_rope', 'add_arc', 'add_poly', 'add_ramp', 'add_conveyor'].includes(dragMode)) {
        onAddBody(snapped); return;
    }
    if (dragMode === 'add_arc' || dragMode === 'add_poly' || dragMode === 'add_ramp' || dragMode === 'add_conveyor') { 
        onAddBody(snapped); return; 
    }

    // 4. Selection (Reverse iterate)
    let clickedBodyId: string | null = null;
    for(let i=state.bodies.length-1; i>=0; i--) {
        const b = state.bodies[i]; let hit = false; const tol = hitRadiusWorld; 
        if (b.type === BodyType.CIRCLE) { const r = b.isParticle ? 10/state.camera.zoom : (b.radius || 20); hit = Vec2.dist(worldPos, b.position) < Math.max(r, tol); } 
        else if (b.type === BodyType.ARC) { const r = b.radius || 100; const dist = Vec2.dist(worldPos, b.position); hit = Math.abs(dist - r) < tol * 2; } 
        else if (b.type === BodyType.LINE) { 
            // Check line hit
             const l = b.length || 100;
             const p1 = Vec2.transform({x: -l/2, y:0}, b.position, b.angle);
             const p2 = Vec2.transform({x: l/2, y:0}, b.position, b.angle);
             hit = Vec2.distToSegment(worldPos, p1, p2) < tol;
        }
        else if (b.type === BodyType.BOX) { hit = Vec2.pointInRotatedBox(worldPos, b.position, b.width||40, b.height||40, b.angle); } 
        else if (b.type === BodyType.POLYGON && b.vertices) { hit = Vec2.dist(worldPos, b.position) < 40; } // Simplified poly hit
        if (hit) { clickedBodyId = b.id; break; }
    }

    if (clickedBodyId) {
        if (['add_spring', 'add_rod', 'add_pin', 'add_rope'].includes(dragMode)) { 
            onSelectBody(clickedBodyId); return; 
        }
        if (dragMode === 'tool_combine' || dragMode === 'tool_traj') {
            onSelectBody(clickedBodyId); return;
        }
        onSelectBody(clickedBodyId); setDraggedBodyId(clickedBodyId); return;
    }
    
    // Deselect if clicking empty space in select mode
    if (dragMode === 'select') {
         onSelectBody(null);
    }
    
    setIsDraggingCam(true);
  };

  const handleMouseMove = (e: React.MouseEvent | React.TouchEvent) => {
      // Touch Pinch Zoom logic
      if ('touches' in e && e.touches.length === 2) {
          const t1 = e.touches[0];
          const t2 = e.touches[1];
          const dist = Math.hypot(t1.clientX - t2.clientX, t1.clientY - t2.clientY);
          
          if (lastTouchDist !== null) {
              const delta = dist - lastTouchDist;
              const cx = (t1.clientX + t2.clientX) / 2;
              const cy = (t1.clientY + t2.clientY) / 2;
              onZoom(delta > 0 ? -100 : 100, cx, cy); // Simple zoom direction
          }
          setLastTouchDist(dist);
          return;
      }

      let cx, cy;
      if ('touches' in e) {
          cx = e.touches[0].clientX; cy = e.touches[0].clientY;
      } else {
          cx = (e as React.MouseEvent).clientX; cy = (e as React.MouseEvent).clientY;
      }

      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = cx - rect.left;
      const sy = cy - rect.top;
      const worldPos = screenToWorld(sx, sy);
      setMousePos(worldPos); 
      
      const moveDist = Math.sqrt(Math.pow(cx - dragStart.x, 2) + Math.pow(cy - dragStart.y, 2));
      if (moveDist > 3) setHasMoved(true);
      
      let finalSnapped = snapToGrid(worldPos);
      setSnappedMousePos(finalSnapped);
      
      if (draggedVector) {
          const body = state.bodies.find(b => b.id === draggedVector.bodyId);
          if (body) {
              // Vector is relative to body position
              const relativeVec = Vec2.sub(worldPos, body.position); 
              onVectorEdit(draggedVector.bodyId, draggedVector.type, relativeVec);
          }
          return;
      }

      if (draggedBodyId && hasMoved) { 
          if (dragMode !== 'select_nodrag') onMoveBody(draggedBodyId, finalSnapped); 
          return; 
      }
      
      if (isDraggingCam) { 
          const dx = cx - dragStart.x;
          const dy = cy - dragStart.y;
          onPan(dx, dy);
          setDragStart({ x: cx, y: cy });
      }
  };

  const handleMouseUp = (e: React.MouseEvent | React.TouchEvent) => {
      if (dragMode === 'tool_cut' && cutStart) {
          // Trigger cut
          const rect = canvasRef.current!.getBoundingClientRect();
          let cx, cy;
          if ('changedTouches' in e) { cx=e.changedTouches[0].clientX; cy=e.changedTouches[0].clientY; } 
          else { cx=(e as React.MouseEvent).clientX; cy=(e as React.MouseEvent).clientY; }
          
          const endPos = screenToWorld(cx - rect.left, cy - rect.top);
          const snappedEnd = snapToGrid(endPos);
          
          const event = new CustomEvent('canvas-cut', { detail: { p1: cutStart, p2: snappedEnd } });
          window.dispatchEvent(event);
          setCutStart(null);
      }
      
      if (fieldBuilder) {
          // Finish Field
          const size = { x: Math.abs(snappedMousePos.x - fieldBuilder.start.x), y: Math.abs(snappedMousePos.y - fieldBuilder.start.y) };
          // Top Left
          const pos = { x: Math.min(snappedMousePos.x, fieldBuilder.start.x), y: Math.min(snappedMousePos.y, fieldBuilder.start.y) };
          onAddField(pos, size, fieldBuilder.shape, []);
          setFieldBuilder(null);
      }

      setIsDraggingCam(false);
      setDraggedBodyId(null);
      setDraggedVector(null);
      setLastTouchDist(null);
  };

  const handleWheel = (e: React.WheelEvent) => {
    onZoom(e.deltaY, e.clientX, e.clientY);
  };

  return (
    <canvas 
      ref={canvasRef}
      className={`w-full h-full block ${dragMode === 'select_nodrag' ? 'cursor-default' : 'cursor-crosshair'} touch-none`}
      onMouseDown={handleMouseDown}
      onTouchStart={handleMouseDown}
      onWheel={handleWheel}
      onMouseMove={handleMouseMove}
      onTouchMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onTouchEnd={handleMouseUp}
      onMouseLeave={handleMouseUp}
    />
  );
});

export default React.memo(SimulationCanvas);
