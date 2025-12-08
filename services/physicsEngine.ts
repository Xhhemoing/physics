
import { BodyType, Constraint, ConstraintType, PhysicsBody, PhysicsField, FieldType, Vector2, FieldShape } from '../types';
import { Vec2 } from './vectorMath';

/**
 * Physics Engine
 * Implements a Semi-Implicit Euler (Symplectic Euler) integrator for stability
 * and an Impulse-Based resolver for collisions and constraints.
 */

export class PhysicsEngine {
  
  /**
   * Main Step Function
   */
  step(
    dt: number,
    bodies: PhysicsBody[],
    constraints: Constraint[],
    fields: PhysicsField[],
    globalGravity: Vector2,
    enableCoulomb: boolean
  ): PhysicsBody[] {
    const subSteps = 8; // High substeps for stability
    const subDt = dt / subSteps;

    // Direct mutation for performance within the step, 
    // but we clone the array structure to avoid React state mutation issues.
    // Deep clone is expensive, so we clone the body objects.
    const activeBodies = new Array(bodies.length);
    for(let i=0; i<bodies.length; i++) {
        const b = bodies[i];
        // Create a shallow copy + forceComponents reset
        activeBodies[i] = { ...b, forceComponents: {} }; 
    }

    for (let i = 0; i < subSteps; i++) {
      const isLastSubstep = i === subSteps - 1;

      // 0. RESET FORCES
      for (const b of activeBodies) {
          if (b.inverseMass === 0) {
              b.force = {x: 0, y: 0};
              continue;
          }
          // Start with constant applied force (e.g. thrusters)
          b.force = b.constantForce ? { ...b.constantForce } : { x: 0, y: 0 };
          
          if (isLastSubstep && Vec2.magSq(b.force) > 0) {
             this.addForceComponent(b, 'Applied', b.force);
          }
      }

      // 1. Apply Forces (Gravity, Fields, Coulomb, Centripetal Helper)
      this.applyForces(activeBodies, fields, globalGravity, enableCoulomb, isLastSubstep);
      
      // 2. Apply Constraint Forces (Springs - Soft Constraints)
      this.applySprings(activeBodies, constraints, isLastSubstep);

      // 3. Integrate Velocity (Symplectic Euler Part 1)
      this.integrateVelocity(activeBodies, subDt);

      // 4. Resolve Collisions (Impulse-based with SAT)
      this.resolveCollisions(activeBodies, subDt, globalGravity, isLastSubstep);
      
      // 5. Resolve Hard Constraints (Rod, Pin, Rope)
      this.resolveHardConstraints(activeBodies, constraints, subDt);

      // 6. Integrate Position (Symplectic Euler Part 2)
      this.integratePosition(activeBodies, subDt);
    }

    return activeBodies;
  }

  private addForceComponent(body: PhysicsBody, name: string, force: Vector2) {
      if (!body.forceComponents) body.forceComponents = {};
      const current = body.forceComponents[name] || {x:0, y:0};
      body.forceComponents[name] = Vec2.add(current, force);
  }

  private isPointInPolygon(point: Vector2, vertices: Vector2[]): boolean {
      if (!vertices || vertices.length < 3) return false;
      let inside = false;
      for (let i = 0, j = vertices.length - 1; i < vertices.length; j = i++) {
          const xi = vertices[i].x, yi = vertices[i].y;
          const xj = vertices[j].x, yj = vertices[j].y;
          
          const intersect = ((yi > point.y) !== (yj > point.y)) &&
              (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
          if (intersect) inside = !inside;
      }
      return inside;
  }

  private evaluateEquation(eq: string, x: number, y: number, t: number): number {
      try {
          // Optimization: Simple eval check, ideally should be parsed once
          const f = new Function('x', 'y', 't', `with(Math){ return ${eq}; }`);
          const res = f(x, y, t);
          return isNaN(res) ? 0 : res;
      } catch (e) {
          return 0;
      }
  }

  private applyForces(bodies: PhysicsBody[], fields: PhysicsField[], gravity: Vector2, enableCoulomb: boolean, isLastSubstep: boolean) {
    // Helper: Apply explicit Centripetal Force for bodies on Arc Tracks
    // This stabilizes the circular motion by supplying the required turning force
    // instead of relying solely on collision resolution impulses.
    for (const body of bodies) {
        if (body.inverseMass === 0) continue;
        
        // Scan for nearby Arcs (Naive O(N*M) but acceptable for typical scene size)
        for (const arc of bodies) {
            if (arc.type !== BodyType.ARC || arc.inverseMass !== 0) continue; // Only static arcs
            
            const rArc = arc.radius || 100;
            const dist = Vec2.dist(body.position, arc.position);
            
            // If roughly on track (within small margin)
            if (Math.abs(dist - rArc) < 5) {
                // Check angles
                const d = Vec2.sub(body.position, arc.position);
                const angle = Math.atan2(d.y, d.x);
                const start = arc.arcStartAngle || 0;
                const end = arc.arcEndAngle || Math.PI;
                const norm = (rad: number) => (rad % (2*Math.PI) + 2*Math.PI) % (2*Math.PI);
                const nA = norm(angle);
                const nStart = norm(start);
                const nEnd = norm(end);
                
                let inArc = false;
                if (nStart < nEnd) {
                    inArc = nA >= nStart && nA <= nEnd;
                } else {
                    inArc = nA >= nStart || nA <= nEnd;
                }

                if (inArc) {
                    // Apply Centripetal Force: F = m * v_t^2 / r
                    // Radial vector (normal)
                    const n = Vec2.normalize(d);
                    const v = body.velocity;
                    
                    // Velocity tangential component
                    const vn = Vec2.dot(v, n);
                    const vtVec = Vec2.sub(v, Vec2.mul(n, vn));
                    const vtSq = Vec2.magSq(vtVec);
                    
                    // F_c direction: Towards center (-n)
                    // F_c magnitude: m * vt^2 / r
                    const fcMag = (body.mass * vtSq) / rArc;
                    const fCentripetal = Vec2.mul(n, -fcMag);
                    
                    body.force = Vec2.add(body.force, fCentripetal);
                    if (isLastSubstep) this.addForceComponent(body, 'Normal (Bias)', fCentripetal);
                }
            }
        }
    }

    // N-Body Coulomb Interaction
    if (enableCoulomb) {
        // Reduced constant for stability
        const k = 2000; 
        for (let i = 0; i < bodies.length; i++) {
            const b1 = bodies[i];
            if (!b1.charge) continue;

            for (let j = i + 1; j < bodies.length; j++) {
                const b2 = bodies[j];
                if (!b2.charge) continue;
                if (b1.inverseMass === 0 && b2.inverseMass === 0) continue;

                const rVec = Vec2.sub(b1.position, b2.position);
                const rSq = Vec2.magSq(rVec);
                
                // Prevent singularity
                if (rSq < 100) continue; 

                const r = Math.sqrt(rSq);
                // F = k * q1 * q2 / r^2
                const fMag = (k * b1.charge * b2.charge) / rSq;
                const fDir = Vec2.div(rVec, r);
                const f = Vec2.mul(fDir, fMag); 

                if (b1.inverseMass !== 0) {
                    b1.force = Vec2.add(b1.force, f);
                    if (isLastSubstep) this.addForceComponent(b1, 'Coulomb', f);
                }
                if (b2.inverseMass !== 0) {
                    const fNeg = Vec2.mul(f, -1);
                    b2.force = Vec2.add(b2.force, fNeg); // Newton's 3rd Law
                    if (isLastSubstep) this.addForceComponent(b2, 'Coulomb', fNeg);
                }
            }
        }
    }

    // Single body forces
    for (const body of bodies) {
      if (body.inverseMass === 0) continue; // Static body

      // 1. Global Gravity
      // F = m * g
      const fGravity = Vec2.mul(gravity, body.mass);
      body.force = Vec2.add(body.force, fGravity);
      if (isLastSubstep) this.addForceComponent(body, 'Gravity', fGravity);

      // 2. Field Forces
      for (const field of fields) {
        let inField = false;

        // Check if body center is in field
        if (field.shape === FieldShape.BOX) {
             inField = 
                body.position.x >= field.position.x &&
                body.position.x <= field.position.x + field.size.x &&
                body.position.y >= field.position.y &&
                body.position.y <= field.position.y + field.size.y;
        } else if (field.shape === FieldShape.CIRCLE) {
             const distSq = Vec2.dist(body.position, field.position);
             const r = field.radius || 100;
             inField = distSq <= r; 
        } else if (field.shape === FieldShape.POLYGON && field.vertices) {
             inField = this.isPointInPolygon(body.position, field.vertices);
        }

        if (inField) {
            if (field.type === FieldType.UNIFORM_ELECTRIC) {
                const E = field.strength as Vector2;
                const fElectric = Vec2.mul(E, body.charge);
                body.force = Vec2.add(body.force, fElectric);
                if (isLastSubstep) this.addForceComponent(body, 'Electric', fElectric);
            } else if (field.type === FieldType.UNIFORM_MAGNETIC) {
                const B = field.strength as number;
                // F = q(v x B). 2D cross product: (vx, vy, 0) x (0, 0, B) = (vy*B, -vx*B, 0)
                const fLorentz = {
                    x: body.velocity.y * B * body.charge,
                    y: -body.velocity.x * B * body.charge
                };
                body.force = Vec2.add(body.force, fLorentz);
                if (isLastSubstep) this.addForceComponent(body, 'Magnetic', fLorentz);
            } else if (field.type === FieldType.AREA_GRAVITY) {
                 const g = field.strength as Vector2;
                 const fAreaG = Vec2.mul(g, body.mass);
                 body.force = Vec2.add(body.force, fAreaG);
                 if (isLastSubstep) this.addForceComponent(body, 'Gravity', fAreaG);
            } else if (field.type === FieldType.CUSTOM && field.equations) {
                const val1 = this.evaluateEquation(field.equations.ex, body.position.x, body.position.y, 0);
                const val2 = this.evaluateEquation(field.equations.ey, body.position.x, body.position.y, 0);
                
                let fCustom = {x:0, y:0};
                // Check if it acts as Magnetic field (velocity dependent)
                if (field.customType === 'MAGNETIC') {
                    // Assume val1 is B magnitude (scalar)
                    const B = val1;
                     fCustom = {
                        x: body.velocity.y * B * body.charge,
                        y: -body.velocity.x * B * body.charge
                    };
                    if (isLastSubstep) this.addForceComponent(body, 'Magnetic', fCustom);
                } else {
                    // Default Electric (Force)
                    // F = qE
                    const E = { x: val1, y: val2 };
                    fCustom = Vec2.mul(E, body.charge);
                    if (isLastSubstep) this.addForceComponent(body, 'Electric', fCustom);
                }
                body.force = Vec2.add(body.force, fCustom);
            }
        }
      }
      
      // 3. Air Drag
      const dragCoeff = 0.005;
      const rotDrag = 0.05;
      const fDrag = Vec2.mul(body.velocity, -dragCoeff * body.mass);
      body.force = Vec2.add(body.force, fDrag);

      body.angularVelocity *= (1 - rotDrag * 0.01); 
    }
  }

  private applySprings(bodies: PhysicsBody[], constraints: Constraint[], isLastSubstep: boolean) {
    for (const c of constraints) {
      if (c.type !== ConstraintType.SPRING) continue;

      const bodyA = bodies.find(b => b.id === c.bodyAId);
      const bodyB = bodies.find(b => b.id === c.bodyBId);

      if (!bodyA || !bodyB) continue;

      const rA = Vec2.rotate(c.localAnchorA, bodyA.angle);
      const rB = Vec2.rotate(c.localAnchorB, bodyB.angle);
      const pA = Vec2.add(bodyA.position, rA);
      const pB = Vec2.add(bodyB.position, rB);

      const delta = Vec2.sub(pB, pA);
      const dist = Vec2.mag(delta);
      const direction = dist === 0 ? {x:0, y:0} : Vec2.div(delta, dist);

      const forceMag = -c.stiffness * (dist - c.length);
      
      const vA_point = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, rA));
      const vB_point = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, rB));
      const relVel = Vec2.sub(vB_point, vA_point);
      const dampingForce = -c.damping * Vec2.dot(relVel, direction);

      const totalForce = Vec2.mul(direction, forceMag + dampingForce);

      if (bodyA.inverseMass !== 0) {
        bodyA.force = Vec2.sub(bodyA.force, totalForce);
        const torque = Vec2.cross(rA, Vec2.mul(totalForce, -1)); 
        bodyA.angularVelocity += torque * bodyA.inverseInertia; 
        
        if (isLastSubstep) this.addForceComponent(bodyA, 'Spring', Vec2.mul(totalForce, -1));
      }
      if (bodyB.inverseMass !== 0) {
        bodyB.force = Vec2.add(bodyB.force, totalForce);
        const torque = Vec2.cross(rB, totalForce);
        bodyB.angularVelocity += torque * bodyB.inverseInertia;

        if (isLastSubstep) this.addForceComponent(bodyB, 'Spring', totalForce);
      }
    }
  }

  private integrateVelocity(bodies: PhysicsBody[], dt: number) {
    for (const body of bodies) {
      if (body.inverseMass === 0) continue;
      // Symplectic Euler: v += a * dt
      const acc = Vec2.mul(body.force, body.inverseMass);
      body.acceleration = acc; 
      body.velocity = Vec2.add(body.velocity, Vec2.mul(acc, dt));
    }
  }

  private integratePosition(bodies: PhysicsBody[], dt: number) {
    for (const body of bodies) {
      if (body.inverseMass === 0) continue;
      // Symplectic Euler: x += v * dt
      body.position = Vec2.add(body.position, Vec2.mul(body.velocity, dt));
      body.angle += body.angularVelocity * dt;
    }
  }

  private resolveCollisions(bodies: PhysicsBody[], dt: number, gravity: Vector2, isLastSubstep: boolean) {
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const bodyA = bodies[i];
        const bodyB = bodies[j];
        
        if (bodyA.inverseMass === 0 && bodyB.inverseMass === 0) continue;

        const manifold = this.detectCollision(bodyA, bodyB);

        if (manifold) {
            const { normal, depth } = manifold;
            
            // 1. Positional Correction (Baumgarte)
            // Fix: Reduced percentage and slop to prevent jitter on smooth surfaces
            const percent = 0.4; 
            const slop = 0.02; 
            const correctionMag = Math.max(depth - slop, 0) / (bodyA.inverseMass + bodyB.inverseMass) * percent;
            const correction = Vec2.mul(normal, correctionMag);
            
            if (bodyA.inverseMass !== 0) bodyA.position = Vec2.sub(bodyA.position, Vec2.mul(correction, bodyA.inverseMass));
            if (bodyB.inverseMass !== 0) bodyB.position = Vec2.add(bodyB.position, Vec2.mul(correction, bodyB.inverseMass));

            // 2. Impulse Resolution
            const vA = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, Vec2.zero()));
            const vB = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, Vec2.zero()));

            const relVel = Vec2.sub(vB, vA);
            const velAlongNormal = Vec2.dot(relVel, normal);

            // Fix: If already separating, do not apply impulse
            if (velAlongNormal > 0) continue;

            let e = Math.min(bodyA.restitution, bodyB.restitution);
            const gravityMag = Vec2.mag(gravity);
            
            // Resting contact threshold
            if (Math.abs(velAlongNormal) < gravityMag * dt * 4) {
                e = 0;
            }

            let jScalar = -(1 + e) * velAlongNormal;
            jScalar /= (bodyA.inverseMass + bodyB.inverseMass);

            const impulse = Vec2.mul(normal, jScalar);
            
            if (bodyA.inverseMass !== 0) bodyA.velocity = Vec2.sub(bodyA.velocity, Vec2.mul(impulse, bodyA.inverseMass));
            if (bodyB.inverseMass !== 0) bodyB.velocity = Vec2.add(bodyB.velocity, Vec2.mul(impulse, bodyB.inverseMass));
            
            if (isLastSubstep) {
                const fNormal = Vec2.div(impulse, dt);
                if (bodyA.inverseMass !== 0) this.addForceComponent(bodyA, 'Normal', Vec2.mul(fNormal, -1));
                if (bodyB.inverseMass !== 0) this.addForceComponent(bodyB, 'Normal', fNormal);
            }

            // 3. Friction
            // Fix: Handle zero friction correctly on smooth tracks
            const mu = Math.sqrt(bodyA.friction * bodyB.friction);
            
            if (mu > 0.001) {
                let tangent = Vec2.sub(relVel, Vec2.mul(normal, velAlongNormal));
                const tangentMag = Vec2.mag(tangent);
                
                if (tangentMag > 0.0001) {
                    tangent = Vec2.div(tangent, tangentMag);
                    
                    let surfaceVelA = bodyA.surfaceSpeed || 0;

                    let jTangent = -Vec2.dot(relVel, tangent) / (bodyA.inverseMass + bodyB.inverseMass);
                    
                    let frictionImpulse: Vector2;

                    if (surfaceVelA !== 0 && bodyA.inverseMass === 0) {
                         const convDir = Vec2.rotate({x:1, y:0}, bodyA.angle);
                         const convVel = Vec2.mul(convDir, surfaceVelA);
                         const relVelWithConv = Vec2.sub(vB, Vec2.add(vA, convVel));
                         
                         const vn = Vec2.dot(relVelWithConv, normal);
                         let vtVec = Vec2.sub(relVelWithConv, Vec2.mul(normal, vn));
                         const vtMag = Vec2.mag(vtVec);
                         let tConv = {x:0, y:0};
                         if (vtMag > 0.0001) tConv = Vec2.div(vtVec, vtMag);

                         const jtConv = -Vec2.dot(relVelWithConv, tConv) / (bodyA.inverseMass + bodyB.inverseMass);
                         
                         let fImp = jtConv;
                         const maxFriction = jScalar * mu;
                         if (Math.abs(fImp) > maxFriction) {
                             fImp = Math.sign(fImp) * maxFriction;
                         }
                         frictionImpulse = Vec2.mul(tConv, fImp);

                    } else {
                        if (Math.abs(jTangent) < jScalar * mu) {
                            frictionImpulse = Vec2.mul(tangent, jTangent);
                        } else {
                            frictionImpulse = Vec2.mul(tangent, -jScalar * mu);
                        }
                    }

                    if (bodyA.inverseMass !== 0) bodyA.velocity = Vec2.sub(bodyA.velocity, Vec2.mul(frictionImpulse, bodyA.inverseMass));
                    if (bodyB.inverseMass !== 0) bodyB.velocity = Vec2.add(bodyB.velocity, Vec2.mul(frictionImpulse, bodyB.inverseMass));
                    
                    if (isLastSubstep) {
                        const fFriction = Vec2.div(frictionImpulse, dt);
                        if (bodyA.inverseMass !== 0) this.addForceComponent(bodyA, 'Friction', Vec2.mul(fFriction, -1));
                        if (bodyB.inverseMass !== 0) this.addForceComponent(bodyB, 'Friction', fFriction);
                    }
                }
            }
        }
      }
    }
  }

  // General Collision Detection Dispatcher
  private detectCollision(bodyA: PhysicsBody, bodyB: PhysicsBody): { normal: Vector2, depth: number } | null {
      // Hollow Body Handling (Chain Collision)
      if (bodyA.isHollow || bodyB.isHollow) {
          const hollow = bodyA.isHollow ? bodyA : bodyB;
          const solid = bodyA.isHollow ? bodyB : bodyA;
          
          const result = this.checkChainCollision(hollow, solid);
          if (result) {
              if (hollow === bodyB) {
                  return { normal: Vec2.mul(result.normal, -1), depth: result.depth };
              }
              return result;
          }
          return null;
      }

      // Circle vs Circle
      if (bodyA.type === BodyType.CIRCLE && bodyB.type === BodyType.CIRCLE) {
          const delta = Vec2.sub(bodyB.position, bodyA.position);
          const distSq = Vec2.magSq(delta);
          const rSum = (bodyA.radius || 10) + (bodyB.radius || 10);
          const minDistSq = rSum * rSum;
          
          if (distSq < minDistSq) { 
              const dist = Math.sqrt(distSq);
              const normal = dist === 0 ? {x:0, y:1} : Vec2.div(delta, dist);
              return {
                  normal: normal,
                  depth: rSum - dist
              };
          }
          return null;
      }

      // Arcs
      if (bodyA.type === BodyType.ARC || bodyB.type === BodyType.ARC) {
          const arc = bodyA.type === BodyType.ARC ? bodyA : bodyB;
          const other = bodyA.type === BodyType.ARC ? bodyB : bodyA;
          
          if (other.type === BodyType.CIRCLE) {
              const res = this.checkCircleArc(other, arc);
              if (res) {
                  if (arc === bodyB) return { normal: Vec2.mul(res.normal, -1), depth: res.depth };
                  return res;
              }
          }
          if (other.type === BodyType.BOX || other.type === BodyType.POLYGON) {
              const res = this.checkPolygonArc(other, arc);
              if (res) {
                   if (arc === bodyB) return { normal: Vec2.mul(res.normal, -1), depth: res.depth };
                   return res;
              }
          }
          return null;
      }

      // Plane logic
      if (bodyA.type === BodyType.PLANE || bodyB.type === BodyType.PLANE) {
          const plane = bodyA.type === BodyType.PLANE ? bodyA : bodyB;
          const other = bodyA.type === BodyType.PLANE ? bodyB : bodyA;
          
          if (other.type === BodyType.CIRCLE) {
              return this.checkCirclePlane(other, plane, plane === bodyB);
          } else if (other.type === BodyType.BOX || other.type === BodyType.POLYGON) {
              return this.checkPolygonPlane(other, plane, plane === bodyB);
          }
      }

      const isPolyA = bodyA.type === BodyType.BOX || bodyA.type === BodyType.POLYGON;
      const isPolyB = bodyB.type === BodyType.BOX || bodyB.type === BodyType.POLYGON;

      if (isPolyA && isPolyB) {
          return this.satPolygonPolygon(bodyA, bodyB);
      }
      
      if (isPolyA && bodyB.type === BodyType.CIRCLE) {
          return this.satPolygonCircle(bodyA, bodyB);
      }
      
      if (bodyA.type === BodyType.CIRCLE && isPolyB) {
          const res = this.satPolygonCircle(bodyB, bodyA);
          if (res) return { normal: Vec2.mul(res.normal, -1), depth: res.depth };
      }

      return null;
  }

  // --- Chain Collision Helpers ---
  private checkChainCollision(chain: PhysicsBody, body: PhysicsBody): { normal: Vector2, depth: number } | null {
      const chainVerts = this.getWorldVertices(chain);
      if (chainVerts.length < 2) return null;

      let maxDepth = -1;
      let bestNormal = Vec2.zero();

      // Check against all segments of the chain
      for (let i = 0; i < chainVerts.length; i++) {
          // If not closed, skip last segment
          if (i === chainVerts.length - 1 && !chain.isHollow) break; 
          // For hollow bodies, we usually assume closed loop for "container" logic,
          // but "Chain" usually implies line strip.
          // Let's assume loop if first point == last point, else strip.
          const v1 = chainVerts[i];
          const v2 = chainVerts[(i + 1) % chainVerts.length];

          // Treat each edge as a "capsule" segment with radius 0 (or small thickness)
          const thickness = 2; // Slight thickness to catch fast objects

          if (body.type === BodyType.CIRCLE) {
              const res = this.checkCircleSegment(body.position, body.radius || 10, v1, v2, thickness);
              if (res && res.depth > maxDepth) {
                  maxDepth = res.depth;
                  bestNormal = res.normal;
              }
          } else {
               // For polygons, check all vertices against this segment
               const polyVerts = this.getWorldVertices(body);
               for(const pv of polyVerts) {
                   const res = this.checkCircleSegment(pv, 0, v1, v2, thickness);
                   if (res && res.depth > maxDepth) {
                       maxDepth = res.depth;
                       bestNormal = res.normal;
                   }
               }
          }
      }

      if (maxDepth > 0) {
          return { normal: bestNormal, depth: maxDepth };
      }
      return null;
  }

  private checkCircleSegment(center: Vector2, radius: number, v1: Vector2, v2: Vector2, thickness: number): { normal: Vector2, depth: number } | null {
      const segment = Vec2.sub(v2, v1);
      const lenSq = Vec2.magSq(segment);
      let t = 0;
      if (lenSq > 0) {
          t = Vec2.dot(Vec2.sub(center, v1), segment) / lenSq;
          t = Math.max(0, Math.min(1, t));
      }
      
      const closest = Vec2.add(v1, Vec2.mul(segment, t));
      const distVec = Vec2.sub(center, closest);
      const dist = Vec2.mag(distVec);

      if (dist < radius + thickness) {
          const normal = dist > 0 ? Vec2.normalize(distVec) : Vec2.perp(Vec2.normalize(segment));
          return { normal, depth: (radius + thickness) - dist };
      }
      return null;
  }


  // --- SAT HELPERS ---
  
  private satPolygonPolygon(a: PhysicsBody, b: PhysicsBody): { normal: Vector2, depth: number } | null {
      const vertsA = this.getWorldVertices(a);
      const vertsB = this.getWorldVertices(b);
      if (vertsA.length === 0 || vertsB.length === 0) return null;

      let normal = Vec2.zero();
      let depth = Number.MAX_VALUE;

      for (let i = 0; i < vertsA.length; i++) {
          const v1 = vertsA[i];
          const v2 = vertsA[(i + 1) % vertsA.length];
          const edge = Vec2.sub(v2, v1);
          const axis = Vec2.normalize(Vec2.perp(edge));

          const [minA, maxA] = this.projectVertices(vertsA, axis);
          const [minB, maxB] = this.projectVertices(vertsB, axis);

          if (maxA < minB || maxB < minA) return null; 

          const overlap = Math.min(maxA - minB, maxB - minA);
          if (overlap < depth) {
              depth = overlap;
              normal = axis;
          }
      }

      for (let i = 0; i < vertsB.length; i++) {
          const v1 = vertsB[i];
          const v2 = vertsB[(i + 1) % vertsB.length];
          const edge = Vec2.sub(v2, v1);
          const axis = Vec2.normalize(Vec2.perp(edge));

          const [minA, maxA] = this.projectVertices(vertsA, axis);
          const [minB, maxB] = this.projectVertices(vertsB, axis);

          if (maxA < minB || maxB < minA) return null;

          const overlap = Math.min(maxA - minB, maxB - minA);
          if (overlap < depth) {
              depth = overlap;
              normal = axis;
          }
      }

      const dir = Vec2.sub(b.position, a.position);
      if (Vec2.dot(dir, normal) < 0) {
          normal = Vec2.mul(normal, -1);
      }

      return { normal, depth };
  }

  private satPolygonCircle(poly: PhysicsBody, circle: PhysicsBody): { normal: Vector2, depth: number } | null {
      const verts = this.getWorldVertices(poly);
      if (verts.length === 0) return null;

      let normal = Vec2.zero();
      let depth = Number.MAX_VALUE;

      for (let i = 0; i < verts.length; i++) {
          const v1 = verts[i];
          const v2 = verts[(i + 1) % verts.length];
          const edge = Vec2.sub(v2, v1);
          const axis = Vec2.normalize(Vec2.perp(edge));

          const [minP, maxP] = this.projectVertices(verts, axis);
          const minC = Vec2.dot(circle.position, axis) - (circle.radius || 10);
          const maxC = Vec2.dot(circle.position, axis) + (circle.radius || 10);

          if (maxP < minC || maxC < minP) return null;

          const overlap = Math.min(maxP - minC, maxC - minP);
          if (overlap < depth) {
              depth = overlap;
              normal = axis;
          }
      }

      let closestVert = verts[0];
      let minDistSq = Number.MAX_VALUE;
      for (const v of verts) {
          const d = Vec2.dist(circle.position, v); 
          if (d * d < minDistSq) {
              minDistSq = d * d;
              closestVert = v;
          }
      }
      
      const axis = Vec2.normalize(Vec2.sub(circle.position, closestVert));
      const [minP, maxP] = this.projectVertices(verts, axis);
      const minC = Vec2.dot(circle.position, axis) - (circle.radius || 10);
      const maxC = Vec2.dot(circle.position, axis) + (circle.radius || 10);

      if (maxP < minC || maxC < minP) return null;

      const overlap = Math.min(maxP - minC, maxC - minP);
      if (overlap < depth) {
          depth = overlap;
          normal = axis;
      }

      const dir = Vec2.sub(circle.position, poly.position);
      if (Vec2.dot(dir, normal) < 0) {
          normal = Vec2.mul(normal, -1);
      }

      return { normal, depth };
  }

  private getWorldVertices(body: PhysicsBody): Vector2[] {
      if (body.vertices) {
          return body.vertices.map(v => Vec2.transform(v, body.position, body.angle));
      }
      if (body.type === BodyType.BOX) {
          const w = (body.width || 40) / 2;
          const h = (body.height || 40) / 2;
          const localVerts = [
              { x: -w, y: -h },
              { x: w, y: -h },
              { x: w, y: h },
              { x: -w, y: h }
          ];
          return localVerts.map(v => Vec2.transform(v, body.position, body.angle));
      }
      return [];
  }

  private projectVertices(vertices: Vector2[], axis: Vector2): [number, number] {
      let min = Number.MAX_VALUE;
      let max = -Number.MAX_VALUE;
      for (const v of vertices) {
          const proj = Vec2.dot(v, axis);
          if (proj < min) min = proj;
          if (proj > max) max = proj;
      }
      return [min, max];
  }

  private checkCirclePlane(circle: PhysicsBody, plane: PhysicsBody, flipNormal: boolean): {normal: Vector2, depth: number} | null {
    if (!plane.normal) return null;
    let distToPlane = Vec2.dot(Vec2.sub(circle.position, plane.position), plane.normal);
    const r = circle.radius || 10;
    
    if (distToPlane < r) {
        let n = plane.normal;
        if (flipNormal) n = Vec2.mul(n, -1);
        return { normal: n, depth: r - distToPlane };
    }
    return null;
  }
  
  private checkPolygonPlane(poly: PhysicsBody, plane: PhysicsBody, flipNormal: boolean): {normal: Vector2, depth: number} | null {
      if (!plane.normal) return null;
      const verts = this.getWorldVertices(poly);
      let minSep = Number.MAX_VALUE;

      for (const v of verts) {
          const sep = Vec2.dot(Vec2.sub(v, plane.position), plane.normal);
          if (sep < minSep) minSep = sep;
      }

      if (minSep < 0) {
          let n = plane.normal;
          if (flipNormal) n = Vec2.mul(n, -1);
          return { normal: n, depth: -minSep };
      }
      return null;
  }

  private checkCircleArc(circle: PhysicsBody, arc: PhysicsBody): {normal: Vector2, depth: number} | null {
      const rArc = arc.radius || 100;
      const rCirc = circle.radius || 20;

      const d = Vec2.sub(circle.position, arc.position);
      const dist = Vec2.mag(d);
      
      const tolerance = rCirc + 10;
      
      if (Math.abs(dist - rArc) < tolerance) {
          const angle = Math.atan2(d.y, d.x); 
          const start = arc.arcStartAngle || 0;
          const end = arc.arcEndAngle || Math.PI;
          const norm = (rad: number) => (rad % (2*Math.PI) + 2*Math.PI) % (2*Math.PI);
          const nA = norm(angle);
          const nStart = norm(start);
          const nEnd = norm(end);

          let inArc = false;
          // Handle wrap-around
          if (nStart < nEnd) {
              inArc = nA >= nStart && nA <= nEnd;
          } else {
              inArc = nA >= nStart || nA <= nEnd;
          }

          if (inArc) {
              const normal = Vec2.normalize(d); 
              let depth = 0;
              
              if (dist < rArc) {
                  // Inside
                  const penetration = (dist + rCirc) - rArc;
                  if (penetration > 0) {
                      return { normal: Vec2.mul(normal, -1), depth: penetration };
                  }
              } else {
                  // Outside
                  const p = rCirc - (dist - rArc);
                  if (p > 0) {
                      return { normal: normal, depth: p };
                  }
              }
          }
      }
      return null;
  }

  private checkPolygonArc(poly: PhysicsBody, arc: PhysicsBody): {normal: Vector2, depth: number} | null {
      const verts = this.getWorldVertices(poly);
      const rArc = arc.radius || 100;
      const start = arc.arcStartAngle || 0;
      const end = arc.arcEndAngle || Math.PI;
      const norm = (rad: number) => (rad % (2*Math.PI) + 2*Math.PI) % (2*Math.PI);
      const nStart = norm(start);
      const nEnd = norm(end);

      let maxDepth = -1;
      let collisionNormal = Vec2.zero();

      for (const v of verts) {
          const d = Vec2.sub(v, arc.position);
          const dist = Vec2.mag(d);
          
          if (Math.abs(dist - rArc) < 20) { // Check proximity
              const angle = Math.atan2(d.y, d.x);
              const nA = norm(angle);

              let inArc = false;
              if (nStart < nEnd) {
                  inArc = nA >= nStart && nA <= nEnd;
              } else {
                  inArc = nA >= nStart || nA <= nEnd;
              }

              if (inArc) {
                  // Simple vertex penetration check
                  let n = Vec2.normalize(d);
                  let depth = 0;
                  
                  if (dist < rArc) {
                      depth = 5 - (rArc - dist); // Arbitrary thickness 5
                      if (depth > 0) {
                          n = Vec2.mul(n, -1); // Push towards center
                      }
                  } else {
                      depth = 5 - (dist - rArc);
                  }

                  if (depth > maxDepth) {
                      maxDepth = depth;
                      collisionNormal = n;
                  }
              }
          }
      }

      if (maxDepth > 0) {
          return { normal: collisionNormal, depth: maxDepth };
      }
      return null;
  }

  private resolveHardConstraints(bodies: PhysicsBody[], constraints: Constraint[], dt: number) {
    for (const c of constraints) {
        if (c.type === ConstraintType.SPRING) continue;

        const bodyA = bodies.find(b => b.id === c.bodyAId);
        const bodyB = bodies.find(b => b.id === c.bodyBId);
        if (!bodyA || !bodyB) continue;

        const rA = Vec2.rotate(c.localAnchorA, bodyA.angle);
        const rB = Vec2.rotate(c.localAnchorB, bodyB.angle);
        const pA = Vec2.add(bodyA.position, rA);
        const pB = Vec2.add(bodyB.position, rB);

        // --- ROD (Exact Distance) OR ROPE (Max Distance) ---
        if (c.type === ConstraintType.ROD || c.type === ConstraintType.ROPE) {
            const delta = Vec2.sub(pB, pA);
            const dist = Vec2.mag(delta);
            let diff = 0;

            if (c.type === ConstraintType.ROD) {
                diff = dist - c.length;
            } else if (c.type === ConstraintType.ROPE) {
                // Only constraint if distance > length (slack)
                if (dist > c.length) {
                    diff = dist - c.length;
                } else {
                    continue; // Slack, do nothing
                }
            }
            
            if (Math.abs(diff) > 0.001) {
                const normal = dist === 0 ? {x:0, y:1} : Vec2.normalize(delta);
                const correction = diff * 0.5; 
                const move = Vec2.mul(normal, correction);

                if (bodyA.inverseMass !== 0) bodyA.position = Vec2.add(bodyA.position, move);
                if (bodyB.inverseMass !== 0) bodyB.position = Vec2.sub(bodyB.position, move);

                // Impulse correction
                const vA = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, rA));
                const vB = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, rB));
                const relVel = Vec2.sub(vB, vA);
                const vn = Vec2.dot(relVel, normal);
                
                // If it's a rope and we are moving towards each other (slackening), don't bounce back
                // But generally we want to kill velocity moving OUT of the constraint
                if (c.type === ConstraintType.ROPE && vn < 0) {
                   // Moving inwards is fine for rope
                } else {
                    const rnA = Vec2.cross(rA, normal);
                    const rnB = Vec2.cross(rB, normal);
                    const k = bodyA.inverseMass + bodyB.inverseMass + 
                              (rnA * rnA * bodyA.inverseInertia) + 
                              (rnB * rnB * bodyB.inverseInertia);
                    
                    const lambda = -vn / k;
                    const P = Vec2.mul(normal, lambda);

                    if (bodyA.inverseMass !== 0) {
                        bodyA.velocity = Vec2.sub(bodyA.velocity, Vec2.mul(P, bodyA.inverseMass));
                        bodyA.angularVelocity -= rnA * lambda * bodyA.inverseInertia;
                    }
                    if (bodyB.inverseMass !== 0) {
                        bodyB.velocity = Vec2.add(bodyB.velocity, Vec2.mul(P, bodyB.inverseMass));
                        bodyB.angularVelocity += rnB * lambda * bodyB.inverseInertia;
                    }
                }
            }
        } else if (c.type === ConstraintType.PIN) {
             const delta = Vec2.sub(pB, pA);
             
             const move = Vec2.mul(delta, 0.5);
             if (bodyA.inverseMass !== 0) bodyA.position = Vec2.add(bodyA.position, move);
             if (bodyB.inverseMass !== 0) bodyB.position = Vec2.sub(bodyB.position, move);
             
             const vA = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, rA));
             const vB = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, rB));
             const relVel = Vec2.sub(vB, vA);
             
             const massSum = bodyA.inverseMass + bodyB.inverseMass;
             if (massSum > 0) {
                 const P = Vec2.div(relVel, -massSum); 
                 
                 if (bodyA.inverseMass !== 0) {
                    bodyA.velocity = Vec2.add(bodyA.velocity, Vec2.mul(P, -bodyA.inverseMass));
                    const torque = Vec2.cross(rA, Vec2.mul(P, -1));
                    bodyA.angularVelocity += torque * bodyA.inverseInertia;
                 }
                 if (bodyB.inverseMass !== 0) {
                    bodyB.velocity = Vec2.add(bodyB.velocity, Vec2.mul(P, bodyB.inverseMass));
                    const torque = Vec2.cross(rB, P);
                    bodyB.angularVelocity += torque * bodyB.inverseInertia;
                 }
             }
        }
    }
  }

  // --- Utility for Cutting ---
  public splitBody(body: PhysicsBody, p1: Vector2, p2: Vector2): PhysicsBody[] | null {
      const verts = this.getWorldVertices(body);
      if (verts.length < 3) return null; // Can only split polygons/boxes

      const poly1: Vector2[] = [];
      const poly2: Vector2[] = [];

      // Check which side of line p1-p2 each vertex is
      // Cross product (p2-p1) x (v-p1)
      const isLeft = (v: Vector2) => {
          return (p2.x - p1.x)*(v.y - p1.y) - (p2.y - p1.y)*(v.x - p1.x) > 0;
      };

      for (let i = 0; i < verts.length; i++) {
          const cur = verts[i];
          const next = verts[(i + 1) % verts.length];
          const curLeft = isLeft(cur);
          const nextLeft = isLeft(next);

          if (curLeft) poly1.push(cur); else poly2.push(cur);

          if (curLeft !== nextLeft) {
              const intersection = Vec2.getLineSegmentIntersection(p1, p2, cur, next);
              if (intersection) {
                  poly1.push(intersection);
                  poly2.push(intersection);
              }
          }
      }

      if (poly1.length < 3 || poly2.length < 3) return null;

      // Create 2 new bodies
      const createPart = (vs: Vector2[], idSuffix: string): PhysicsBody => {
          // Centroid
          let cx = 0, cy = 0;
          vs.forEach(v => { cx += v.x; cy += v.y; });
          cx /= vs.length; cy /= vs.length;
          const center = {x: cx, y: cy};
          const localVs = vs.map(v => ({x: v.x - center.x, y: v.y - center.y}));

          return {
              ...body,
              id: body.id + idSuffix,
              type: BodyType.POLYGON,
              position: center,
              vertices: localVs,
              mass: body.mass / 2, 
              inverseMass: body.inverseMass === 0 ? 0 : body.inverseMass * 2,
              angle: 0, 
              angularVelocity: body.angularVelocity,
              selected: false
          };
      };

      return [createPart(poly1, '_A'), createPart(poly2, '_B')];
  }
}
