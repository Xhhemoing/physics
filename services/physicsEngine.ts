

import { BodyType, Constraint, ConstraintType, PhysicsBody, PhysicsField, FieldType, Vector2, FieldShape } from '../types';
import { Vec2 } from './vectorMath';
import { CollisionSystem } from './collision';

/**
 * Physics Engine
 * Refined Semi-Implicit Euler (Symplectic Euler) integrator.
 */

export class PhysicsEngine {
  
  private collisionSystem = new CollisionSystem();

  /**
   * Main Step Function
   */
  step(
    dt: number,
    bodies: PhysicsBody[],
    constraints: Constraint[],
    fields: PhysicsField[],
    globalGravity: Vector2,
    enableCoulomb: boolean,
    enableUniversalGravity: boolean = false,
    enableAirResistance: boolean = false
  ): PhysicsBody[] {
    const subSteps = 8; // High substeps for stability
    const subDt = dt / subSteps;

    const activeBodies = new Array(bodies.length);
    for(let i=0; i<bodies.length; i++) {
        const b = bodies[i];
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
          b.force = b.constantForce ? { ...b.constantForce } : { x: 0, y: 0 };
          
          if (isLastSubstep && Vec2.magSq(b.force) > 0) {
             this.addForceComponent(b, 'Applied', b.force);
          }
      }

      // 1. Apply Forces
      this.applyForces(activeBodies, fields, globalGravity, enableCoulomb, enableUniversalGravity, isLastSubstep);
      
      // 2. Apply Air Resistance
      if (enableAirResistance) {
          this.applyAirResistance(activeBodies, isLastSubstep);
      }

      // 3. Apply Constraint Forces
      this.applySprings(activeBodies, constraints, isLastSubstep);

      // 4. Integrate Velocity
      this.integrateVelocity(activeBodies, subDt);

      // 5. Resolve Collisions
      this.resolveCollisions(activeBodies, subDt, globalGravity, isLastSubstep);
      
      // 6. Resolve Hard Constraints
      this.resolveHardConstraints(activeBodies, constraints, subDt);

      // 7. Integrate Position
      this.integratePosition(activeBodies, subDt);
    }

    return activeBodies;
  }

  private addForceComponent(body: PhysicsBody, name: string, force: Vector2) {
      if (!body.forceComponents) body.forceComponents = {};
      const current = body.forceComponents[name] || {x:0, y:0};
      body.forceComponents[name] = Vec2.add(current, force);
  }

  private evaluateEquation(eq: string, x: number, y: number, t: number): number {
      try {
          const f = new Function('x', 'y', 't', `with(Math){ return ${eq}; }`);
          const res = f(x, y, t);
          return isNaN(res) ? 0 : res;
      } catch (e) {
          return 0;
      }
  }

  private applyForces(
      bodies: PhysicsBody[], 
      fields: PhysicsField[], 
      gravity: Vector2, 
      enableCoulomb: boolean, 
      enableUniversalGravity: boolean,
      isLastSubstep: boolean
    ) {
    
    // N-Body Interactions
    if (enableCoulomb || enableUniversalGravity) {
        const kCoulomb = 2000; 
        const kGravity = 500; 

        for (let i = 0; i < bodies.length; i++) {
            const b1 = bodies[i];
            if (b1.inverseMass === 0) continue; 

            for (let j = 0; j < bodies.length; j++) {
                if (i === j) continue;
                const b2 = bodies[j];
                
                const rVec = Vec2.sub(b1.position, b2.position);
                const rSq = Vec2.magSq(rVec);
                
                if (rSq < 100) continue; 
                const r = Math.sqrt(rSq);
                const dir = Vec2.div(rVec, r); 

                let fTotal = {x:0, y:0};

                if (enableCoulomb && b1.charge !== 0 && b2.charge !== 0) {
                     const fMag = (kCoulomb * b1.charge * b2.charge) / rSq;
                     const f = Vec2.mul(dir, fMag);
                     fTotal = Vec2.add(fTotal, f);
                     if (isLastSubstep) this.addForceComponent(b1, 'Coulomb', f);
                }

                if (enableUniversalGravity && b1.mass > 0 && b2.mass > 0) {
                     const fMag = -(kGravity * b1.mass * b2.mass) / rSq;
                     const f = Vec2.mul(dir, fMag);
                     fTotal = Vec2.add(fTotal, f);
                     if (isLastSubstep) this.addForceComponent(b1, 'Univ. Gravity', f);
                }
                
                b1.force = Vec2.add(b1.force, fTotal);
            }
        }
    }

    // Single body forces
    for (const body of bodies) {
      if (body.inverseMass === 0) continue; 

      // 1. Global Gravity
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
             // Basic bounding check can be added here
             inField = true; // Simplified for custom fields, check usually not needed for global fields
        }

        if (inField) {
            if (field.type === FieldType.UNIFORM_ELECTRIC) {
                const E = field.strength as Vector2;
                const fElectric = Vec2.mul(E, body.charge);
                body.force = Vec2.add(body.force, fElectric);
                if (isLastSubstep) this.addForceComponent(body, 'Electric', fElectric);
            } else if (field.type === FieldType.UNIFORM_MAGNETIC) {
                const B = field.strength as number;
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
                if (field.customType === 'MAGNETIC') {
                    const B = val1;
                     fCustom = {
                        x: body.velocity.y * B * body.charge,
                        y: -body.velocity.x * B * body.charge
                    };
                    if (isLastSubstep) this.addForceComponent(body, 'Magnetic', fCustom);
                } else {
                    const E = { x: val1, y: val2 };
                    fCustom = Vec2.mul(E, body.charge);
                    if (isLastSubstep) this.addForceComponent(body, 'Electric', fCustom);
                }
                body.force = Vec2.add(body.force, fCustom);
            }
        }
      }
      
      this.applyCentripetalAssist(body, bodies);
    }
  }

  private applyAirResistance(bodies: PhysicsBody[], isLastSubstep: boolean) {
      const dragCoeff = 0.002;
      const rotDrag = 0.05;

      for (const body of bodies) {
          if (body.inverseMass === 0) continue;
          
          const fDrag = Vec2.mul(body.velocity, -dragCoeff * body.mass);
          body.force = Vec2.add(body.force, fDrag);
          body.angularVelocity *= (1 - rotDrag * 0.01); 

          if (isLastSubstep && Vec2.magSq(fDrag) > 0.0001) {
              this.addForceComponent(body, 'Drag', fDrag);
          }
      }
  }

  private applyCentripetalAssist(body: PhysicsBody, allBodies: PhysicsBody[]) {
     for (const arc of allBodies) {
        if (arc.type !== BodyType.ARC || arc.inverseMass !== 0) continue; 
        const rArc = arc.radius || 100;
        const dist = Vec2.dist(body.position, arc.position);
        if (Math.abs(dist - rArc) < 15) {
            const d = Vec2.sub(body.position, arc.position);
            const angle = Math.atan2(d.y, d.x);
            const start = arc.arcStartAngle || 0;
            const end = arc.arcEndAngle || Math.PI;
            const norm = (rad: number) => (rad % (2*Math.PI) + 2*Math.PI) % (2*Math.PI);
            const nA = norm(angle); const nStart = norm(start); const nEnd = norm(end);
            
            let inArc = false;
            if (nStart < nEnd) { inArc = nA >= nStart && nA <= nEnd; } else { inArc = nA >= nStart || nA <= nEnd; }

            if (inArc) {
                const n = Vec2.normalize(d);
                const v = body.velocity;
                const vn = Vec2.dot(v, n);
                const vtVec = Vec2.sub(v, Vec2.mul(n, vn));
                const vtSq = Vec2.magSq(vtVec);
                if (vtSq > 1) {
                    const fcMag = (body.mass * vtSq) / rArc;
                    const fCentripetal = Vec2.mul(n, -fcMag);
                    body.force = Vec2.add(body.force, fCentripetal);
                }
            }
        }
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
      const acc = Vec2.mul(body.force, body.inverseMass);
      body.acceleration = acc; 
      body.velocity = Vec2.add(body.velocity, Vec2.mul(acc, dt));
    }
  }

  private integratePosition(bodies: PhysicsBody[], dt: number) {
    for (const body of bodies) {
      if (body.inverseMass === 0) continue;
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

        const manifold = this.collisionSystem.detectCollision(bodyA, bodyB);

        if (manifold) {
            const { normal, depth } = manifold;
            
            // 1. Positional Correction (Baumgarte)
            const percent = 0.5; 
            const slop = 0.01; 
            const correctionMag = Math.max(depth - slop, 0) / (bodyA.inverseMass + bodyB.inverseMass) * percent;
            const correction = Vec2.mul(normal, correctionMag);
            
            if (bodyA.inverseMass !== 0) bodyA.position = Vec2.sub(bodyA.position, Vec2.mul(correction, bodyA.inverseMass));
            if (bodyB.inverseMass !== 0) bodyB.position = Vec2.add(bodyB.position, Vec2.mul(correction, bodyB.inverseMass));

            // 2. Impulse Resolution
            const vA = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, Vec2.zero()));
            const vB = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, Vec2.zero()));

            const relVel = Vec2.sub(vB, vA);
            const velAlongNormal = Vec2.dot(relVel, normal);

            if (velAlongNormal > 0) continue;

            let e = Math.min(bodyA.restitution, bodyB.restitution);
            const gravityMag = Vec2.mag(gravity);
            
            if (Math.abs(velAlongNormal) < gravityMag * dt * 5) {
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
            const mu = Math.sqrt(bodyA.friction * bodyB.friction);
            
            if (mu > 0.0001) {
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

        if (c.type === ConstraintType.ROD || c.type === ConstraintType.ROPE) {
            const delta = Vec2.sub(pB, pA);
            const dist = Vec2.mag(delta);
            let diff = 0;

            if (c.type === ConstraintType.ROD) {
                diff = dist - c.length;
            } else if (c.type === ConstraintType.ROPE) {
                if (dist > c.length) {
                    diff = dist - c.length;
                } else {
                    continue; 
                }
            }
            
            if (Math.abs(diff) > 0.001) {
                const normal = dist === 0 ? {x:0, y:1} : Vec2.normalize(delta);
                const correction = diff * 0.5; 
                const move = Vec2.mul(normal, correction);

                if (bodyA.inverseMass !== 0) bodyA.position = Vec2.add(bodyA.position, move);
                if (bodyB.inverseMass !== 0) bodyB.position = Vec2.sub(bodyB.position, move);

                const vA = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, rA));
                const vB = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, rB));
                const relVel = Vec2.sub(vB, vA);
                const vn = Vec2.dot(relVel, normal);
                
                if (c.type === ConstraintType.ROPE && vn < 0) {
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

  public splitBody(body: PhysicsBody, p1: Vector2, p2: Vector2): PhysicsBody[] | null {
      const verts = this.collisionSystem.getWorldVertices(body);
      if (verts.length < 3) return null; 

      const poly1: Vector2[] = [];
      const poly2: Vector2[] = [];

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

      const createPart = (vs: Vector2[], idSuffix: string): PhysicsBody => {
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