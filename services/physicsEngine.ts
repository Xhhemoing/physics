
import { BodyType, Constraint, ConstraintType, PhysicsBody, PhysicsField, FieldType, Vector2 } from '../types';
import { Vec2 } from './vectorMath';

/**
 * Physics Engine
 * Implements a Semi-Implicit Euler (Symplectic Euler) integrator for stability
 * and an Impulse-Based resolver for collisions and constraints.
 * 
 * Collision Detection uses Separating Axis Theorem (SAT).
 */

export class PhysicsEngine {
  // Global settings
  private restitutionThreshold = 0.5; // Velocity threshold for bouncing

  /**
   * Main Step Function
   */
  step(
    dt: number,
    bodies: PhysicsBody[],
    constraints: Constraint[],
    fields: PhysicsField[],
    globalGravity: Vector2
  ): PhysicsBody[] {
    const subSteps = 8; // High substeps for stability
    const subDt = dt / subSteps;

    const newBodies = bodies.map(b => ({ ...b })); 

    for (let i = 0; i < subSteps; i++) {
      // 1. Apply Forces
      this.applyForces(newBodies, fields, globalGravity);
      
      // 2. Apply Constraint Forces (Springs - Soft Constraints)
      this.applySprings(newBodies, constraints);

      // 3. Integrate Velocity (Symplectic Euler Part 1)
      this.integrateVelocity(newBodies, subDt);

      // 4. Resolve Collisions (Impulse-based with SAT)
      this.resolveCollisions(newBodies, subDt, globalGravity);
      
      // 5. Resolve Hard Constraints (Rod, Pin)
      this.resolveHardConstraints(newBodies, constraints, subDt);

      // 6. Integrate Position (Symplectic Euler Part 2)
      this.integratePosition(newBodies, subDt);
    }

    return newBodies;
  }

  private applyForces(bodies: PhysicsBody[], fields: PhysicsField[], gravity: Vector2) {
    bodies.forEach(body => {
      if (body.inverseMass === 0) return; // Static body

      // Reset force to zero initially or use stored constant force
      body.force = body.constantForce ? { ...body.constantForce } : { x: 0, y: 0 };

      // 1. Global Gravity
      // F = m * g
      const fGravity = Vec2.mul(gravity, body.mass);
      body.force = Vec2.add(body.force, fGravity);

      // 2. Field Forces
      fields.forEach(field => {
        // Simple AABB check for field containment (approximate for rotated bodies)
        const inField = 
          body.position.x >= field.position.x &&
          body.position.x <= field.position.x + field.size.x &&
          body.position.y >= field.position.y &&
          body.position.y <= field.position.y + field.size.y;

        if (inField) {
            if (field.type === FieldType.UNIFORM_ELECTRIC) {
                // F = qE
                const E = field.strength as Vector2;
                const fElectric = Vec2.mul(E, body.charge);
                body.force = Vec2.add(body.force, fElectric);
            } else if (field.type === FieldType.UNIFORM_MAGNETIC) {
                // F = q(v x B) -> Lorentz force
                const B = field.strength as number;
                // v cross B (where B is in Z direction) -> (vy * Bz, -vx * Bz)
                const fLorentz = {
                    x: body.velocity.y * B * body.charge,
                    y: -body.velocity.x * B * body.charge
                };
                body.force = Vec2.add(body.force, fLorentz);
            } else if (field.type === FieldType.AREA_GRAVITY) {
                 const g = field.strength as Vector2;
                 body.force = Vec2.add(body.force, Vec2.mul(g, body.mass));
            }
        }
      });
      
      // 3. Air Drag
      const dragCoeff = 0.005; // Reduced drag for better vacuum simulation defaults
      const rotDrag = 0.05;
      body.force = Vec2.sub(body.force, Vec2.mul(body.velocity, dragCoeff * body.mass));
      body.angularVelocity *= (1 - rotDrag * 0.01); 
    });
  }

  private applySprings(bodies: PhysicsBody[], constraints: Constraint[]) {
    constraints.forEach(c => {
      if (c.type !== ConstraintType.SPRING) return;

      const bodyA = bodies.find(b => b.id === c.bodyAId);
      const bodyB = bodies.find(b => b.id === c.bodyBId);

      if (!bodyA || !bodyB) return;

      // Transform local anchors to world space
      const rA = Vec2.rotate(c.localAnchorA, bodyA.angle);
      const rB = Vec2.rotate(c.localAnchorB, bodyB.angle);
      const pA = Vec2.add(bodyA.position, rA);
      const pB = Vec2.add(bodyB.position, rB);

      const delta = Vec2.sub(pB, pA);
      const dist = Vec2.mag(delta);
      const direction = dist === 0 ? {x:0, y:0} : Vec2.div(delta, dist);

      // Hooke's Law: F = -k * (currentLength - restLength)
      const forceMag = -c.stiffness * (dist - c.length);
      
      // Damping: Fd = -b * v_rel_projected
      const vA_point = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, rA));
      const vB_point = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, rB));
      const relVel = Vec2.sub(vB_point, vA_point);
      
      const dampingForce = -c.damping * Vec2.dot(relVel, direction);

      const totalForce = Vec2.mul(direction, forceMag + dampingForce);

      // Apply forces and torques
      if (bodyA.inverseMass !== 0) {
        bodyA.force = Vec2.sub(bodyA.force, totalForce);
        const torque = Vec2.cross(rA, Vec2.mul(totalForce, -1)); 
        bodyA.angularVelocity += torque * bodyA.inverseInertia; 
      }
      if (bodyB.inverseMass !== 0) {
        bodyB.force = Vec2.add(bodyB.force, totalForce);
        const torque = Vec2.cross(rB, totalForce);
        bodyB.angularVelocity += torque * bodyB.inverseInertia;
      }
    });
  }

  private integrateVelocity(bodies: PhysicsBody[], dt: number) {
    bodies.forEach(body => {
      if (body.inverseMass === 0) return;
      
      // v += (F/m) * dt
      const acc = Vec2.mul(body.force, body.inverseMass);
      body.acceleration = acc; 
      body.velocity = Vec2.add(body.velocity, Vec2.mul(acc, dt));
    });
  }

  private integratePosition(bodies: PhysicsBody[], dt: number) {
    bodies.forEach(body => {
      if (body.inverseMass === 0) return;
      
      // x += v * dt
      body.position = Vec2.add(body.position, Vec2.mul(body.velocity, dt));
      
      // Rotation 
      body.angle += body.angularVelocity * dt;
    });
  }

  // --- COLLISION RESOLUTION (SAT) ---
  private resolveCollisions(bodies: PhysicsBody[], dt: number, gravity: Vector2) {
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const bodyA = bodies[i];
        const bodyB = bodies[j];
        
        if (bodyA.inverseMass === 0 && bodyB.inverseMass === 0) continue;

        const manifold = this.detectCollision(bodyA, bodyB);

        if (manifold) {
            const { normal, depth } = manifold;
            
            // 1. Positional Correction (Baumgarte Stabilization)
            // Reduced percent to prevent jitter/explosions (Repulsion Bug Fix)
            const percent = 0.2; 
            const slop = 0.05; // increased slop slightly
            const correctionMag = Math.max(depth - slop, 0) / (bodyA.inverseMass + bodyB.inverseMass) * percent;
            const correction = Vec2.mul(normal, correctionMag);
            
            if (bodyA.inverseMass !== 0) bodyA.position = Vec2.sub(bodyA.position, Vec2.mul(correction, bodyA.inverseMass));
            if (bodyB.inverseMass !== 0) bodyB.position = Vec2.add(bodyB.position, Vec2.mul(correction, bodyB.inverseMass));

            // 2. Impulse Resolution
            const vA = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, Vec2.zero())); // Simplified for CoM
            const vB = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, Vec2.zero()));

            const relVel = Vec2.sub(vB, vA);
            const velAlongNormal = Vec2.dot(relVel, normal);

            if (velAlongNormal > 0) continue; // Moving away

            let e = Math.min(bodyA.restitution, bodyB.restitution);

            // Resting Contact Threshold (prevent micro-bouncing)
            const gravityMag = Vec2.mag(gravity);
            // Higher threshold for stability
            if (Math.abs(velAlongNormal) < gravityMag * dt * 8) {
                e = 0;
            }

            let jScalar = -(1 + e) * velAlongNormal;
            jScalar /= (bodyA.inverseMass + bodyB.inverseMass);

            const impulse = Vec2.mul(normal, jScalar);
            
            if (bodyA.inverseMass !== 0) bodyA.velocity = Vec2.sub(bodyA.velocity, Vec2.mul(impulse, bodyA.inverseMass));
            if (bodyB.inverseMass !== 0) bodyB.velocity = Vec2.add(bodyB.velocity, Vec2.mul(impulse, bodyB.inverseMass));
            
            // 3. Friction
            let tangent = Vec2.sub(relVel, Vec2.mul(normal, velAlongNormal));
            const tangentMag = Vec2.mag(tangent);
            if (tangentMag > 0.0001) {
                tangent = Vec2.div(tangent, tangentMag);
            } else {
                tangent = {x:0, y:0};
            }

            // Handle Conveyor Belts
            let surfaceVelA = 0;
            if (bodyA.surfaceSpeed) surfaceVelA = bodyA.surfaceSpeed;

            // Recalculate jt for friction
            let jTangent = -Vec2.dot(relVel, tangent) / (bodyA.inverseMass + bodyB.inverseMass);
            const mu = Math.sqrt(bodyA.friction * bodyB.friction);
            
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
                 
                 // Apply dynamic friction clamp
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
        }
      }
    }
  }

  // General Collision Detection Dispatcher
  private detectCollision(bodyA: PhysicsBody, bodyB: PhysicsBody): { normal: Vector2, depth: number } | null {
      // Circle vs Circle
      if (bodyA.type === BodyType.CIRCLE && bodyB.type === BodyType.CIRCLE) {
          const distSq = Vec2.dist(bodyA.position, bodyB.position);
          const rSum = (bodyA.radius || 10) + (bodyB.radius || 10);
          
          // Use squared distance for check to avoid sqrt
          const distRaw = distSq; 
          const minDist = rSum * rSum;
          
          if (distRaw < minDist) { 
              const dist = Math.sqrt(distRaw);
              // Normal points A -> B
              const normal = dist === 0 ? {x:0, y:1} : Vec2.div(Vec2.sub(bodyB.position, bodyA.position), dist);
              return {
                  normal: normal,
                  depth: rSum - dist
              };
          }
          return null;
      }

      // Arcs (Static tracks)
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

      // Polygon/Box vs Polygon/Box/Circle
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

  // --- SAT HELPERS ---
  
  private satPolygonPolygon(a: PhysicsBody, b: PhysicsBody): { normal: Vector2, depth: number } | null {
      const vertsA = this.getWorldVertices(a);
      const vertsB = this.getWorldVertices(b);

      let normal = Vec2.zero();
      let depth = Number.MAX_VALUE;

      // Test axes of A
      for (let i = 0; i < vertsA.length; i++) {
          const v1 = vertsA[i];
          const v2 = vertsA[(i + 1) % vertsA.length];
          const edge = Vec2.sub(v2, v1);
          const axis = Vec2.normalize(Vec2.perp(edge));

          const [minA, maxA] = this.projectVertices(vertsA, axis);
          const [minB, maxB] = this.projectVertices(vertsB, axis);

          if (maxA < minB || maxB < minA) return null; // Separated

          const overlap = Math.min(maxA - minB, maxB - minA);
          if (overlap < depth) {
              depth = overlap;
              normal = axis;
          }
      }

      // Test axes of B
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

      // Ensure normal points from A to B
      const dir = Vec2.sub(b.position, a.position);
      if (Vec2.dot(dir, normal) < 0) {
          normal = Vec2.mul(normal, -1);
      }

      return { normal, depth };
  }

  private satPolygonCircle(poly: PhysicsBody, circle: PhysicsBody): { normal: Vector2, depth: number } | null {
      const verts = this.getWorldVertices(poly);
      let normal = Vec2.zero();
      let depth = Number.MAX_VALUE;

      // 1. Test polygon axes
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

      // 2. Test axis from closest vertex to circle center
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

      // Ensure normal points from Poly to Circle
      const dir = Vec2.sub(circle.position, poly.position);
      if (Vec2.dot(dir, normal) < 0) {
          normal = Vec2.mul(normal, -1);
      }

      return { normal, depth };
  }

  private getWorldVertices(body: PhysicsBody): Vector2[] {
      // If explicit vertices exist (Polygon)
      if (body.vertices) {
          return body.vertices.map(v => Vec2.transform(v, body.position, body.angle));
      }
      // Construct Box vertices on fly if not present (Legacy BOX support)
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

  // Check Circle vs Plane
  private checkCirclePlane(circle: PhysicsBody, plane: PhysicsBody, flipNormal: boolean): {normal: Vector2, depth: number} | null {
    if (!plane.normal) return null;
    let distToPlane = Vec2.dot(Vec2.sub(circle.position, plane.position), plane.normal);
    const r = circle.radius || 10;
    
    // Check if behind plane or inside radius
    if (distToPlane < r) {
        let n = plane.normal;
        if (flipNormal) n = Vec2.mul(n, -1);
        return { normal: n, depth: r - distToPlane };
    }
    return null;
  }
  
  // Check Polygon/Box vs Plane
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

  // Check Circle vs Arc
  private checkCircleArc(circle: PhysicsBody, arc: PhysicsBody): {normal: Vector2, depth: number} | null {
      const rArc = arc.radius || 100;
      const rCirc = circle.radius || 20;

      // Vector from Arc center to Circle center
      const d = Vec2.sub(circle.position, arc.position);
      const dist = Vec2.mag(d);
      
      // Arc logic: It's a thin shell at 'radius'.
      if (Math.abs(dist - rArc) < rCirc) {
          // Check Angle
          const angle = Math.atan2(d.y, d.x); // -PI to PI
          
          // Basic containment check
          const start = arc.arcStartAngle || 0;
          const end = arc.arcEndAngle || Math.PI;
          
          // Check if 'a' is inside [start, end]
          // Normalize all to [0, 2PI]
          const norm = (rad: number) => (rad % (2*Math.PI) + 2*Math.PI) % (2*Math.PI);
          const nA = norm(angle);
          const nStart = norm(start);
          const nEnd = norm(end);

          let inArc = false;
          if (nStart < nEnd) {
              inArc = nA >= nStart && nA <= nEnd;
          } else {
              // Crosses 0 boundary (e.g. 350 to 10)
              inArc = nA >= nStart || nA <= nEnd;
          }

          if (inArc) {
              const normal = Vec2.normalize(d); // Points away from Arc Center
              const pen = rCirc - Math.abs(dist - rArc);
              
              let n = normal;
              if (dist < rArc) {
                  // Inside the arc, normal points INWARD (Center)
                  n = Vec2.mul(normal, -1); 
              }
              
              return { normal: n, depth: pen };
          }
      }
      return null;
  }

  // Check Polygon vs Arc (Simplified check for vertices against the arc shell)
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

      // Check each vertex
      for (const v of verts) {
          const d = Vec2.sub(v, arc.position);
          const dist = Vec2.mag(d);
          
          // Tolerance for "thickness" of arc track - say 5 pixels half-width
          const halfWidth = 5; 
          
          if (Math.abs(dist - rArc) < halfWidth) {
              const angle = Math.atan2(d.y, d.x);
              const nA = norm(angle);

              let inArc = false;
              if (nStart < nEnd) {
                  inArc = nA >= nStart && nA <= nEnd;
              } else {
                  inArc = nA >= nStart || nA <= nEnd;
              }

              if (inArc) {
                  const depth = halfWidth - Math.abs(dist - rArc);
                  if (depth > maxDepth) {
                      maxDepth = depth;
                      let n = Vec2.normalize(d);
                      if (dist < rArc) {
                          n = Vec2.mul(n, -1); // Points inward
                      }
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
    constraints.forEach(c => {
        if (c.type === ConstraintType.SPRING) return;

        const bodyA = bodies.find(b => b.id === c.bodyAId);
        const bodyB = bodies.find(b => b.id === c.bodyBId);
        if (!bodyA || !bodyB) return;

        // Calculate world positions of anchors
        const rA = Vec2.rotate(c.localAnchorA, bodyA.angle);
        const rB = Vec2.rotate(c.localAnchorB, bodyB.angle);
        const pA = Vec2.add(bodyA.position, rA);
        const pB = Vec2.add(bodyB.position, rB);

        if (c.type === ConstraintType.ROD) {
            // Distance Constraint
            const delta = Vec2.sub(pB, pA);
            const dist = Vec2.mag(delta);
            const diff = (dist - c.length);
            
            if (Math.abs(diff) > 0.001) {
                const normal = Vec2.normalize(delta);
                // Positional correction
                const correction = diff * 0.5; // Split error
                const move = Vec2.mul(normal, correction);

                if (bodyA.inverseMass !== 0) bodyA.position = Vec2.add(bodyA.position, move);
                if (bodyB.inverseMass !== 0) bodyB.position = Vec2.sub(bodyB.position, move);

                // Remove velocity along normal (Constraint stabilization)
                const vA = Vec2.add(bodyA.velocity, Vec2.crossNV(bodyA.angularVelocity, rA));
                const vB = Vec2.add(bodyB.velocity, Vec2.crossNV(bodyB.angularVelocity, rB));
                const relVel = Vec2.sub(vB, vA);
                const vn = Vec2.dot(relVel, normal);
                
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
        } else if (c.type === ConstraintType.PIN) {
             const delta = Vec2.sub(pB, pA);
             
             // Positional correction
             const move = Vec2.mul(delta, 0.5);
             if (bodyA.inverseMass !== 0) bodyA.position = Vec2.add(bodyA.position, move);
             if (bodyB.inverseMass !== 0) bodyB.position = Vec2.sub(bodyB.position, move);
             
             // Kill relative velocity
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
    });
  }
}
