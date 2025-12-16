
import { BodyType, PhysicsBody, Vector2 } from '../types';
import { Vec2 } from './vectorMath';

export class CollisionSystem {
  
  // General Collision Detection Dispatcher
  public detectCollision(bodyA: PhysicsBody, bodyB: PhysicsBody): { normal: Vector2, depth: number } | null {
      // HOLLOW BODY or LINE HANDLING
      if (bodyA.isHollow || bodyB.isHollow || bodyA.type === BodyType.LINE || bodyB.type === BodyType.LINE) {
          const hollow = (bodyA.isHollow || bodyA.type === BodyType.LINE) ? bodyA : bodyB;
          const solid = (bodyA.isHollow || bodyA.type === BodyType.LINE) ? bodyB : bodyA;
          
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
          
          const rA = bodyA.isParticle ? 1 : (bodyA.radius || 10);
          const rB = bodyB.isParticle ? 1 : (bodyB.radius || 10);
          
          const rSum = rA + rB;
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
          // If not closed, skip last segment (LINE is open, Arc is open, Polygon can be closed)
          if (i === chainVerts.length - 1 && (chain.isHollow || chain.type === BodyType.LINE || chain.type === BodyType.ARC)) break; 
          
          const v1 = chainVerts[i];
          const v2 = chainVerts[(i + 1) % chainVerts.length];

          // Treat each edge as a "capsule" segment 
          const thickness = 2; // Matches visual line width

          if (body.type === BodyType.CIRCLE) {
              const r = body.isParticle ? 1 : (body.radius || 10);
              const res = this.checkCircleSegment(body.position, r, v1, v2, thickness);
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
          let normal: Vector2;
          if (dist > 0) {
              normal = Vec2.normalize(distVec);
          } else {
              normal = Vec2.perp(Vec2.normalize(segment));
          }
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

      const circleRadius = circle.isParticle ? 1 : (circle.radius || 10);

      let normal = Vec2.zero();
      let depth = Number.MAX_VALUE;

      for (let i = 0; i < verts.length; i++) {
          const v1 = verts[i];
          const v2 = verts[(i + 1) % verts.length];
          const edge = Vec2.sub(v2, v1);
          const axis = Vec2.normalize(Vec2.perp(edge));

          const [minP, maxP] = this.projectVertices(verts, axis);
          const minC = Vec2.dot(circle.position, axis) - circleRadius;
          const maxC = Vec2.dot(circle.position, axis) + circleRadius;

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
      const minC = Vec2.dot(circle.position, axis) - circleRadius;
      const maxC = Vec2.dot(circle.position, axis) + circleRadius;

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

  public getWorldVertices(body: PhysicsBody): Vector2[] {
      if (body.type === BodyType.LINE) {
          const halfLen = (body.length || 100) / 2;
          const localVerts = [
              { x: -halfLen, y: 0 },
              { x: halfLen, y: 0 }
          ];
          return localVerts.map(v => Vec2.transform(v, body.position, body.angle));
      }
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
    const r = circle.isParticle ? 1 : (circle.radius || 10);
    
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
      const rCirc = circle.isParticle ? 0.5 : (circle.radius || 20); 

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
          if (nStart < nEnd) {
              inArc = nA >= nStart && nA <= nEnd;
          } else {
              inArc = nA >= nStart || nA <= nEnd;
          }

          if (inArc) {
              const normal = Vec2.normalize(d); 
              let depth = 0;
              
              if (dist < rArc) {
                  const penetration = (dist + rCirc) - rArc;
                  if (penetration > 0) {
                      return { normal: Vec2.mul(normal, -1), depth: penetration };
                  }
              } else {
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
      
      const THICKNESS = 2;

      for (const v of verts) {
          const d = Vec2.sub(v, arc.position);
          const dist = Vec2.mag(d);
          
          if (Math.abs(dist - rArc) < 20) { 
              const angle = Math.atan2(d.y, d.x);
              const nA = norm(angle);

              let inArc = false;
              if (nStart < nEnd) {
                  inArc = nA >= nStart && nA <= nEnd;
              } else {
                  inArc = nA >= nStart || nA <= nEnd;
              }

              if (inArc) {
                  let n = Vec2.normalize(d);
                  let depth = 0;
                  
                  if (dist < rArc) {
                      depth = THICKNESS - (rArc - dist); 
                      if (depth > 0) {
                          n = Vec2.mul(n, -1); 
                      }
                  } else {
                      depth = THICKNESS - (dist - rArc);
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
}
