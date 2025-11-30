import { Vector2 } from '../types';

export const Vec2 = {
  add: (v1: Vector2, v2: Vector2): Vector2 => ({ x: v1.x + v2.x, y: v1.y + v2.y }),
  sub: (v1: Vector2, v2: Vector2): Vector2 => ({ x: v1.x - v2.x, y: v1.y - v2.y }),
  mul: (v: Vector2, s: number): Vector2 => ({ x: v.x * s, y: v.y * s }),
  div: (v: Vector2, s: number): Vector2 => ({ x: v.x / s, y: v.y / s }),
  dot: (v1: Vector2, v2: Vector2): number => v1.x * v2.x + v1.y * v2.y,
  cross: (v1: Vector2, v2: Vector2): number => v1.x * v2.y - v1.y * v2.x,
  
  // Vector cross Number (Result is Vector) - effectively (v.y, -v.x) * s
  crossVN: (v: Vector2, s: number): Vector2 => ({ x: v.y * s, y: -v.x * s }),
  // Number cross Vector (Result is Vector) - effectively (-v.y, v.x) * s
  crossNV: (s: number, v: Vector2): Vector2 => ({ x: -v.y * s, y: v.x * s }),

  magSq: (v: Vector2): number => v.x * v.x + v.y * v.y,
  mag: (v: Vector2): number => Math.sqrt(v.x * v.x + v.y * v.y),
  normalize: (v: Vector2): Vector2 => {
    const m = Math.sqrt(v.x * v.x + v.y * v.y);
    return m === 0 ? { x: 0, y: 0 } : { x: v.x / m, y: v.y / m };
  },
  dist: (v1: Vector2, v2: Vector2): number => Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2)),
  rotate: (v: Vector2, angle: number): Vector2 => ({
    x: v.x * Math.cos(angle) - v.y * Math.sin(angle),
    y: v.x * Math.sin(angle) + v.y * Math.cos(angle)
  }),
  // Transform local point to world space
  transform: (v: Vector2, pos: Vector2, angle: number): Vector2 => {
    const rot = {
        x: v.x * Math.cos(angle) - v.y * Math.sin(angle),
        y: v.x * Math.sin(angle) + v.y * Math.cos(angle)
    };
    return { x: rot.x + pos.x, y: rot.y + pos.y };
  },
  perp: (v: Vector2): Vector2 => ({ x: -v.y, y: v.x }), // Perpendicular (90 deg CCW)
  zero: (): Vector2 => ({ x: 0, y: 0 })
};