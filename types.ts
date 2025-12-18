

// Fundamental Vector Type
export interface Vector2 {
  x: number;
  y: number;
}

// Shape Types
export enum BodyType {
  CIRCLE = 'CIRCLE',
  BOX = 'BOX',
  POLYGON = 'POLYGON',
  PLANE = 'PLANE', // Infinite floor/wall
  ARC = 'ARC'      // Curved track
}

// Physics Body Properties
export interface PhysicsBody {
  id: string;
  type: BodyType;
  position: Vector2;
  velocity: Vector2;
  acceleration: Vector2;
  force: Vector2; // Net force
  
  // Detailed Force Analysis (visual only, calculated per frame)
  forceComponents?: Record<string, Vector2>; 

  // Research & Control
  constantForce: Vector2; // Continuous applied force
  showTrajectory: boolean;
  trail: Vector2[]; // Array of past positions

  // Analysis Visuals
  showVelocity?: boolean;
  showAcceleration?: boolean;
  showForce?: boolean;
  showCharge?: boolean;

  // Physical properties
  mass: number; // 0 for infinite mass (static)
  inverseMass: number;
  restitution: number; // Bounciness (0-1)
  friction: number; // 0-1
  charge: number; // Coulomb
  angle: number; // Radians
  angularVelocity: number;
  momentInertia: number;
  inverseInertia: number;

  // Geometry
  radius?: number; // For Circle and Arc
  width?: number; // For Box
  height?: number; // For Box
  vertices?: Vector2[]; // For Polygon (Local coordinates relative to position)
  normal?: Vector2; // For Plane
  
  // Special Flags
  isParticle?: boolean; // If true, renders as a point, ignores radius for visual
  isHollow?: boolean; // If true, treated as a chain of edges (for containers/tracks)
  
  // Arc Specific
  arcStartAngle?: number; // Radians
  arcEndAngle?: number;   // Radians

  // Conveyor Specific
  surfaceSpeed?: number; // Speed along local X axis (tangent)

  // Visuals
  color: string;
  selected: boolean;
}

// Constraints
export enum ConstraintType {
  SPRING = 'SPRING',
  ROD = 'ROD',
  PIN = 'PIN', // Revolute Joint
  ROPE = 'ROPE' // Inelastic Rope (Slack allowed)
}

export interface Constraint {
  id: string;
  type: ConstraintType;
  bodyAId: string;
  bodyBId: string;
  
  // Anchors in local body coordinates
  localAnchorA: Vector2; 
  localAnchorB: Vector2;

  length: number; // Rest length (for Spring/Rod)
  stiffness: number; // k (for Spring)
  damping: number;   // (for Spring)
}

// Fields (Gravity, Electric, Magnetic)
export enum FieldType {
  UNIFORM_ELECTRIC = 'UNIFORM_ELECTRIC',
  UNIFORM_MAGNETIC = 'UNIFORM_MAGNETIC',
  AREA_GRAVITY = 'AREA_GRAVITY',
  CUSTOM = 'CUSTOM' // Custom math function
}

export enum FieldShape {
  BOX = 'BOX',
  CIRCLE = 'CIRCLE',
  POLYGON = 'POLYGON'
}

export interface PhysicsField {
  id: string;
  type: FieldType;
  shape: FieldShape;
  position: Vector2; // Top-left for Box, Center for Circle
  size: Vector2; // Width/Height for Box
  radius?: number; // For Circle
  vertices?: Vector2[]; // For Polygon (World Space)
  strength: Vector2 | number; // Vector for E-field/Gravity, Scalar for B-field
  
  // Custom equations strings (e.g. "Math.sin(x)")
  equations?: {
    ex: string;
    ey: string;
  };
  customType?: 'ELECTRIC' | 'MAGNETIC'; // Modality of the custom field

  visible: boolean;
}

export interface SimulationState {
  name?: string; // Canvas Name
  bodies: PhysicsBody[];
  constraints: Constraint[];
  fields: PhysicsField[];
  time: number;
  paused: boolean;
  gravity: Vector2;
  enableCoulomb: boolean; // Global switch for Coulomb
  enableUniversalGravity: boolean; // Global switch for G*m1*m2/r^2
  enableAirResistance: boolean; // Global switch for Drag
  selectedBodyId: string | null;
  selectedFieldId: string | null;
  selectedConstraintId: string | null;
  trackingBodyId?: string | null; // Camera tracking
  camera: {
    x: number;
    y: number;
    zoom: number;
  };
}

export enum ChartType {
  VELOCITY_TIME = 'v-t',
  POS_X_TIME = 'x-t',
  POS_Y_TIME = 'y-t',
  KINETIC_TIME = 'ke-t',
  TRAJECTORY = 'y-x'
}