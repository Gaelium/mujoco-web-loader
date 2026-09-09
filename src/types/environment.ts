/**
 * Environment configuration types — vendored spine subset (OPEN-SOURCE-SPLIT.md).
 *
 * An environment defines the physical world surrounding the robot:
 * gravity, ground surface, lighting hints, and static scene objects.
 * Copied verbatim from the app's `types/environment.ts` minus the
 * app-catalog preset-id union.
 */

import type { Vec3, MjQuat } from './simulation'

// ---------------------------------------------------------------------------
// Physics parameters
// ---------------------------------------------------------------------------

/** Tunable physics parameters exposed to the user. */
export interface PhysicsParams {
  /** Gravity vector in m/s^2 (default [0, 0, -9.81]). */
  readonly gravity: Vec3
  /** Integration timestep in seconds (default 0.002). */
  readonly timestep: number
  /** Number of constraint solver iterations (default 50). */
  readonly iterations: number
  /** Global coefficient of friction (default 1.0). */
  readonly friction: number
}

/** Sensible defaults for PhysicsParams. */
export const DEFAULT_PHYSICS_PARAMS: PhysicsParams = {
  gravity: [0, 0, -9.81],
  timestep: 0.002,
  iterations: 50,
  friction: 1.0,
}

// ---------------------------------------------------------------------------
// Ground / surface
// ---------------------------------------------------------------------------

/** Ground plane appearance. */
export type GroundType = 'grid' | 'solid' | 'checker' | 'none'

export interface GroundConfig {
  readonly type: GroundType
  /** Ground plane size in metres (half-extents). */
  readonly size: number
  /** Primary colour RGBA [0-1]. */
  readonly color1: readonly [number, number, number, number]
  /** Secondary colour for checkerboard patterns. */
  readonly color2: readonly [number, number, number, number]
}

// ---------------------------------------------------------------------------
// Static scene objects (obstacles, ramps, etc.)
// ---------------------------------------------------------------------------

export type ScenePrimitiveShape = 'box' | 'sphere' | 'cylinder' | 'capsule' | 'mesh'

/** A static (non-robot) object placed in the environment. */
export interface ScenePrimitive {
  readonly id: string
  readonly name: string
  readonly shape: ScenePrimitiveShape
  /** Size params: box=[hx,hy,hz], sphere=[r,0,0], cylinder=[r,h,0], capsule=[r,h,0], mesh=[hx,hy,hz] (bounding box). */
  readonly size: Vec3
  readonly position: Vec3
  readonly quaternion: MjQuat
  /** RGBA colour [0-1]. */
  readonly rgba: readonly [number, number, number, number]
  /** Mass in kg (0 = static / fixed). */
  readonly mass: number
  /** Reference to an imported mesh stored by the host app (only for shape === 'mesh'). */
  readonly meshId?: string
}

// ---------------------------------------------------------------------------
// Lighting hints (passed to the render layer, not to MuJoCo)
// ---------------------------------------------------------------------------

export interface LightingConfig {
  /** Ambient light intensity [0-1]. */
  readonly ambientIntensity: number
  /** Directional light intensity [0-1]. */
  readonly directionalIntensity: number
  /** Direction the main light shines FROM. */
  readonly directionalPosition: Vec3
  /** Whether to cast shadows. */
  readonly shadows: boolean
}

// ---------------------------------------------------------------------------
// Full environment config
// ---------------------------------------------------------------------------

/** Complete environment configuration. */
export interface EnvironmentConfig {
  /** Unique preset identifier (e.g. 'empty', 'tabletop', 'outdoor'). */
  readonly id: string
  /** Human-readable display name. */
  readonly name: string
  /** Short description for UI tooltip. */
  readonly description: string
  readonly physics: PhysicsParams
  readonly ground: GroundConfig
  readonly lighting: LightingConfig
  /** Static objects pre-placed in the scene. */
  readonly primitives: readonly ScenePrimitive[]
}
