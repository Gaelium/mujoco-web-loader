/**
 * Shared simulation types — the vendored type spine (OPEN-SOURCE-SPLIT.md).
 *
 * This is the trimmed subset of the app's `types/simulation.ts` that the
 * loader actually consumes: geometry primitives, scene-graph descriptors,
 * per-frame state, the URDF injection configs (including the cross-boundary
 * `URDFActuatorConfig`), trajectory configs, and the `SimResult` value type.
 * Type bodies are copied verbatim from the source of truth so app types and
 * package types stay structurally identical.
 */

// ---------------------------------------------------------------------------
// Geometry primitives (MuJoCo conventions unless noted)
// ---------------------------------------------------------------------------

/** Position vector [x, y, z] in metres. */
export type Vec3 = readonly [number, number, number]

/** MuJoCo quaternion [w, x, y, z]. */
export type MjQuat = readonly [number, number, number, number]

// ---------------------------------------------------------------------------
// Robot / scene-graph types
// ---------------------------------------------------------------------------

/** A single rigid body in the MuJoCo scene graph. */
export interface BodyState {
  /** MuJoCo body index. */
  readonly id: number
  /** Human-readable name from the model XML / URDF. */
  readonly name: string
  /** World-frame position [x, y, z]. */
  readonly position: Vec3
  /** World-frame orientation as MuJoCo quaternion [w, x, y, z]. */
  readonly quaternion: MjQuat
}

/** Joint type subset supported by the platform. */
export type JointType = 'hinge' | 'slide' | 'ball' | 'free'

/**
 * Static description of a single MuJoCo actuator (sent once at model load).
 *
 * Surfaces the control semantics a sim-to-real layer needs: which joint the
 * actuator drives, whether it's position/velocity/torque controlled, and the
 * control range. For injected URDF position actuators (see
 * `physics/actuatorInjection.ts`) `ctrlRange` equals the joint's limit range,
 * which is also what the real servo's min/max ticks will map to.
 */
export interface ActuatorDescriptor {
  /** MuJoCo actuator index (0..nu-1), matching the `data.ctrl` slot. */
  readonly id: number
  /** Actuator name from the model XML. */
  readonly name: string
  /** Name of the joint this actuator transmits to (empty if non-joint transmission). */
  readonly jointName: string
  /** Control semantics, inferred from the actuator's gain/bias type. */
  readonly type: 'position' | 'velocity' | 'motor' | 'other'
  /**
   * Control range [lo, hi]. For position actuators this is the target-angle
   * range (radians); equal values ([0, 0]) mean unbounded.
   */
  readonly ctrlRange: readonly [number, number]
}

/** Runtime state of a single joint. */
export interface JointState {
  /** MuJoCo joint index. */
  readonly id: number
  readonly name: string
  readonly type: JointType
  /** Generalised position (scalar for hinge/slide, 4-vec for ball, 7-vec for free). */
  readonly qpos: readonly number[]
  /** Generalised velocity. */
  readonly qvel: readonly number[]
  /** Applied control signal (NaN if no actuator). */
  readonly ctrl: number
}

/** Describes a visual mesh attached to a body (for the render layer). */
export interface VisualGeom {
  readonly bodyId: number
  /** Geom type string matching MuJoCo's mjtGeom enum names. */
  readonly type: 'plane' | 'sphere' | 'capsule' | 'ellipsoid' | 'cylinder' | 'box' | 'mesh'
  /** Size parameters (interpretation depends on type). */
  readonly size: Vec3
  /** Local position offset relative to parent body. */
  readonly localPos: Vec3
  /** Local orientation offset relative to parent body. */
  readonly localQuat: MjQuat
  /** RGBA colour [0-1]. */
  readonly rgba: readonly [number, number, number, number]
  /** Mesh file path in VFS (only for type === 'mesh'). */
  readonly meshFile?: string
}

/** Sensor reading from MuJoCo. */
export interface SensorReading {
  readonly id: number
  readonly name: string
  /** Sensor type string (e.g. 'accelerometer', 'gyro', 'force', 'torque', 'touch'). */
  readonly type: string
  /** Raw data — length depends on sensor type. */
  readonly data: readonly number[]
  /**
   * MuJoCo body index the sensor is attached to, or -1 if unresolved.
   * For site-based sensors (gyro, accelerometer, framequat) this is the
   * body that owns the site. The render layer uses this to anchor 3D
   * visualizations (arrows, rays) to the correct body transform.
   */
  readonly bodyId: number
  /**
   * Site index this sensor references, or undefined for non-site sensors.
   * Lidar (rangefinder) visualization uses this to look up the site's world
   * transform in SimState.sites without needing to ship sensor_objid arrays
   * across the worker boundary.
   */
  readonly siteId?: number
}

/**
 * World-frame transform of a MuJoCo site.
 *
 * Sites are massless reference frames parented to bodies. Many sensors
 * (gyro, accel, framequat, rangefinder) are anchored to sites rather than
 * bodies because they need a specific orientation and offset within the
 * parent body. Rangefinders cast along the site's +z axis, so the
 * visualization needs both position and direction per site.
 */
export interface SiteState {
  /** MuJoCo site index. */
  readonly id: number
  /** Human-readable name from the model XML. */
  readonly name: string
  /** Body index this site is attached to (from model.site_bodyid). */
  readonly bodyId: number
  /** World-frame position [x, y, z]. */
  readonly position: Vec3
  /**
   * World-frame +z axis of the site (unit vector).
   * This is the direction a rangefinder ray is cast.
   */
  readonly direction: Vec3
}

export interface CameraDescriptor {
  /** MuJoCo camera index (0..ncam-1). */
  readonly id: number
  /** Camera name from MJCF `<camera name="…">`. */
  readonly name: string
  /** Body index the camera is attached to (`model.cam_bodyid[i]`). */
  readonly bodyId: number
  /** Vertical field of view in degrees (`model.cam_fovy[i]`). */
  readonly fovy: number
  /**
   * Near clipping plane in metres. **Not** read from MuJoCo (MJCF's
   * `<camera>` element doesn't accept `znear`). The simulation store
   * applies app-level overrides by camera name before handing the
   * descriptor list to the render layer; if no override exists the render
   * layer falls back to its `CameraConfig` default.
   */
  readonly near?: number
  /** Far clipping plane in metres. Same provenance as `near`. */
  readonly far?: number
}

/**
 * Per-frame world-space pose of a simulated camera.
 *
 * MuJoCo computes `data.cam_xpos`/`cam_xmat` each step as the camera's pose
 * in world coordinates (Z-up). The rotation is kept as a raw row-major 3×3
 * matrix because that's MuJoCo's native representation — converting to a
 * quaternion on the main thread via `THREE.Matrix4.set` avoids re-deriving
 * the quaternion convention twice and matches how `site_xmat` is handled.
 *
 * MuJoCo cameras look along -Z in their local frame (OpenGL convention),
 * the same as `THREE.PerspectiveCamera`, so no extra orientation flip is
 * needed when the camera is mounted under the codebase's `mujoco_frame`
 * (Z-up) parent group.
 */
export interface CameraState {
  /** MuJoCo camera index (matches `CameraDescriptor.id`). */
  readonly id: number
  /** World-frame position [x, y, z] from `data.cam_xpos[i*3..i*3+2]`. */
  readonly position: Vec3
  /**
   * World-frame orientation as row-major 3×3 from `data.cam_xmat[i*9..i*9+8]`.
   * Index layout: `[m00,m01,m02, m10,m11,m12, m20,m21,m22]`.
   */
  readonly xmat: readonly [
    number, number, number,
    number, number, number,
    number, number, number,
  ]
}

/** A single contact point reported by MuJoCo. */
export interface ContactPoint {
  /** World-frame position of contact. */
  readonly position: Vec3
  /** Contact normal (points from geom2 → geom1). */
  readonly normal: Vec3
  /** Normal force magnitude. */
  readonly force: number
  /** Name of first geom. */
  readonly geom1: string
  /** Name of second geom. */
  readonly geom2: string
}

/** Metadata about a loaded robot model (sent once after load). */
export interface RobotDescriptor {
  /** Unique id assigned at load time. */
  readonly id: string
  /** Display name derived from URDF <robot name="…">. */
  readonly name: string
  /** Number of bodies (including worldbody). */
  readonly bodyCount: number
  /** Number of joints. */
  readonly jointCount: number
  /** Number of actuators. */
  readonly actuatorCount: number
  /** Body descriptors (id + name) for the render layer to set up meshes. */
  readonly bodies: readonly Pick<BodyState, 'id' | 'name'>[]
  /** Joint descriptors. */
  readonly joints: readonly Pick<JointState, 'id' | 'name' | 'type'>[]
  /**
   * Actuator descriptors (control semantics + joint mapping). Empty for models
   * with no actuators. The index of each entry is its `data.ctrl` slot.
   */
  readonly actuators: readonly ActuatorDescriptor[]
  /** Visual geoms for the render layer. */
  readonly visuals: readonly VisualGeom[]
  /**
   * Simulated cameras declared in the model. Empty when the model has no
   * `<camera>` tags or the WASM bindings don't expose `ncam`.
   */
  readonly cameras: readonly CameraDescriptor[]
}

// ---------------------------------------------------------------------------
// Simulation state (per-frame snapshot)
// ---------------------------------------------------------------------------

export interface SimState {
  /** Simulation time in seconds. */
  readonly time: number
  /** Wall-clock timestamp (performance.now()) when this frame was produced. */
  readonly wallTime: number
  /** Current timestep (dt) in seconds. */
  readonly timestep: number
  /** Whether the sim is paused. */
  readonly paused: boolean
  /** Per-body transforms for the current frame. */
  readonly bodies: readonly BodyState[]
  /** Per-joint state for the current frame. */
  readonly joints: readonly JointState[]
  /** Sensor readings for the current frame. */
  readonly sensors: readonly SensorReading[]
  /** Active contacts for the current frame. */
  readonly contacts: readonly ContactPoint[]
  /**
   * World-frame site transforms for the current frame.
   * Absent or empty when the model has no sites or the WASM bindings don't
   * expose site_xpos/site_xmat — consumers should treat missing as "no data".
   */
  readonly sites?: readonly SiteState[]
  /**
   * World-frame pose of each simulated camera. Absent or empty when the
   * model has no `<camera>` tags or the WASM bindings don't expose
   * `data.cam_xpos`/`cam_xmat`.
   */
  readonly cameras?: readonly CameraState[]
}

// ---------------------------------------------------------------------------
// URDF injection configs (the two-pass load)
// ---------------------------------------------------------------------------

/** Sensor configuration for URDF uploads. Defaults: IMU on, lidar off. */
export interface URDFSensorConfig {
  /** IMU on the root link (gyro + accelerometer + framequat). Default: true. */
  readonly imu?: boolean
  /** Horizontal lidar ring on the root link. Default: false. */
  readonly lidar?: boolean
  /** Lidar ray count when enabled. Default: 36 (matches the bundled Go2 example). */
  readonly lidarRays?: number
  /** Lidar cutoff range in metres when enabled. Default: 5. */
  readonly lidarRange?: number
}

/**
 * Actuator configuration for URDF uploads. Drives `physics/actuatorInjection.ts`.
 * Defaults mirror the bundled `trs_so_arm100/so_arm100.xml`.
 */
export interface URDFActuatorConfig {
  /**
   * `'position'` (default) injects a PD position actuator per movable joint so
   * the controller commands joint angles (1:1 with a real servo's
   * Goal_Position). `'none'` leaves the URDF actuator-less (legacy torque/force
   * control via `qfrc_applied`).
   */
  readonly mode?: 'position' | 'none'
  /** Position gain (kp) for every actuator unless overridden. Default: 50. */
  readonly defaultKp?: number
  /** Damping ratio of the position servo. Default: 1 (critically damped). */
  readonly defaultDampratio?: number
  /** Actuator force/torque clamp [lo, hi]. Default: [-3.5, 3.5]. */
  readonly defaultForcerange?: readonly [number, number]
  /** Joint armature (reflected rotor inertia). Default: 0.1. */
  readonly defaultArmature?: number
  /** Joint dry friction loss. Default: 0.1. */
  readonly defaultFrictionloss?: number
  /**
   * Initial joint-space pose (radians, one per movable joint in ascending
   * joint-id order). Applied to qpos and the position targets at load/reset so
   * the arm starts in a designed rest pose instead of all-zeros.
   */
  readonly homeQpos?: readonly number[]
  /** Per-joint tuning overrides keyed by joint name. */
  readonly perJoint?: Readonly<
    Record<
      string,
      {
        readonly kp?: number
        readonly forcerange?: readonly [number, number]
        readonly range?: readonly [number, number]
      }
    >
  >
}

// ---------------------------------------------------------------------------
// Model identity
// ---------------------------------------------------------------------------

/** Compiled-model identity for the exporter's recompile-equivalence gate. */
export interface ModelSignature {
  readonly nq: number
  readonly nv: number
  readonly nu: number
  /** Actuator names in ctrl order. */
  readonly actuatorNames: readonly string[]
  readonly timestep: number
}

// ---------------------------------------------------------------------------
// Scripted trajectories
// ---------------------------------------------------------------------------

/**
 * M4 G3 — one key pose of a scripted joint-space trajectory. Values are
 * position-actuator targets (radians), one per actuator in ctrl order.
 */
export interface TrajectoryKeyPose {
  readonly ctrl: readonly number[]
  /** Per-joint speed limit toward this pose, rad/s (overrides the default). */
  readonly maxSpeed?: number
  /** Hold at this pose after reaching it, seconds (servo settle / grasp dwell). */
  readonly dwell?: number
}

/**
 * M4 G3 — scripted key-pose trajectory, executed against `data.ctrl` with
 * linear joint-space interpolation under per-joint speed limits (the
 * profile's ramp discipline). The COMMAND is rate-limited; a pose is
 * "reached" when the command arrives — dwells must cover servo settling
 * (the honest bound on the physical pose is the actuator itself).
 */
export interface TrajectoryConfig {
  readonly id: string
  readonly poses: readonly TrajectoryKeyPose[]
  /** Default per-joint speed limit, rad/s. */
  readonly maxSpeed: number
}

// ---------------------------------------------------------------------------
// Result type for async operations
// ---------------------------------------------------------------------------

/** Typed result returned by loader/engine operations. */
export type SimResult<T> =
  | { readonly ok: true; readonly value: T }
  | { readonly ok: false; readonly error: string; readonly detail?: string }
