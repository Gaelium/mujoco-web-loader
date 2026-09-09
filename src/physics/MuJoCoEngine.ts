/**
 * Typed wrapper around the raw MuJoCo WASM module.
 *
 * This class owns the MuJoCo lifecycle: loading the WASM binary,
 * compiling models from XML, stepping the simulation, and reading
 * state. It runs exclusively inside the physics Web Worker — never
 * import this file on the main thread.
 */

import type {
  Vec3,
  MjQuat,
  BodyState,
  JointState,
  JointType,
  SensorReading,
  SiteState,
  CameraDescriptor,
  CameraState,
  ContactPoint,
  SimState,
  VisualGeom,
  RobotDescriptor,
  ActuatorDescriptor,
} from '../types/simulation'

// ---------------------------------------------------------------------------
// Internal type aliases for the raw MuJoCo WASM API
// ---------------------------------------------------------------------------

/**
 * Subset of the MuJoCo WASM module API that MuJoCoEngine depends on.
 * Defined here so the rest of the codebase doesn't import mujoco-js directly
 * and so tests can supply a mock that satisfies this shape.
 */
export interface MjModule {
  FS: {
    writeFile(path: string, data: string | Uint8Array): void
    /** Read a binary file from the VFS. */
    readFile(path: string): Uint8Array
    mkdir(path: string): void
    readdir(path: string): string[]
    unlink(path: string): void
  }
  MjModel: {
    /** Official @mujoco/mujoco static method — takes a VFS path. */
    mj_loadXML?(path: string): MjModelInstance
    /** Explicit VFS path loader (same semantics as mj_loadXML). */
    from_xml_path?(path: string): MjModelInstance
    /** Load from raw XML string content (no VFS path resolution). */
    from_xml_string?(xml: string): MjModelInstance
    /** Legacy mujoco-js loader. */
    loadFromXML?(path: string): MjModelInstance
    /** Load a pre-compiled .mjb binary from a VFS path. */
    mj_loadModel?(path: string): MjModelInstance
    /** Load a pre-compiled .mjb binary from a VFS path (alternative name). */
    from_binary_path?(path: string): MjModelInstance
    /** Load a pre-compiled .mjb binary from a VFS path (alternative name). */
    loadBinary?(path: string): MjModelInstance
  }
  /** Save a compiled model to a VFS file in MuJoCo binary (.mjb) format. */
  mj_saveModel?(model: MjModelInstance, path: string, buffer: null, bufferSize: number): void
  MjData: {
    new (model: MjModelInstance): MjDataInstance
  }
  mj_step(model: MjModelInstance, data: MjDataInstance): void
  mj_resetData(model: MjModelInstance, data: MjDataInstance): void
  mj_forward(model: MjModelInstance, data: MjDataInstance): void
  mj_id2name(model: MjModelInstance, objType: number, id: number): string
  mj_version(): number
  mjtObj: {
    mjOBJ_BODY: { value: number }
    mjOBJ_JOINT: { value: number }
    mjOBJ_GEOM: { value: number }
    mjOBJ_MESH: { value: number }
    mjOBJ_SITE?: { value: number }
    mjOBJ_CAMERA?: { value: number }
    mjOBJ_EQUALITY?: { value: number }
    mjOBJ_SENSOR: { value: number }
    mjOBJ_ACTUATOR: { value: number }
  }
  mjtEq?: {
    mjEQ_CONNECT: { value: number }
    mjEQ_WELD: { value: number }
    mjEQ_JOINT: { value: number }
    mjEQ_TENDON: { value: number }
  }
  mjtTrn?: {
    mjTRN_JOINT?: { value: number }
  }
  mjtJoint: {
    mjJNT_FREE: { value: number }
    mjJNT_BALL: { value: number }
    mjJNT_SLIDE: { value: number }
    mjJNT_HINGE: { value: number }
  }
  mjtGeom: {
    mjGEOM_PLANE: { value: number }
    mjGEOM_SPHERE: { value: number }
    mjGEOM_CAPSULE: { value: number }
    mjGEOM_ELLIPSOID: { value: number }
    mjGEOM_CYLINDER: { value: number }
    mjGEOM_BOX: { value: number }
    mjGEOM_MESH: { value: number }
  }
}

/** Instance of a compiled MuJoCo model (subset used by the engine). */
export interface MjModelInstance {
  nbody: number
  njnt: number
  ngeom: number
  /** Number of actuators (nu in old mujoco-js, na in official @mujoco/mujoco). */
  nu?: number
  na?: number
  nsensor: number
  /** Embind cleanup — call to free WASM heap memory. */
  delete?(): void
  /** Save the compiled model to a VFS path (.mjb binary). Some bindings expose this as an instance method. */
  saveModel?(path: string): void
  /** Save the compiled model — alternative instance-method name. */
  save?(path: string): void
  nsensordata: number
  nq: number
  nv: number
  /** Number of equality constraints defined in the model. */
  neq?: number
  /**
   * Equality constraint type per slot (mjtEq enum values).
   * 0 = connect, 1 = weld, 2 = joint, 3 = tendon, 4 = distance.
   */
  readonly eq_type?: Int32Array
  /**
   * Flat equality data (neq * mjNEQDATA). For weld constraints the slot
   * layout is [anchor1_x, anchor1_y, anchor1_z, relpose_px, relpose_py,
   * relpose_pz, relpose_qw, relpose_qx, relpose_qy, relpose_qz, torquescale].
   */
  readonly eq_data?: Float64Array
  /** Initial active flag per equality (before runtime overrides). */
  readonly eq_active0?: Uint8Array
  /** Number of keyframes defined in the model. */
  nkey?: number
  /** Keyframe qpos values (nkey * nq flat array). */
  readonly key_qpos?: Float64Array
  /** Keyframe ctrl values (nkey * nu flat array). */
  readonly key_ctrl?: Float64Array
  opt: { timestep: number; gravity: Float64Array | { get(i: number): number } }
  readonly qpos0: Float64Array
  readonly jnt_type: Int32Array
  readonly jnt_qposadr: Int32Array
  readonly jnt_dofadr: Int32Array
  readonly jnt_bodyid: Int32Array
  /** Whether each joint is limited (njnt bytes). May be absent on older bindings. */
  readonly jnt_limited?: Uint8Array
  /** Joint limit ranges (njnt * 2 flat: [min, max] per joint). */
  readonly jnt_range?: Float64Array
  readonly geom_type: Int32Array
  readonly geom_bodyid: Int32Array
  readonly geom_size: Float64Array
  readonly geom_pos: Float64Array
  readonly geom_quat: Float64Array
  readonly geom_rgba: Float32Array
  readonly geom_dataid: Int32Array
  /** Geom visualization group (0-5). Groups 0-2 are visual, 3+ collision-only. */
  readonly geom_group?: Int32Array
  readonly name_eqadr?: Int32Array
  readonly sensor_type: Int32Array
  readonly sensor_adr: Int32Array
  readonly sensor_dim: Int32Array
  /**
   * MuJoCo mjtObj enum per sensor — indicates whether sensor_objid references
   * a site, body, geom, joint, etc. Site-based IMU sensors (gyro, accelerometer,
   * framequat) store a site index here.
   */
  readonly sensor_objtype: Int32Array
  /**
   * Object index per sensor — interpreted according to sensor_objtype.
   * For site-based sensors this is an index into site_bodyid.
   */
  readonly sensor_objid: Int32Array
  /** Body index that each site is attached to (length nsite). */
  readonly site_bodyid: Int32Array
  /** Number of sites in the model (available on MuJoCo 3.x Embind surfaces). */
  nsite?: number
  /** Site names index array. May be absent on older bindings. */
  readonly name_siteadr?: Int32Array
  /** Number of cameras declared in the model (MuJoCo 3.x). */
  ncam?: number
  /** Body index each camera is attached to (length ncam). */
  readonly cam_bodyid?: Int32Array
  /**
   * Local camera position relative to parent body (ncam * 3 flat).
   * **Not** readonly: interactive camera placement writes into this array
   * via `MuJoCoEngine.setCameraLocalPose` and then calls `mj_forward`.
   */
  cam_pos?: Float64Array
  /**
   * Local camera orientation quaternion [w,x,y,z] (ncam * 4 flat).
   * **Not** readonly — see `cam_pos`.
   */
  cam_quat?: Float64Array
  /** Vertical field of view in degrees, one per camera (length ncam). */
  readonly cam_fovy?: Float64Array
  /** Camera names index array. */
  readonly name_camadr?: Int32Array
  readonly name_bodyadr: Int32Array
  readonly name_jntadr: Int32Array
  readonly name_geomadr: Int32Array
  readonly name_sensoradr: Int32Array
  readonly names: Uint8Array
  // Actuator introspection (used for sim-to-real control-semantics mapping).
  /** Transmission type per actuator (mjtTrn; 0 = joint). Length nu. */
  readonly actuator_trntype?: Int32Array
  /** Transmission target ids per actuator (nu * 2; slot 0 is the joint id for joint transmission). */
  readonly actuator_trnid?: Int32Array
  /** Control range per actuator (nu * 2 flat: [lo, hi]). */
  readonly actuator_ctrlrange?: Float64Array
  /** Gain type per actuator (mjtGain). Length nu. */
  readonly actuator_gaintype?: Int32Array
  /** Bias type per actuator (mjtBias; 0 = none → open-loop torque/motor). Length nu. */
  readonly actuator_biastype?: Int32Array
}

/** Instance of MuJoCo simulation data (subset used by the engine). */
export interface MjDataInstance {
  /** Embind cleanup — call to free WASM heap memory. */
  delete?(): void
  time: number
  ncon: number
  readonly qpos: Float64Array
  readonly qvel: Float64Array
  readonly ctrl: Float64Array
  readonly qfrc_applied: Float64Array
  readonly xpos: Float64Array
  readonly xquat: Float64Array
  readonly sensordata: Float64Array
  /**
   * World-space site positions (nsite × 3 flat). Optional because older
   * WASM bindings may not expose it — callers must guard.
   */
  readonly site_xpos?: Float64Array
  /**
   * World-space site orientation matrices (nsite × 9, row-major 3×3).
   * +z axis of site i in world frame = [site_xmat[9i+2], site_xmat[9i+5],
   * site_xmat[9i+8]] (the third column).
   */
  readonly site_xmat?: Float64Array
  /** World-space camera positions (ncam × 3 flat). Optional — older bindings may omit. */
  readonly cam_xpos?: Float64Array
  /** World-space camera orientation matrices (ncam × 9, row-major 3×3). */
  readonly cam_xmat?: Float64Array
  /**
   * Per-equality active flag (byte). MuJoCo 3.x moved this from mjModel to
   * mjData so constraints can be toggled at runtime without recompilation.
   * Size: neq bytes.
   */
  readonly eq_active?: Uint8Array
  readonly contact: { size(): number; get(i: number): MjContactInstance | undefined }
}

interface MjContactInstance {
  readonly pos: Float64Array | { get(i: number): number }
  readonly frame: Float64Array | { get(i: number): number }
  geom1: number
  geom2: number
  /** Embind cleanup — must be called to free the C++ wrapper allocated by `data.contact.get(i)`. */
  delete?(): void
}

// ---------------------------------------------------------------------------
// Joint type mapping
// ---------------------------------------------------------------------------

const JOINT_TYPE_MAP: Record<number, JointType> = {
  0: 'free',
  1: 'ball',
  2: 'slide',
  3: 'hinge',
}

/** Number of qpos components per joint type. */
const JOINT_QPOS_SIZE: Record<JointType, number> = {
  free: 7,
  ball: 4,
  slide: 1,
  hinge: 1,
}

/** Number of qvel (dof) components per joint type. */
const JOINT_QVEL_SIZE: Record<JointType, number> = {
  free: 6,
  ball: 3,
  slide: 1,
  hinge: 1,
}

// ---------------------------------------------------------------------------
// Geom type mapping
// ---------------------------------------------------------------------------

const GEOM_TYPE_MAP: Record<number, VisualGeom['type']> = {
  0: 'plane',
  2: 'sphere',
  3: 'capsule',
  4: 'ellipsoid',
  5: 'cylinder',
  6: 'box',
  7: 'mesh',
}

// ---------------------------------------------------------------------------
// MuJoCoEngine
// ---------------------------------------------------------------------------

export class MuJoCoEngine {
  private mj: MjModule
  private model: MjModelInstance | null = null
  private data: MjDataInstance | null = null
  private bodyNames: string[] = []
  private jointNames: string[] = []
  private geomNames: string[] = []
  private sensorNames: string[] = []
  private siteNames: string[] = []
  private cameraNames: string[] = []

  constructor(mj: MjModule) {
    this.mj = mj
  }

  /** @returns MuJoCo version number from the WASM module. */
  getVersion(): number {
    return this.mj.mj_version()
  }

  /**
   * Return the raw MjModelInstance for callers that need to pass it to
   * WASM module functions (e.g. `mj_saveLastXML`). Most callers should
   * use the typed helpers on this class instead.
   *
   * @throws Error if no model is loaded
   */
  getRawModel(): MjModelInstance {
    if (!this.model) throw new Error('No model loaded')
    return this.model
  }

  /**
   * Write a file to MuJoCo's Emscripten virtual filesystem.
   * @param path - VFS path (e.g. '/model.xml')
   * @param data - File content as string or binary
   */
  writeToVFS(path: string, data: string | Uint8Array): void {
    this.mj.FS.writeFile(path, data)
  }

  /**
   * Ensure a directory exists in the VFS (creates it if missing).
   * @param path - Directory path
   */
  ensureVFSDir(path: string): void {
    try {
      this.mj.FS.readdir(path)
    } catch {
      this.mj.FS.mkdir(path)
    }
  }

  /**
   * List the contents of a VFS directory.
   * @param path - Directory path
   * @returns Array of filenames, or empty array if the directory doesn't exist
   */
  listVFSDir(path: string): string[] {
    try {
      return this.mj.FS.readdir(path)
    } catch {
      return []
    }
  }

  /**
   * Load and compile a model from an XML string (URDF or MJCF).
   * Assets must already be written to the VFS before calling this.
   * @param xml - Raw XML content
   * @param filename - VFS path to write the XML to (default '/model.xml')
   * @returns RobotDescriptor with model metadata
   * @throws Error if MuJoCo compilation fails
   */
  loadModel(xml: string, filename: string = '/model.xml'): RobotDescriptor {
    // Dispose previous model/data to prevent WASM heap leaks (Embind objects)
    this.data?.delete?.()
    this.model?.delete?.()

    // Load the model. Write to VFS first, then use mj_loadXML.
    // Note: from_xml_string is broken in @mujoco/mujoco 3.6.1 WASM
    // (hangs on valid XML). Use VFS-based loaders instead.
    this.mj.FS.writeFile(filename, xml)

    let model: MjModelInstance
    let loaderUsed: string

    if (typeof this.mj.MjModel.mj_loadXML === 'function') {
      loaderUsed = 'mj_loadXML'
      console.log(`[MuJoCoEngine] Using mj_loadXML for: ${filename}`)
      model = this.mj.MjModel.mj_loadXML(filename)
    } else if (typeof this.mj.MjModel.from_xml_path === 'function') {
      loaderUsed = 'from_xml_path'
      console.log(`[MuJoCoEngine] Using from_xml_path for: ${filename}`)
      model = this.mj.MjModel.from_xml_path(filename)
    } else if (typeof this.mj.MjModel.loadFromXML === 'function') {
      loaderUsed = 'loadFromXML'
      console.log(`[MuJoCoEngine] Using loadFromXML for: ${filename}`)
      model = this.mj.MjModel.loadFromXML(filename)
      } else {
      const available = Object.getOwnPropertyNames(this.mj.MjModel).join(', ')
      throw new Error(`No model loader found on MjModel. Available: ${available}`)
    }
    console.log(`[MuJoCoEngine] ${loaderUsed} returned, nbody=${model.nbody}`)
    return this.installModel(model)
  }

  /**
   * Save the currently loaded model as a MuJoCo binary (.mjb).
   *
   * Probes for whichever save API the WASM bindings expose, in priority
   * order: module-level `mj_saveModel`, instance method `saveModel`,
   * instance method `save`. Returns the binary bytes, or null if no
   * variant is callable on the current WASM build.
   *
   * Important: on the current `@mujoco/mujoco` package, `mj_saveModel`
   * is exposed by Embind but its glue calls `writeI53ToI64` — an
   * Emscripten runtime helper that isn't in EXPORTED_RUNTIME_METHODS.
   * Invoking `mj_saveModel` aborts the entire WASM runtime, so we
   * detect the missing export at runtime and skip the call. The
   * companion `binaryCache.ts` therefore stays dormant on this build;
   * once upstream ships the helper, save will start working without
   * any code change here.
   *
   * @returns Binary bytes, or null if save is unsupported on this build
   */
  saveBinary(): Uint8Array | null {
    if (!this.model) return null

    // CRITICAL: on the current @mujoco/mujoco WASM build, `mj.mj_saveModel`
    // is exposed as an Embind lazy-resolve getter whose glue calls
    // `writeI53ToI64`. Just *reading* `mj.mj_saveModel` (e.g. via `typeof`)
    // invokes that getter, which aborts the entire WASM runtime when the
    // helper isn't in EXPORTED_RUNTIME_METHODS.
    //
    // We therefore gate the ENTIRE function on writeI53ToI64 being exported.
    // No property access on `mj.mj_saveModel`, `model.saveModel`, etc.
    // happens until we know the runtime helpers exist.
    //
    // Note: `writeI53ToI64` itself is *also* a poisoned getter when missing
    // — Emscripten installs an abort-fn under `Object.defineProperty` for
    // every entry in EXPORTED_RUNTIME_METHODS that wasn't actually built in.
    // So we cannot use `typeof mj.writeI53ToI64` here — that would itself
    // trigger the abort. Use `getOwnPropertyDescriptor` to inspect the
    // property without invoking it; real exports have `.value` set,
    // poisoned exports have `.get` set.
    const writeI53Desc = Object.getOwnPropertyDescriptor(this.mj, 'writeI53ToI64')
    if (!writeI53Desc || typeof writeI53Desc.value !== 'function') {
      if (!MuJoCoEngine.warnedSaveUnavailable) {
        MuJoCoEngine.warnedSaveUnavailable = true
        console.info(
          '[MuJoCoEngine] Binary save disabled — @mujoco/mujoco lacks writeI53ToI64 in EXPORTED_RUNTIME_METHODS. Compile cache is inert.',
        )
      }
      return null
    }

    const path = '/__simulo_save.mjb'

    const variants: Array<readonly [string, () => void]> = []
    if (typeof this.mj.mj_saveModel === 'function') {
      const fn = this.mj.mj_saveModel
      const m = this.model
      variants.push(['mj.mj_saveModel(model, path, null, 0)', () => fn(m, path, null, 0)])
    }
    if (typeof this.model.saveModel === 'function') {
      const m = this.model
      const fn = this.model.saveModel
      variants.push(['model.saveModel(path)', () => fn.call(m, path)])
    }
    if (typeof this.model.save === 'function') {
      const m = this.model
      const fn = this.model.save
      variants.push(['model.save(path)', () => fn.call(m, path)])
    }

    if (variants.length === 0) {
      console.info('[MuJoCoEngine] No binary save API present on this build.')
      return null
    }

    for (const [name, run] of variants) {
      try {
        run()
        const bytes = this.mj.FS.readFile(path)
        try { this.mj.FS.unlink(path) } catch { /* ignore */ }
        console.log(`[MuJoCoEngine] saveBinary OK via ${name}: ${bytes.byteLength} bytes`)
        // Detach from VFS-owned memory by copying.
        return new Uint8Array(bytes)
      } catch (err) {
        console.warn(`[MuJoCoEngine] saveBinary variant ${name} failed:`, err)
      }
    }
    console.warn('[MuJoCoEngine] saveBinary: no working API variant found')
    return null
  }

  /** One-shot console hint flag for the dormant-cache message. */
  private static warnedSaveUnavailable = false

  /**
   * Load a model from pre-compiled MuJoCo binary (.mjb) data.
   *
   * Skips XML compilation entirely — typically 10-50× faster than
   * `loadModel` for non-trivial robots. Probes for the available
   * binary loader at runtime and throws if none is exposed.
   *
   * @param binary - Pre-compiled .mjb bytes
   * @param filename - VFS path to write to before loading (default '/model.mjb')
   * @returns RobotDescriptor with model metadata
   * @throws Error if no binary loader is available or compilation fails
   */
  loadBinary(binary: Uint8Array, filename: string = '/model.mjb'): RobotDescriptor {
    this.data?.delete?.()
    this.model?.delete?.()

    this.mj.FS.writeFile(filename, binary)

    let model: MjModelInstance
    if (typeof this.mj.MjModel.mj_loadModel === 'function') {
      model = this.mj.MjModel.mj_loadModel(filename)
    } else if (typeof this.mj.MjModel.from_binary_path === 'function') {
      model = this.mj.MjModel.from_binary_path(filename)
    } else if (typeof this.mj.MjModel.loadBinary === 'function') {
      model = this.mj.MjModel.loadBinary(filename)
    } else {
      const available = Object.getOwnPropertyNames(this.mj.MjModel).join(', ')
      throw new Error(`No binary model loader found on MjModel. Available: ${available}`)
    }
    return this.installModel(model)
  }

  /**
   * Wire a freshly-loaded MjModel into the engine: create MjData, read
   * names, run forward kinematics, and return the descriptor. Shared by
   * `loadModel` (XML path) and `loadBinary` (cached path).
   */
  private installModel(model: MjModelInstance): RobotDescriptor {
    const data = new this.mj.MjData(model)

    this.model = model
    this.data = data

    this.bodyNames = this.readNames(model.name_bodyadr, model.nbody, this.mj.mjtObj.mjOBJ_BODY.value)
    this.jointNames = this.readNames(model.name_jntadr, model.njnt, this.mj.mjtObj.mjOBJ_JOINT.value)
    this.geomNames = this.readGeomNames(model)
    this.sensorNames = this.readNames(model.name_sensoradr, model.nsensor, this.mj.mjtObj.mjOBJ_SENSOR.value)

    // Site names — only read if the bindings expose nsite + mjOBJ_SITE.
    const siteTypeVal = this.mj.mjtObj.mjOBJ_SITE?.value
    const nsite = model.nsite ?? 0
    if (siteTypeVal !== undefined && nsite > 0) {
      const addrs = model.name_siteadr ?? new Int32Array(nsite)
      this.siteNames = this.readNames(addrs, nsite, siteTypeVal)
    } else {
      this.siteNames = []
    }

    // Camera names — only read if the bindings expose ncam + mjOBJ_CAMERA.
    const camTypeVal = this.mj.mjtObj.mjOBJ_CAMERA?.value
    const ncam = model.ncam ?? 0
    if (camTypeVal !== undefined && ncam > 0) {
      const addrs = model.name_camadr ?? new Int32Array(ncam)
      this.cameraNames = this.readNames(addrs, ncam, camTypeVal)
    } else {
      this.cameraNames = []
    }

    // Compute forward kinematics so xpos/xquat are populated
    this.mj.mj_forward(model, data)

    return this.buildDescriptor('model')
  }

  /**
   * Advance the simulation by one timestep.
   * @throws Error if no model is loaded
   */
  step(): void {
    const { model, data } = this.requireLoaded()
    this.mj.mj_step(model, data)
  }

  /**
   * Advance the simulation by multiple timesteps.
   * @param count - Number of steps to take
   */
  stepN(count: number): void {
    for (let i = 0; i < count; i++) {
      this.step()
    }
  }

  /**
   * Reset simulation data to the initial state (qpos0, zero velocities).
   * @throws Error if no model is loaded
   */
  reset(): void {
    const { model, data } = this.requireLoaded()
    this.mj.mj_resetData(model, data)
    this.mj.mj_forward(model, data)
  }

  /**
   * Check whether the loaded model defines any keyframes.
   * @returns true if nkey > 0
   */
  hasKeyframes(): boolean {
    if (!this.model) return false
    return (this.model.nkey ?? 0) > 0
  }

  /**
   * Reset simulation state to a named keyframe by index.
   * Copies keyframe qpos (and ctrl if available) into the simulation data,
   * then runs forward kinematics to update body transforms.
   * @param keyId - Keyframe index (0-based)
   */
  resetToKeyframe(keyId: number): void {
    const { model, data } = this.requireLoaded()
    const nkey = model.nkey ?? 0
    if (keyId < 0 || keyId >= nkey) return
    if (!model.key_qpos) return

    const nq = model.nq
    const offset = keyId * nq
    for (let i = 0; i < nq; i++) {
      data.qpos[i] = model.key_qpos[offset + i]
    }

    // Also apply keyframe ctrl values if available
    if (model.key_ctrl) {
      const nu = model.nu ?? model.na ?? 0
      const ctrlOffset = keyId * nu
      for (let i = 0; i < nu; i++) {
        data.ctrl[i] = model.key_ctrl[ctrlOffset + i]
      }
    }

    // Restore qpos0 for environment primitive joints. The keyframe was
    // defined before environment injection, so MuJoCo zero-pads the extra
    // DOFs. Restoring from qpos0 puts primitives back at their configured
    // positions (set via body pos/quat in the MJCF).
    this.restoreQpos0ForBodies('_simulo_prim_')

    this.mj.mj_forward(model, data)
  }

  /**
   * Restore qpos from qpos0 for all joints whose parent body name
   * starts with the given prefix. Used to undo keyframe zeroing of
   * environment primitive joints that weren't in the original keyframe.
   */
  private restoreQpos0ForBodies(prefix: string): void {
    const { model, data } = this.requireLoaded()
    const bodyType = this.mj.mjtObj.mjOBJ_BODY.value
    for (let j = 0; j < model.njnt; j++) {
      const bodyId = model.jnt_bodyid[j]
      const bodyName = this.mj.mj_id2name(model, bodyType, bodyId)
      if (!bodyName.startsWith(prefix)) continue
      const addr = model.jnt_qposadr[j]
      const type = JOINT_TYPE_MAP[model.jnt_type[j]] ?? 'hinge'
      const size = JOINT_QPOS_SIZE[type]
      for (let k = 0; k < size; k++) {
        data.qpos[addr + k] = model.qpos0[addr + k]
      }
    }
  }

  /**
   * Set qpos values starting at a given address.
   * @param startAddr - Starting index in the qpos array
   * @param values - Values to write
   */
  setQpos(startAddr: number, values: readonly number[]): void {
    const { data } = this.requireLoaded()
    for (let i = 0; i < values.length; i++) {
      data.qpos[startAddr + i] = values[i]
    }
  }

  /**
   * Read a single qpos value.
   * @param addr - Index in the qpos array
   * @returns The value at that index
   */
  getQposValue(addr: number): number {
    const { data } = this.requireLoaded()
    return data.qpos[addr]
  }

  /**
   * Zero the velocity DOFs of the first free joint.
   * Free joints have 6 DOFs (3 translational + 3 rotational).
   */
  zeroFreeJointVel(): void {
    const { model, data } = this.requireLoaded()
    for (let i = 0; i < model.njnt; i++) {
      if (model.jnt_type[i] === 0) { // mjJNT_FREE = 0
        const dofAddr = model.jnt_dofadr[i]
        for (let d = 0; d < 6; d++) {
          data.qvel[dofAddr + d] = 0
        }
        return
      }
    }
  }

  /**
   * Compute forward kinematics (update xpos/xquat from qpos).
   * Call this after directly modifying qpos to update body transforms.
   */
  forward(): void {
    const { model, data } = this.requireLoaded()
    this.mj.mj_forward(model, data)
  }

  // -------------------------------------------------------------------------
  // Equality constraints (used for weld-based fixed-base support)
  // -------------------------------------------------------------------------

  /** Number of equality constraints in the loaded model. */
  getEqualityCount(): number {
    const { model } = this.requireLoaded()
    return model.neq ?? 0
  }

  /**
   * Find an equality constraint by name.
   * @param name - Constraint name from the model XML
   * @returns Index in [0, neq), or -1 if not found
   */
  findEqualityByName(name: string): number {
    const { model } = this.requireLoaded()
    const eqCount = model.neq ?? 0
    if (eqCount === 0) return -1
    const eqObjType = this.mj.mjtObj.mjOBJ_EQUALITY?.value
    if (eqObjType === undefined) return -1
    for (let i = 0; i < eqCount; i++) {
      const candidate = this.mj.mj_id2name(model, eqObjType, i)
      if (candidate === name) return i
    }
    return -1
  }

  /**
   * Enable or disable an equality constraint at runtime.
   * Requires MuJoCo 3.x which exposes `data.eq_active`.
   * @param index - Equality index
   * @param active - true to enable the constraint, false to disable
   * @returns true if applied, false if runtime toggling isn't available
   */
  setEqualityActive(index: number, active: boolean): boolean {
    const { model, data } = this.requireLoaded()
    const eqCount = model.neq ?? 0
    if (index < 0 || index >= eqCount) return false
    if (!data.eq_active) return false
    data.eq_active[index] = active ? 1 : 0
    this.mj.mj_forward(model, data)
    return true
  }

  /**
   * Update a weld equality constraint's target relative pose.
   * The weld stores its relpose at a fixed offset inside `eq_data`; this
   * function writes (pos, quat) into that slot so the constraint pins
   * the welded body to the desired world pose.
   *
   * @param index - Equality index
   * @param pos - Desired position [x, y, z]
   * @param quat - Desired orientation as MuJoCo quaternion [w, x, y, z]
   * @returns true if applied, false if the model lacks eq_data
   */
  setWeldRelpose(index: number, pos: Vec3, quat: MjQuat): boolean {
    const { model } = this.requireLoaded()
    const eqCount = model.neq ?? 0
    if (index < 0 || index >= eqCount) return false
    if (!model.eq_data || eqCount === 0) return false
    // Each equality slot stores mjNEQDATA floats; infer the stride so this
    // works across MuJoCo versions (mjNEQDATA is 11 in 3.x, historically 7).
    const stride = Math.floor(model.eq_data.length / eqCount)
    if (stride < 10) return false
    const base = index * stride
    // mjEQ_WELD layout: anchor1[3], relpose[7] (pos+quat), torquescale[1].
    model.eq_data[base + 3] = pos[0]
    model.eq_data[base + 4] = pos[1]
    model.eq_data[base + 5] = pos[2]
    model.eq_data[base + 6] = quat[0]
    model.eq_data[base + 7] = quat[1]
    model.eq_data[base + 8] = quat[2]
    model.eq_data[base + 9] = quat[3]
    return true
  }

  /**
   * Get the qpos address for a joint by name.
   * @param jointName - Name of the joint to find
   * @returns qpos offset, or -1 if not found
   */
  getJointQposAddr(jointName: string): number {
    const { model } = this.requireLoaded()
    const idx = this.jointNames.indexOf(jointName)
    if (idx < 0) return -1
    return model.jnt_qposadr[idx]
  }

  /**
   * Get the qpos address of the first free joint in the model.
   * MuJoCo free joints have type 0.
   * @returns qpos offset, or -1 if no free joint exists
   */
  getFirstFreeJointQposAddr(): number {
    const { model } = this.requireLoaded()
    for (let i = 0; i < model.njnt; i++) {
      if (model.jnt_type[i] === 0) { // mjJNT_FREE = 0
        return model.jnt_qposadr[i]
      }
    }
    return -1
  }

  /**
   * Get a joint's limit range by name.
   *
   * @param jointName - Joint name in the model
   * @returns `[min, max]` if the joint is limited, or null if unlimited / not
   *          found / the bindings don't expose `jnt_range`. A `[0, 0]` range
   *          (MuJoCo's "no limit" encoding) is reported as null.
   */
  getJointRange(jointName: string): readonly [number, number] | null {
    const { model } = this.requireLoaded()
    const idx = this.jointNames.indexOf(jointName)
    if (idx < 0) return null
    if (model.jnt_limited && model.jnt_limited[idx] === 0) return null
    const range = model.jnt_range
    if (!range || idx * 2 + 1 >= range.length) return null
    const lo = range[idx * 2]
    const hi = range[idx * 2 + 1]
    if (lo === 0 && hi === 0) return null
    return [lo, hi]
  }

  /**
   * Read static descriptors for every actuator in the model.
   *
   * Resolves each actuator's driven joint (for joint transmissions), control
   * range, and a best-effort control-semantics classification (`position` when
   * the actuator has bias feedback, `motor` for open-loop torque). Returns []
   * when the model has no actuators.
   *
   * @returns Array of ActuatorDescriptor, indexed by `data.ctrl` slot
   */
  getActuatorInfo(): readonly ActuatorDescriptor[] {
    const { model } = this.requireLoaded()
    const nu = model.nu ?? model.na ?? 0
    if (nu === 0) return []
    const jointObj = this.mj.mjtObj.mjOBJ_JOINT.value
    const actObj = this.mj.mjtObj.mjOBJ_ACTUATOR.value
    const trnJoint = this.mj.mjtTrn?.mjTRN_JOINT?.value ?? 0
    const out: ActuatorDescriptor[] = []
    for (let i = 0; i < nu; i++) {
      const name = this.mj.mj_id2name(model, actObj, i)
      let jointName = ''
      const trntype = model.actuator_trntype?.[i]
      if ((trntype === undefined || trntype === trnJoint) && model.actuator_trnid) {
        const jointId = model.actuator_trnid[i * 2]
        if (jointId >= 0) jointName = this.mj.mj_id2name(model, jointObj, jointId)
      }
      const ctrlRange: readonly [number, number] = model.actuator_ctrlrange
        ? [model.actuator_ctrlrange[i * 2], model.actuator_ctrlrange[i * 2 + 1]]
        : [0, 0]
      out.push({ id: i, name, jointName, type: this.classifyActuator(i), ctrlRange })
    }
    return out
  }

  /**
   * Classify an actuator's control semantics from its bias type. A non-zero
   * bias type means internal feedback (position/velocity servo); zero means
   * open-loop force ("motor"). Velocity vs position isn't distinguished here —
   * position is the overwhelmingly common feedback actuator and the only one
   * this platform injects.
   */
  private classifyActuator(index: number): ActuatorDescriptor['type'] {
    const { model } = this.requireLoaded()
    const biastype = model.actuator_biastype?.[index]
    if (biastype === undefined) return 'other'
    return biastype === 0 ? 'motor' : 'position'
  }

  /**
   * Set a control value on a specific actuator.
   * @param index - Actuator index
   * @param value - Control signal value
   */
  setControl(index: number, value: number): void {
    const { data } = this.requireLoaded()
    data.ctrl[index] = value
  }

  /**
   * Set multiple control values at once.
   * @param controls - Sparse map of actuator index to value
   */
  setControls(controls: Readonly<Record<number, number>>): void {
    const { data } = this.requireLoaded()
    for (const [indexStr, value] of Object.entries(controls)) {
      data.ctrl[Number(indexStr)] = value
    }
  }

  /**
   * Apply a generalized force/torque to a specific DOF.
   * This works even without actuators defined in the model.
   * @param dofIndex - Degree-of-freedom index (0-based)
   * @param value - Force/torque value
   */
  setAppliedForce(dofIndex: number, value: number): void {
    const { data } = this.requireLoaded()
    data.qfrc_applied[dofIndex] = value
  }

  /**
   * Apply generalized forces/torques to all DOFs from an array.
   * @param forces - Array of force values, one per DOF
   */
  setAppliedForces(forces: readonly number[]): void {
    const { data, model } = this.requireLoaded()
    const nv = model.nv
    for (let i = 0; i < Math.min(forces.length, nv); i++) {
      data.qfrc_applied[i] = forces[i]
    }
  }

  /**
   * Clear all applied forces (set qfrc_applied to zero).
   */
  clearAppliedForces(): void {
    const { data } = this.requireLoaded()
    data.qfrc_applied.fill(0)
  }

  /**
   * Get the number of degrees of freedom (nv) in the model.
   * @returns Number of DOFs
   */
  getNumDOF(): number {
    const { model } = this.requireLoaded()
    return model.nv
  }

  /**
   * Get the current simulation timestep.
   * @returns timestep in seconds
   */
  getTimestep(): number {
    const { model } = this.requireLoaded()
    return model.opt.timestep
  }

  /**
   * Set the simulation timestep.
   * @param dt - New timestep in seconds
   */
  setTimestep(dt: number): void {
    const { model } = this.requireLoaded()
    model.opt.timestep = dt
  }

  /**
   * Set the gravity vector.
   * @param gravity - [x, y, z] gravity in m/s²
   */
  setGravity(gravity: Vec3): void {
    const { model } = this.requireLoaded()
    const g = model.opt.gravity
    if (g instanceof Float64Array) {
      g[0] = gravity[0]
      g[1] = gravity[1]
      g[2] = gravity[2]
    } else {
      // Accessor-style (some WASM bindings) — not directly writable.
      // Fall back to timestep-only update.
    }
  }

  /**
   * Read the current simulation time.
   * @returns Simulation time in seconds
   */
  getTime(): number {
    const { data } = this.requireLoaded()
    return data.time
  }

  /**
   * Read all body transforms (positions and orientations).
   * @returns Array of BodyState objects
   */
  getBodyStates(): readonly BodyState[] {
    const { model, data } = this.requireLoaded()
    const bodies: BodyState[] = []
    for (let i = 0; i < model.nbody; i++) {
      const pOff = i * 3
      const qOff = i * 4
      bodies.push({
        id: i,
        name: this.bodyNames[i],
        position: [data.xpos[pOff], data.xpos[pOff + 1], data.xpos[pOff + 2]],
        quaternion: [data.xquat[qOff], data.xquat[qOff + 1], data.xquat[qOff + 2], data.xquat[qOff + 3]],
      })
    }
    return bodies
  }

  /**
   * Read all joint states.
   * @returns Array of JointState objects
   */
  getJointStates(): readonly JointState[] {
    const { model, data } = this.requireLoaded()
    const joints: JointState[] = []
    for (let i = 0; i < model.njnt; i++) {
      const type = JOINT_TYPE_MAP[model.jnt_type[i]] ?? 'hinge'
      const qposAdr = model.jnt_qposadr[i]
      const dofAdr = model.jnt_dofadr[i]
      const qposSize = JOINT_QPOS_SIZE[type]
      const qvelSize = JOINT_QVEL_SIZE[type]

      joints.push({
        id: i,
        name: this.jointNames[i],
        type,
        qpos: Array.from(data.qpos.subarray(qposAdr, qposAdr + qposSize)),
        qvel: Array.from(data.qvel.subarray(dofAdr, dofAdr + qvelSize)),
        ctrl: NaN,
      })
    }
    return joints
  }

  /**
   * Read all sensor readings.
   * @returns Array of SensorReading objects
   */
  getSensorReadings(): readonly SensorReading[] {
    const { model, data } = this.requireLoaded()
    const siteTypeVal = this.mj.mjtObj.mjOBJ_SITE?.value
    const sensors: SensorReading[] = []
    for (let i = 0; i < model.nsensor; i++) {
      const adr = model.sensor_adr[i]
      const dim = model.sensor_dim[i]
      const objType = model.sensor_objtype?.[i]
      const objId = model.sensor_objid?.[i]
      const siteId =
        siteTypeVal !== undefined && objType === siteTypeVal && objId !== undefined && objId >= 0
          ? objId
          : undefined
      sensors.push({
        id: i,
        name: this.sensorNames[i],
        type: String(model.sensor_type[i]),
        data: Array.from(data.sensordata.subarray(adr, adr + dim)),
        bodyId: this.getSensorBodyId(i),
        ...(siteId !== undefined ? { siteId } : {}),
      })
    }
    return sensors
  }

  /**
   * Read world-frame transforms for every site in the model.
   *
   * Returns the position and +z axis direction of each site. The lidar
   * visualizer uses these to place rays and hit points; rangefinder sensors
   * cast along the site's +z axis per MuJoCo convention.
   *
   * Returns [] when nsite is zero or when data.site_xpos/site_xmat are
   * not exposed on the current WASM bindings — callers should treat empty
   * as "no site data available".
   *
   * @returns Array of SiteState objects (empty if unavailable)
   */
  getSiteStates(): readonly SiteState[] {
    const { model, data } = this.requireLoaded()
    const nsite = model.nsite ?? 0
    const xpos = data.site_xpos
    const xmat = data.site_xmat
    if (nsite === 0 || !xpos || !xmat) return []

    const siteBodyId = model.site_bodyid
    const sites: SiteState[] = []
    for (let i = 0; i < nsite; i++) {
      // Row-major 3×3: rows are [m00 m01 m02, m10 m11 m12, m20 m21 m22].
      // Local +z axis in world frame = third column = (m02, m12, m22).
      const px = xpos[i * 3]
      const py = xpos[i * 3 + 1]
      const pz = xpos[i * 3 + 2]
      const dx = xmat[i * 9 + 2]
      const dy = xmat[i * 9 + 5]
      const dz = xmat[i * 9 + 8]
      sites.push({
        id: i,
        name: this.siteNames[i] ?? String(i),
        bodyId: siteBodyId && i < siteBodyId.length ? siteBodyId[i] : -1,
        position: [px, py, pz],
        direction: [dx, dy, dz],
      })
    }
    return sites
  }

  /**
   * Read camera descriptors (static metadata) for every camera in the model.
   *
   * Returns [] when the model has no `<camera>` tags or the bindings don't
   * expose `ncam` — callers should treat empty as "no camera data available".
   *
   * @returns Array of CameraDescriptor objects (empty if unavailable)
   */
  getCameraDescriptors(): readonly CameraDescriptor[] {
    const { model } = this.requireLoaded()
    const ncam = model.ncam ?? 0
    if (ncam === 0) return []
    const bodyIds = model.cam_bodyid
    const fovys = model.cam_fovy
    const cameras: CameraDescriptor[] = []
    for (let i = 0; i < ncam; i++) {
      cameras.push({
        id: i,
        name: this.cameraNames[i] ?? String(i),
        bodyId: bodyIds && i < bodyIds.length ? bodyIds[i] : -1,
        fovy: fovys && i < fovys.length ? fovys[i] : 45,
      })
    }
    return cameras
  }

  /**
   * Read world-frame transforms for every camera in the model.
   *
   * MuJoCo populates `data.cam_xpos`/`cam_xmat` during `mj_forward`/`mj_step`,
   * resolving the camera's pose through the body kinematic tree. The matrix
   * is row-major 3×3 in MuJoCo's C ABI; we ship it as-is and let the main
   * thread convert to a `THREE.Matrix4` (which also accepts row-major via
   * `.set()`).
   *
   * Returns [] when ncam is zero or when the WASM bindings don't expose
   * `data.cam_xpos`/`cam_xmat`.
   *
   * @returns Array of CameraState objects (empty if unavailable)
   */
  /**
   * Write a new local pose for a single camera into `model.cam_pos` /
   * `model.cam_quat`. Does **not** call `mj_forward` — the caller decides
   * (typically once at the end of a batch).
   *
   * No-op if `index` is out of range or the bindings don't expose camera
   * arrays.
   *
   * @param index - MuJoCo camera index (0..ncam-1)
   * @param pos - Local position offset relative to the parent body
   * @param quat - Local orientation as MuJoCo quaternion [w, x, y, z]
   */
  setCameraLocalPose(index: number, pos: Vec3, quat: MjQuat): void {
    const { model } = this.requireLoaded()
    const ncam = model.ncam ?? 0
    if (index < 0 || index >= ncam) return
    const camPos = model.cam_pos
    const camQuat = model.cam_quat
    if (!camPos || !camQuat) return
    const p = index * 3
    camPos[p] = pos[0]
    camPos[p + 1] = pos[1]
    camPos[p + 2] = pos[2]
    const q = index * 4
    camQuat[q] = quat[0]
    camQuat[q + 1] = quat[1]
    camQuat[q + 2] = quat[2]
    camQuat[q + 3] = quat[3]
  }

  /** Lookup a camera index by name; returns -1 when not found. */
  findCameraByName(name: string): number {
    for (let i = 0; i < this.cameraNames.length; i++) {
      if (this.cameraNames[i] === name) return i
    }
    return -1
  }

  getCameraStates(): readonly CameraState[] {
    const { model, data } = this.requireLoaded()
    const ncam = model.ncam ?? 0
    const xpos = data.cam_xpos
    const xmat = data.cam_xmat
    if (ncam === 0 || !xpos || !xmat) return []

    const cameras: CameraState[] = []
    for (let i = 0; i < ncam; i++) {
      const p = i * 3
      const m = i * 9
      cameras.push({
        id: i,
        position: [xpos[p], xpos[p + 1], xpos[p + 2]],
        xmat: [
          xmat[m], xmat[m + 1], xmat[m + 2],
          xmat[m + 3], xmat[m + 4], xmat[m + 5],
          xmat[m + 6], xmat[m + 7], xmat[m + 8],
        ],
      })
    }
    return cameras
  }

  /**
   * Resolve the body index a sensor is anchored to.
   *
   * For site-based sensors (gyro, accelerometer, framequat) this follows
   * sensor_objid → site_bodyid. For body-based sensors (framepos on a body,
   * etc.) the objid is the body itself. Returns -1 when the sensor references
   * a non-spatial object (joint, actuator) or the lookup tables are missing —
   * callers should treat -1 as "no world anchor available".
   *
   * @param sensorIndex - Index into model.sensor_type / model.sensor_objid
   * @returns Body index in [0, nbody), or -1 if unresolved
   */
  getSensorBodyId(sensorIndex: number): number {
    const { model } = this.requireLoaded()
    if (sensorIndex < 0 || sensorIndex >= model.nsensor) return -1

    const objType = model.sensor_objtype?.[sensorIndex]
    const objId = model.sensor_objid?.[sensorIndex]
    if (objType === undefined || objId === undefined || objId < 0) return -1

    const bodyTypeVal = this.mj.mjtObj.mjOBJ_BODY.value
    const siteTypeVal = this.mj.mjtObj.mjOBJ_SITE?.value

    if (objType === bodyTypeVal) {
      return objId < model.nbody ? objId : -1
    }
    if (siteTypeVal !== undefined && objType === siteTypeVal && model.site_bodyid) {
      if (objId >= model.site_bodyid.length) return -1
      return model.site_bodyid[objId]
    }
    return -1
  }

  /**
   * Read active contact points.
   *
   * Critical: `data.contact.get(i)` allocates an Embind C++ wrapper on the
   * WASM heap for each call. Without `.delete()`, those wrappers leak at
   * roughly `ncon × frameRate` per second — for humanoids with self-contact
   * this can grow the heap by ~10–50 MB/min and eventually overflow the 2 GB
   * WASM cap with no other symptom. Always wrap reads in try/finally so a
   * read failure mid-iteration can't strand a wrapper.
   *
   * @returns Array of ContactPoint objects
   */
  getContacts(): readonly ContactPoint[] {
    const { data } = this.requireLoaded()
    const contacts: ContactPoint[] = []
    const ncon = data.ncon
    // Cache the container in case `data.contact` itself returns a fresh
    // Embind wrapper on each access; freed in the outer finally below.
    const contactsAccessor = data.contact
    try {
      for (let i = 0; i < ncon; i++) {
        const c = contactsAccessor.get(i)
        if (!c) continue
        try {
          const pos = this.readVec3FromAccessor(c.pos, 0)
          // Contact frame: first 3 values are the normal
          const normal = this.readVec3FromAccessor(c.frame, 0)
          contacts.push({
            position: pos,
            normal,
            force: 0, // Force requires efc_force computation; placeholder for now
            geom1: this.geomNames[c.geom1] ?? String(c.geom1),
            geom2: this.geomNames[c.geom2] ?? String(c.geom2),
          })
        } finally {
          c.delete?.()
        }
      }
    } finally {
      ;(contactsAccessor as { delete?(): void }).delete?.()
    }
    return contacts
  }

  /**
   * Build a complete visual geometry list for the render layer.
   * @returns Array of VisualGeom objects
   */
  getVisualGeoms(): readonly VisualGeom[] {
    const { model } = this.requireLoaded()
    const visuals: VisualGeom[] = []
    for (let i = 0; i < model.ngeom; i++) {
      const typeVal = model.geom_type[i]
      const typeName = GEOM_TYPE_MAP[typeVal]
      if (!typeName) continue // skip unsupported geom types (hfield, sdf, etc.)

      // Skip plane geoms — ground planes are rendered by the environment layer.
      if (typeName === 'plane') continue

      // Skip collision-only geoms (group >= 3). Menagerie models use group 2
      // for visual meshes and group 3 for collision shapes. MuJoCo convention
      // shows groups 0-2 by default. If geom_group is unavailable (older WASM
      // bindings), include all geoms to preserve backward compatibility.
      if (model.geom_group && model.geom_group[i] >= 3) continue

      const sOff = i * 3
      const pOff = i * 3
      const qOff = i * 4
      const rOff = i * 4

      visuals.push({
        bodyId: model.geom_bodyid[i],
        type: typeName,
        size: [model.geom_size[sOff], model.geom_size[sOff + 1], model.geom_size[sOff + 2]],
        localPos: [model.geom_pos[pOff], model.geom_pos[pOff + 1], model.geom_pos[pOff + 2]],
        localQuat: [model.geom_quat[qOff], model.geom_quat[qOff + 1], model.geom_quat[qOff + 2], model.geom_quat[qOff + 3]],
        rgba: [model.geom_rgba[rOff], model.geom_rgba[rOff + 1], model.geom_rgba[rOff + 2], model.geom_rgba[rOff + 3]],
        ...(typeName === 'mesh' && model.geom_dataid[i] >= 0
          ? { meshFile: this.getMeshName(model.geom_dataid[i]) }
          : {}),
      })
    }
    return visuals
  }

  /**
   * Build a full SimState snapshot of the current simulation.
   * @param paused - Whether the sim is currently paused
   * @returns SimState object
   */
  getSimState(paused: boolean): SimState {
    const sites = this.getSiteStates()
    const cameras = this.getCameraStates()
    return {
      time: this.getTime(),
      wallTime: typeof performance !== 'undefined' ? performance.now() : Date.now(),
      timestep: this.getTimestep(),
      paused,
      bodies: this.getBodyStates(),
      joints: this.getJointStates(),
      sensors: this.getSensorReadings(),
      contacts: this.getContacts(),
      ...(sites.length > 0 ? { sites } : {}),
      ...(cameras.length > 0 ? { cameras } : {}),
    }
  }

  /**
   * Get a RobotDescriptor for the currently loaded model.
   * @param id - Unique id to assign
   * @returns RobotDescriptor
   */
  getDescriptor(id: string): RobotDescriptor {
    return this.buildDescriptor(id)
  }

  /**
   * Check whether a model is currently loaded.
   * @returns true if model and data are available
   */
  isLoaded(): boolean {
    return this.model !== null && this.data !== null
  }

  /**
   * Dispose the current model and data, freeing resources.
   */
  dispose(): void {
    this.data?.delete?.()
    this.model?.delete?.()
    this.model = null
    this.data = null
    this.bodyNames = []
    this.jointNames = []
    this.geomNames = []
    this.sensorNames = []
    this.siteNames = []
    this.cameraNames = []
  }

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  private requireLoaded(): { model: MjModelInstance; data: MjDataInstance } {
    if (!this.model || !this.data) {
      throw new Error('No model loaded. Call loadModel() first.')
    }
    return { model: this.model, data: this.data }
  }

  private readNames(_adrArray: Int32Array, count: number, objType: number): string[] {
    const names: string[] = []
    for (let i = 0; i < count; i++) {
      names.push(this.mj.mj_id2name(this.model!, objType, i))
    }
    return names
  }

  private getMeshName(meshId: number): string {
    try {
      return this.mj.mj_id2name(this.model!, this.mj.mjtObj.mjOBJ_MESH.value, meshId)
    } catch {
      return String(meshId)
    }
  }

  private readGeomNames(model: MjModelInstance): string[] {
    const names: string[] = []
    for (let i = 0; i < model.ngeom; i++) {
      names.push(this.mj.mj_id2name(model, this.mj.mjtObj.mjOBJ_GEOM.value, i))
    }
    return names
  }

  private readVec3FromAccessor(
    arr: Float64Array | { get(i: number): number },
    offset: number,
  ): Vec3 {
    if (arr instanceof Float64Array) {
      return [arr[offset], arr[offset + 1], arr[offset + 2]]
    }
    return [arr.get(offset), arr.get(offset + 1), arr.get(offset + 2)]
  }

  private buildDescriptor(id: string): RobotDescriptor {
    const { model } = this.requireLoaded()
    return {
      id,
      name: this.bodyNames[0] ?? 'unknown',
      bodyCount: model.nbody,
      jointCount: model.njnt,
      actuatorCount: model.nu ?? model.na ?? 0,
      bodies: this.bodyNames.map((name, i) => ({ id: i, name })),
      joints: this.jointNames.map((name, i) => ({
        id: i,
        name,
        type: JOINT_TYPE_MAP[model.jnt_type[i]] ?? 'hinge',
      })),
      actuators: this.getActuatorInfo(),
      visuals: this.getVisualGeoms(),
      cameras: this.getCameraDescriptors(),
    }
  }
}
