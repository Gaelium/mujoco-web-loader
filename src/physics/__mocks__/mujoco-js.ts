/**
 * Mock factory for the MuJoCo WASM module.
 *
 * Produces mock MjModule, MjModelInstance, and MjDataInstance objects that
 * satisfy the MuJoCoEngine's MjModule interface. All numeric arrays are
 * pre-populated with deterministic test data.
 */

import type { MjModule, MjModelInstance, MjDataInstance } from '../MuJoCoEngine'

// ---------------------------------------------------------------------------
// Configuration for mock model generation
// ---------------------------------------------------------------------------

export interface MockModelConfig {
  /** Number of bodies (including worldbody). Default: 3 */
  nbody?: number
  /** Number of joints. Default: 2 */
  njnt?: number
  /** Number of geoms. Default: 2 */
  ngeom?: number
  /** Number of actuators. Default: 1 */
  nu?: number
  /** Number of sensors. Default: 1 */
  nsensor?: number
  /** Number of sites. Default: nsensor (one site per sensor for site-based sensors). */
  nsite?: number
  /** Number of equality constraints. Default: 0 */
  neq?: number
  /** Equality constraint names. Default: [] */
  equalityNames?: string[]
  /** Timestep in seconds. Default: 0.002 */
  timestep?: number
  /** Body names. Default: ['worldbody', 'link1', 'link2'] */
  bodyNames?: string[]
  /** Joint names. Default: ['joint1', 'joint2'] */
  jointNames?: string[]
  /** Geom names. Default: ['geom0', 'geom1'] */
  geomNames?: string[]
  /** Sensor names. Default: ['sensor0'] */
  sensorNames?: string[]
  /** Site names. Default: synthesized as site0, site1, ... */
  siteNames?: string[]
  /** Sensor types per sensor (mjtSensor values). Default: all 4 (gyro). */
  sensorTypes?: number[]
  /** Per-sensor dimension (length of sensordata slice). Default: all 3. */
  sensorDims?: number[]
  /** Joint types (MuJoCo int values: 0=free, 1=ball, 2=slide, 3=hinge). Default: [3, 3] */
  jointTypes?: number[]
  /** Geom types (MuJoCo int values: 2=sphere, 6=box, etc.). Default: [2, 6] */
  geomTypes?: number[]
  /** Site world-frame positions (nsite × 3 flat). Default: zeros. */
  siteXpos?: number[]
  /** Site world-frame rotation matrices (nsite × 9 flat, row-major). Default: identity per site. */
  siteXmat?: number[]
  /** Per-joint limited flag (njnt). Default: all 1 (limited). */
  jointLimited?: number[]
  /** Joint limit ranges (njnt × 2 flat). Default: [-3.14, 3.14] per joint. */
  jointRanges?: number[]
  /** Joint id each actuator drives (nu). Default: actuator i → joint i. */
  actuatorJointIds?: number[]
  /** Per-actuator bias type (nu): 0 = motor (open-loop), nonzero = position. Default: all 1. */
  actuatorBiasTypes?: number[]
  /** Per-actuator control ranges (nu × 2 flat). Default: [-3.14, 3.14] per actuator. */
  actuatorCtrlRanges?: number[]
}

// ---------------------------------------------------------------------------
// Factory: create a mock MjModelInstance
// ---------------------------------------------------------------------------

/**
 * Create a mock MjModelInstance with configurable parameters.
 * @param config - Optional overrides for model properties
 * @returns Mock model with deterministic data arrays
 */
export function createMockModel(config: MockModelConfig = {}): MjModelInstance {
  const nbody = config.nbody ?? 3
  const njnt = config.njnt ?? 2
  const ngeom = config.ngeom ?? 2
  const nu = config.nu ?? 1
  const nsensor = config.nsensor ?? 1
  const neq = config.neq ?? 0
  const timestep = config.timestep ?? 0.002
  const jointTypes = config.jointTypes ?? Array(njnt).fill(3) // all hinge
  const geomTypes = config.geomTypes ?? [2, 6] // sphere, box

  // Equality constraints: MuJoCo 3.x uses mjNEQDATA=11 per constraint.
  // For weld: [anchor1(3), relpose_pos(3), relpose_quat(4), torquescale(1)].
  const EQ_STRIDE = 11
  const eqType = new Int32Array(neq).fill(1) // mjEQ_WELD
  const eqData = new Float64Array(neq * EQ_STRIDE)
  const eqActive0 = new Uint8Array(neq)
  // Initialise relpose to identity: anchor[3]=0, pos[3]=0, quat[4]=(1,0,0,0), torq=1
  for (let i = 0; i < neq; i++) {
    eqData[i * EQ_STRIDE + 6] = 1 // qw
    eqData[i * EQ_STRIDE + 10] = 1 // torquescale
  }

  // qpos/qvel dimensions from joint types
  const qposSizeMap: Record<number, number> = { 0: 7, 1: 4, 2: 1, 3: 1 }
  const qvelSizeMap: Record<number, number> = { 0: 6, 1: 3, 2: 1, 3: 1 }
  let nq = 0
  let nv = 0
  const jntQposadr = new Int32Array(njnt)
  const jntDofadr = new Int32Array(njnt)
  for (let i = 0; i < njnt; i++) {
    jntQposadr[i] = nq
    jntDofadr[i] = nv
    nq += qposSizeMap[jointTypes[i]] ?? 1
    nv += qvelSizeMap[jointTypes[i]] ?? 1
  }

  // Per-sensor dimensions (default 3; overridable by config)
  const sensorDimList = config.sensorDims ?? new Array(nsensor).fill(3)
  let nsensordata = 0
  const sensorAdr = new Int32Array(nsensor)
  const sensorDimArr = new Int32Array(nsensor)
  for (let i = 0; i < nsensor; i++) {
    sensorAdr[i] = nsensordata
    sensorDimArr[i] = sensorDimList[i] ?? 3
    nsensordata += sensorDimArr[i]
  }
  const sensorType = new Int32Array(nsensor)
  const sensorTypeList = config.sensorTypes ?? new Array(nsensor).fill(4)
  for (let i = 0; i < nsensor; i++) sensorType[i] = sensorTypeList[i] ?? 4
  // Default: site-based sensors (objtype=mjOBJ_SITE=6 in MuJoCo). Each sensor
  // references site i, and site i is attached to body min(i, nbody-1).
  const sensorObjType = new Int32Array(nsensor).fill(6)
  const sensorObjId = new Int32Array(nsensor)
  for (let i = 0; i < nsensor; i++) sensorObjId[i] = i

  // Sites — one per sensor by default so site-based sensors resolve.
  const nsite = config.nsite ?? nsensor
  const siteBodyid = new Int32Array(Math.max(nsite, nsensor))
  for (let i = 0; i < siteBodyid.length; i++) {
    siteBodyid[i] = Math.min(i, nbody - 1)
  }
  const siteXpos = new Float64Array(nsite * 3)
  const siteXmat = new Float64Array(nsite * 9)
  if (config.siteXpos) {
    for (let i = 0; i < Math.min(config.siteXpos.length, siteXpos.length); i++) {
      siteXpos[i] = config.siteXpos[i]
    }
  }
  if (config.siteXmat) {
    for (let i = 0; i < Math.min(config.siteXmat.length, siteXmat.length); i++) {
      siteXmat[i] = config.siteXmat[i]
    }
  } else {
    // Identity rotation per site so site +z = world +z by default.
    for (let i = 0; i < nsite; i++) {
      siteXmat[i * 9 + 0] = 1
      siteXmat[i * 9 + 4] = 1
      siteXmat[i * 9 + 8] = 1
    }
  }

  // Body ID for each joint (body 1, 2, ...)
  const jntBodyid = new Int32Array(njnt)
  for (let i = 0; i < njnt; i++) {
    jntBodyid[i] = i + 1
  }

  // Geom body IDs
  const geomBodyid = new Int32Array(ngeom)
  for (let i = 0; i < ngeom; i++) {
    geomBodyid[i] = Math.min(i, nbody - 1)
  }

  // Geom size (3 floats per geom)
  const geomSize = new Float64Array(ngeom * 3)
  for (let i = 0; i < ngeom; i++) {
    geomSize[i * 3] = 0.1 * (i + 1)
    geomSize[i * 3 + 1] = 0.1 * (i + 1)
    geomSize[i * 3 + 2] = 0.1 * (i + 1)
  }

  // Geom local pos/quat
  const geomPos = new Float64Array(ngeom * 3) // all zeros
  const geomQuat = new Float64Array(ngeom * 4)
  for (let i = 0; i < ngeom; i++) {
    geomQuat[i * 4] = 1 // w=1 identity
  }

  // Geom RGBA (as Float32Array to match MuJoCo)
  const geomRgba = new Float32Array(ngeom * 4)
  for (let i = 0; i < ngeom; i++) {
    geomRgba[i * 4] = 0.5
    geomRgba[i * 4 + 1] = 0.5
    geomRgba[i * 4 + 2] = 0.5
    geomRgba[i * 4 + 3] = 1.0
  }

  // Geom dataid (-1 = no mesh)
  const geomDataid = new Int32Array(ngeom).fill(-1)

  // Joint limits — limited by default with a symmetric range.
  const jntLimited = new Uint8Array(
    config.jointLimited ?? new Array(njnt).fill(1),
  )
  const jntRange = new Float64Array(njnt * 2)
  if (config.jointRanges) {
    jntRange.set(config.jointRanges.slice(0, njnt * 2))
  } else {
    for (let i = 0; i < njnt; i++) {
      jntRange[i * 2] = -3.14
      jntRange[i * 2 + 1] = 3.14
    }
  }

  // Actuator introspection arrays. Default: joint transmission, actuator i
  // drives joint i, position bias (feedback), symmetric ctrl range.
  const actuatorTrntype = new Int32Array(nu) // 0 = mjTRN_JOINT
  const actuatorTrnid = new Int32Array(nu * 2).fill(-1)
  const actJointIds = config.actuatorJointIds ?? Array.from({ length: nu }, (_, i) => i)
  for (let i = 0; i < nu; i++) actuatorTrnid[i * 2] = actJointIds[i] ?? i
  const actuatorBiastype = new Int32Array(
    config.actuatorBiasTypes ?? new Array(nu).fill(1),
  )
  const actuatorGaintype = new Int32Array(nu).fill(1)
  const actuatorCtrlrange = new Float64Array(nu * 2)
  if (config.actuatorCtrlRanges) {
    actuatorCtrlrange.set(config.actuatorCtrlRanges.slice(0, nu * 2))
  } else {
    for (let i = 0; i < nu; i++) {
      actuatorCtrlrange[i * 2] = -3.14
      actuatorCtrlrange[i * 2 + 1] = 3.14
    }
  }

  return {
    delete() { /* Embind cleanup stub */ },
    nbody,
    njnt,
    ngeom,
    nu,
    nsensor,
    nsensordata,
    nq,
    nv,
    opt: { timestep, gravity: new Float64Array([0, 0, -9.81]) },
    qpos0: new Float64Array(nq),
    jnt_type: new Int32Array(jointTypes),
    jnt_qposadr: jntQposadr,
    jnt_dofadr: jntDofadr,
    jnt_bodyid: jntBodyid,
    jnt_limited: jntLimited,
    jnt_range: jntRange,
    geom_type: new Int32Array(geomTypes),
    geom_bodyid: geomBodyid,
    geom_size: geomSize,
    geom_pos: geomPos,
    geom_quat: geomQuat,
    geom_rgba: geomRgba,
    geom_dataid: geomDataid,
    sensor_type: sensorType,
    sensor_adr: sensorAdr,
    sensor_dim: sensorDimArr,
    sensor_objtype: sensorObjType,
    sensor_objid: sensorObjId,
    site_bodyid: siteBodyid,
    nsite,
    name_siteadr: new Int32Array(nsite),
    neq,
    eq_type: eqType,
    eq_data: eqData,
    eq_active0: eqActive0,
    name_eqadr: new Int32Array(neq),
    name_bodyadr: new Int32Array(nbody),
    name_jntadr: new Int32Array(njnt),
    name_geomadr: new Int32Array(ngeom),
    name_sensoradr: new Int32Array(nsensor),
    names: new Uint8Array(0),
    actuator_trntype: actuatorTrntype,
    actuator_trnid: actuatorTrnid,
    actuator_ctrlrange: actuatorCtrlrange,
    actuator_gaintype: actuatorGaintype,
    actuator_biastype: actuatorBiastype,
  }
}

// ---------------------------------------------------------------------------
// Factory: create a mock MjDataInstance
// ---------------------------------------------------------------------------

/**
 * Create a mock MjDataInstance that matches a given model.
 * @param model - Mock model to derive dimensions from
 * @param overrides - Optional per-field overrides (e.g., siteXpos, siteXmat).
 * @returns Mock data with zeroed arrays
 */
export function createMockData(
  model: MjModelInstance,
  overrides: { siteXpos?: number[]; siteXmat?: number[] } = {},
): MjDataInstance {
  const xpos = new Float64Array(model.nbody * 3)
  const xquat = new Float64Array(model.nbody * 4)
  // Set identity quaternions
  for (let i = 0; i < model.nbody; i++) {
    xquat[i * 4] = 1 // w=1
    // Set body positions at increasing heights
    xpos[i * 3 + 2] = i * 0.5
  }

  const nsite = model.nsite ?? 0
  const siteXpos = new Float64Array(nsite * 3)
  const siteXmat = new Float64Array(nsite * 9)
  if (overrides.siteXpos) {
    for (let i = 0; i < Math.min(overrides.siteXpos.length, siteXpos.length); i++) {
      siteXpos[i] = overrides.siteXpos[i]
    }
  }
  if (overrides.siteXmat) {
    for (let i = 0; i < Math.min(overrides.siteXmat.length, siteXmat.length); i++) {
      siteXmat[i] = overrides.siteXmat[i]
    }
  } else {
    // Identity rotations per site.
    for (let i = 0; i < nsite; i++) {
      siteXmat[i * 9 + 0] = 1
      siteXmat[i * 9 + 4] = 1
      siteXmat[i * 9 + 8] = 1
    }
  }

  return {
    delete() { /* Embind cleanup stub */ },
    time: 0,
    ncon: 0,
    qpos: new Float64Array(model.nq),
    qvel: new Float64Array(model.nv),
    qfrc_applied: new Float64Array(model.nv),
    ctrl: new Float64Array(model.nu ?? model.na ?? 0),
    xpos,
    xquat,
    sensordata: new Float64Array(model.nsensordata),
    eq_active: new Uint8Array(model.neq ?? 0),
    site_xpos: siteXpos,
    site_xmat: siteXmat,
    contact: {
      size: () => 0,
      get: () => undefined,
    },
  }
}

/**
 * Create a mock MjDataInstance with simulated contacts.
 * @param model - Mock model to derive dimensions from
 * @param contacts - Contact configurations to inject
 * @returns Mock data with active contacts
 */
export function createMockDataWithContacts(
  model: MjModelInstance,
  contacts: Array<{ geom1: number; geom2: number; pos: [number, number, number]; normal: [number, number, number] }>,
): MjDataInstance {
  const base = createMockData(model)
  return {
    ...base,
    ncon: contacts.length,
    contact: {
      size: () => contacts.length,
      get: (i: number) => {
        const c = contacts[i]
        if (!c) return undefined
        // MuJoCo contact frame: first 3 values = normal, rest = tangent vectors
        const frame = new Float64Array(9)
        frame[0] = c.normal[0]
        frame[1] = c.normal[1]
        frame[2] = c.normal[2]
        return {
          pos: new Float64Array(c.pos),
          frame,
          geom1: c.geom1,
          geom2: c.geom2,
        }
      },
    },
  }
}

// ---------------------------------------------------------------------------
// Factory: create a complete mock MjModule
// ---------------------------------------------------------------------------

/**
 * Create a mock MjModule with full factory support.
 * @param modelConfig - Config to use when loadFromXML is called
 * @returns Mock module satisfying the MjModule interface
 */
export function createMockMjModule(modelConfig: MockModelConfig = {}): MockMjModule {
  const vfsFiles = new Map<string, string | Uint8Array>()
  const vfsDirs = new Set<string>(['/'])

  const bodyNames = modelConfig.bodyNames ?? ['worldbody', 'link1', 'link2']
  const jointNames = modelConfig.jointNames ?? ['joint1', 'joint2']
  const geomNames = modelConfig.geomNames ?? ['geom0', 'geom1']
  const sensorNames = modelConfig.sensorNames ?? ['sensor0']
  const defaultNsite = modelConfig.nsite ?? (modelConfig.nsensor ?? 1)
  const siteNames = modelConfig.siteNames ??
    Array.from({ length: defaultNsite }, (_, i) => `site${i}`)
  const equalityNames = modelConfig.equalityNames ?? []

  // Build the name lookup
  const allNames: Record<string, Record<number, string>> = {
    '1': {}, // mjOBJ_BODY
    '3': {}, // mjOBJ_JOINT
    '5': {}, // mjOBJ_GEOM
    '6': {}, // mjOBJ_SITE
    '20': {}, // mjOBJ_SENSOR
    '19': {}, // mjOBJ_ACTUATOR
    '21': {}, // mjOBJ_EQUALITY
  }
  bodyNames.forEach((n, i) => { allNames['1'][i] = n })
  jointNames.forEach((n, i) => { allNames['3'][i] = n })
  geomNames.forEach((n, i) => { allNames['5'][i] = n })
  siteNames.forEach((n, i) => { allNames['6'][i] = n })
  sensorNames.forEach((n, i) => { allNames['20'][i] = n })
  equalityNames.forEach((n, i) => { allNames['21'][i] = n })

  let mockModel: MjModelInstance | null = null
  let lastLoadedData: MjDataInstance | null = null

  const stepFn = (_model: MjModelInstance, data: MjDataInstance): void => {
    data.time += _model.opt.timestep
  }

  const resetDataFn = (_model: MjModelInstance, data: MjDataInstance): void => {
    data.time = 0
    data.qpos.fill(0)
    data.qvel.fill(0)
    data.ctrl.fill(0)
  }

  const forwardFn = (): void => {
    // No-op in mock — xpos/xquat are pre-set
  }

  const module: MockMjModule = {
    FS: {
      writeFile: (path: string, data: string | Uint8Array) => {
        vfsFiles.set(path, data)
      },
      readFile: (path: string): Uint8Array => {
        const v = vfsFiles.get(path)
        if (v === undefined) throw new Error(`ENOENT: ${path}`)
        return typeof v === 'string' ? new TextEncoder().encode(v) : v
      },
      mkdir: (path: string) => {
        vfsDirs.add(path)
      },
      readdir: (path: string) => {
        if (!vfsDirs.has(path)) throw new Error(`ENOENT: ${path}`)
        const entries = ['.', '..']
        const prefix = path.endsWith('/') ? path : path + '/'
        for (const filePath of vfsFiles.keys()) {
          if (filePath.startsWith(prefix)) {
            const rest = filePath.slice(prefix.length)
            const name = rest.split('/')[0]
            if (name && !entries.includes(name)) entries.push(name)
          }
        }
        for (const dir of vfsDirs) {
          if (dir.startsWith(prefix) && dir !== path) {
            const rest = dir.slice(prefix.length)
            const name = rest.split('/')[0]
            if (name && !entries.includes(name)) entries.push(name)
          }
        }
        return entries
      },
      unlink: (path: string) => {
        vfsFiles.delete(path)
      },
    },
    MjModel: {
      mj_loadXML: () => {
        const model = createMockModel(modelConfig)
        mockModel = model
        return model
      },
      from_xml_path: () => {
        const model = createMockModel(modelConfig)
        mockModel = model
        return model
      },
      from_xml_string: () => {
        const model = createMockModel(modelConfig)
        mockModel = model
        return model
      },
      loadFromXML: () => {
        const model = createMockModel(modelConfig)
        mockModel = model
        return model
      },
      // Mock binary loader — reads the magic-prefixed VFS file written by
      // mj_saveModel below and reconstructs an equivalent model. The mock's
      // models are config-driven, so the "binary" is just a marker; the
      // round-trip preserves nbody/njnt/etc by reusing modelConfig.
      mj_loadModel: () => {
        const model = createMockModel(modelConfig)
        mockModel = model
        return model
      },
    },
    MjData: class {
      time: number
      ncon: number
      qpos: Float64Array
      qvel: Float64Array
      ctrl: Float64Array
      xpos: Float64Array
      xquat: Float64Array
      sensordata: Float64Array
      site_xpos?: Float64Array
      site_xmat?: Float64Array
      eq_active: Uint8Array
      contact: { size(): number; get(i: number): undefined }

      constructor(model: MjModelInstance) {
        const data = createMockData(model, {
          siteXpos: modelConfig.siteXpos,
          siteXmat: modelConfig.siteXmat,
        })
        this.time = data.time
        this.ncon = data.ncon
        this.qpos = data.qpos as Float64Array
        this.qvel = data.qvel as Float64Array
        this.ctrl = data.ctrl as Float64Array
        this.xpos = data.xpos as Float64Array
        this.xquat = data.xquat as Float64Array
        this.sensordata = data.sensordata as Float64Array
        this.site_xpos = data.site_xpos
        this.site_xmat = data.site_xmat
        this.eq_active = (data.eq_active ?? new Uint8Array(model.neq ?? 0)) as Uint8Array
        this.contact = data.contact as { size(): number; get(i: number): undefined }
        lastLoadedData = this as unknown as MjDataInstance
      }
    } as unknown as MjModule['MjData'],
    mj_step: stepFn,
    mj_resetData: resetDataFn,
    mj_forward: forwardFn,
    // Mock binary save — writes a magic marker to the VFS path so the loader
    // (mj_loadModel above) can read it back. The mock's "binary" is opaque;
    // round-trip identity is preserved by reconstructing from modelConfig.
    mj_saveModel: (_model: MjModelInstance, path: string) => {
      vfsFiles.set(path, new Uint8Array([0x4d, 0x4a, 0x42, 0x00])) // "MJB\0"
    },
    // Stub the Emscripten runtime helper that mj_saveModel relies on.
    // Real WASM builds may omit it; the engine's saveBinary() detects the
    // missing export and skips the call. In the mock we provide it so the
    // happy-path tests can exercise the save variant.
    writeI53ToI64: () => { /* mock stub */ },
    mj_id2name: (_model: MjModelInstance, objType: number, id: number): string => {
      return allNames[String(objType)]?.[id] ?? `unnamed_${objType}_${id}`
    },
    mj_version: () => 315,
    mjtObj: {
      mjOBJ_BODY: { value: 1 },
      mjOBJ_JOINT: { value: 3 },
      mjOBJ_GEOM: { value: 5 },
      mjOBJ_SITE: { value: 6 },
      mjOBJ_MESH: { value: 10 },
      mjOBJ_SENSOR: { value: 20 },
      mjOBJ_ACTUATOR: { value: 19 },
      mjOBJ_EQUALITY: { value: 21 },
    },
    mjtEq: {
      mjEQ_CONNECT: { value: 0 },
      mjEQ_WELD: { value: 1 },
      mjEQ_JOINT: { value: 2 },
      mjEQ_TENDON: { value: 3 },
    },
    mjtTrn: {
      mjTRN_JOINT: { value: 0 },
    },
    mjtJoint: {
      mjJNT_FREE: { value: 0 },
      mjJNT_BALL: { value: 1 },
      mjJNT_SLIDE: { value: 2 },
      mjJNT_HINGE: { value: 3 },
    },
    mjtGeom: {
      mjGEOM_PLANE: { value: 0 },
      mjGEOM_SPHERE: { value: 2 },
      mjGEOM_CAPSULE: { value: 3 },
      mjGEOM_ELLIPSOID: { value: 4 },
      mjGEOM_CYLINDER: { value: 5 },
      mjGEOM_BOX: { value: 6 },
      mjGEOM_MESH: { value: 7 },
    },

    // Test helpers
    getVFSFiles: () => vfsFiles,
    getVFSDirs: () => vfsDirs,
    getLastModel: () => mockModel,
    getLastData: () => lastLoadedData,
  }

  return module
}

/** Extended mock module with test introspection helpers. */
export interface MockMjModule extends MjModule {
  getVFSFiles(): Map<string, string | Uint8Array>
  getVFSDirs(): Set<string>
  getLastModel(): MjModelInstance | null
  getLastData(): MjDataInstance | null
  /** Emscripten runtime helper used by mj_saveModel — present in the mock
   *  so save tests can exercise the happy path. */
  writeI53ToI64(): void
}
