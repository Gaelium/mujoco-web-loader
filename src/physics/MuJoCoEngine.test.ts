import { describe, it, expect, beforeEach } from 'vitest'
import { MuJoCoEngine } from './MuJoCoEngine'
import {
  createMockMjModule,
  createMockModel,
  createMockDataWithContacts,
  type MockMjModule,
} from './__mocks__/mujoco-js'

const SIMPLE_XML = '<mujoco><worldbody><body name="link1"/></worldbody></mujoco>'

describe('MuJoCoEngine', () => {
  let mj: MockMjModule
  let engine: MuJoCoEngine

  beforeEach(() => {
    mj = createMockMjModule()
    engine = new MuJoCoEngine(mj)
  })

  // -----------------------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------------------

  describe('getVersion', () => {
    it('returns the MuJoCo version number', () => {
      expect(engine.getVersion()).toBe(315)
    })
  })

  describe('isLoaded', () => {
    it('returns false before loadModel', () => {
      expect(engine.isLoaded()).toBe(false)
    })

    it('returns true after loadModel', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.isLoaded()).toBe(true)
    })
  })

  // -----------------------------------------------------------------------
  // VFS operations
  // -----------------------------------------------------------------------

  describe('writeToVFS', () => {
    it('writes string data to the virtual filesystem', () => {
      engine.writeToVFS('/test.xml', '<mujoco/>')
      expect(mj.getVFSFiles().get('/test.xml')).toBe('<mujoco/>')
    })

    it('writes binary data to the virtual filesystem', () => {
      const data = new Uint8Array([1, 2, 3])
      engine.writeToVFS('/mesh.stl', data)
      expect(mj.getVFSFiles().get('/mesh.stl')).toBe(data)
    })
  })

  describe('ensureVFSDir', () => {
    it('creates a directory that does not exist', () => {
      engine.ensureVFSDir('/meshes')
      expect(mj.getVFSDirs().has('/meshes')).toBe(true)
    })

    it('does not throw if directory already exists', () => {
      engine.ensureVFSDir('/')
      expect(mj.getVFSDirs().has('/')).toBe(true)
    })
  })

  // -----------------------------------------------------------------------
  // Model loading
  // -----------------------------------------------------------------------

  describe('loadModel', () => {
    it('writes the XML to the VFS', () => {
      engine.loadModel(SIMPLE_XML)
      expect(mj.getVFSFiles().has('/model.xml')).toBe(true)
    })

    it('writes to a custom filename', () => {
      engine.loadModel(SIMPLE_XML, '/robot.urdf')
      expect(mj.getVFSFiles().has('/robot.urdf')).toBe(true)
    })

    it('returns a RobotDescriptor', () => {
      const desc = engine.loadModel(SIMPLE_XML)
      expect(desc.id).toBe('model')
      expect(desc.bodyCount).toBe(3)
      expect(desc.jointCount).toBe(2)
      expect(desc.actuatorCount).toBe(1)
    })

    it('populates body descriptors with names', () => {
      const desc = engine.loadModel(SIMPLE_XML)
      expect(desc.bodies).toHaveLength(3)
      expect(desc.bodies[0]).toEqual({ id: 0, name: 'worldbody' })
      expect(desc.bodies[1]).toEqual({ id: 1, name: 'link1' })
      expect(desc.bodies[2]).toEqual({ id: 2, name: 'link2' })
    })

    it('populates joint descriptors with types', () => {
      const desc = engine.loadModel(SIMPLE_XML)
      expect(desc.joints).toHaveLength(2)
      expect(desc.joints[0]).toEqual({ id: 0, name: 'joint1', type: 'hinge' })
    })

    it('populates visual geoms', () => {
      const desc = engine.loadModel(SIMPLE_XML)
      expect(desc.visuals.length).toBeGreaterThan(0)
      expect(desc.visuals[0].type).toBe('sphere')
    })

    it('calls mj_forward to populate transforms', () => {
      let forwardCalled = false
      mj.mj_forward = () => { forwardCalled = true }
      engine.loadModel(SIMPLE_XML)
      expect(forwardCalled).toBe(true)
    })
  })

  // -----------------------------------------------------------------------
  // Binary save/load (compile-once cache pipeline)
  // -----------------------------------------------------------------------

  describe('saveBinary / loadBinary', () => {
    it('returns null from saveBinary when no model is loaded', () => {
      expect(engine.saveBinary()).toBeNull()
    })

    it('returns bytes from saveBinary after loadModel', () => {
      engine.loadModel(SIMPLE_XML)
      const bytes = engine.saveBinary()
      expect(bytes).not.toBeNull()
      expect(bytes).toBeInstanceOf(Uint8Array)
      expect(bytes!.byteLength).toBeGreaterThan(0)
    })

    it('round-trips through loadBinary preserving descriptor structure', () => {
      const xmlDesc = engine.loadModel(SIMPLE_XML)
      const bytes = engine.saveBinary()
      expect(bytes).not.toBeNull()

      // Fresh engine to prove loadBinary doesn't depend on prior state.
      const fresh = new MuJoCoEngine(mj)
      const binDesc = fresh.loadBinary(bytes!)
      expect(binDesc.bodyCount).toBe(xmlDesc.bodyCount)
      expect(binDesc.jointCount).toBe(xmlDesc.jointCount)
      expect(binDesc.actuatorCount).toBe(xmlDesc.actuatorCount)
      expect(binDesc.bodies.map((b) => b.name)).toEqual(xmlDesc.bodies.map((b) => b.name))
    })

    it('writes the binary to the VFS at the requested path', () => {
      engine.loadModel(SIMPLE_XML)
      const bytes = engine.saveBinary()!
      const fresh = new MuJoCoEngine(mj)
      fresh.loadBinary(bytes, '/cached/model.mjb')
      expect(mj.getVFSFiles().has('/cached/model.mjb')).toBe(true)
    })

    it('returns null when no save variant is available', () => {
      const minimalMj = createMockMjModule()
      // Strip every save variant.
      delete (minimalMj as { mj_saveModel?: unknown }).mj_saveModel
      const eng = new MuJoCoEngine(minimalMj)
      eng.loadModel(SIMPLE_XML)
      const lastModel = minimalMj.getLastModel() as { saveModel?: unknown; save?: unknown } | null
      if (lastModel) {
        delete lastModel.saveModel
        delete lastModel.save
      }
      expect(eng.saveBinary()).toBeNull()
    })

    it('throws from loadBinary when no binary loader is available', () => {
      const minimalMj = createMockMjModule()
      delete minimalMj.MjModel.mj_loadModel
      delete minimalMj.MjModel.from_binary_path
      delete minimalMj.MjModel.loadBinary
      const eng = new MuJoCoEngine(minimalMj)
      expect(() => eng.loadBinary(new Uint8Array([1, 2, 3]))).toThrow(/binary model loader/i)
    })

    it('does not access mj_saveModel when writeI53ToI64 is missing (Embind getter regression)', () => {
      // The real @mujoco/mujoco build exposes mj_saveModel as a lazy-resolve
      // Embind getter whose glue calls writeI53ToI64. If writeI53ToI64 isn't
      // exported, even *reading* mj_saveModel aborts the WASM runtime. This
      // regression guards saveBinary against that — it must short-circuit
      // BEFORE any property access on mj.mj_saveModel.
      const trapMj = createMockMjModule()
      const trapAny = trapMj as unknown as { writeI53ToI64?: unknown }
      delete trapAny.writeI53ToI64
      let mjSaveModelTouched = false
      Object.defineProperty(trapMj, 'mj_saveModel', {
        configurable: true,
        get() {
          mjSaveModelTouched = true
          throw new Error('Embind getter would abort WASM here')
        },
      })
      const eng = new MuJoCoEngine(trapMj)
      eng.loadModel(SIMPLE_XML)
      // Strip instance variants too so saveBinary returns null for the right reason.
      const lastModel = trapMj.getLastModel() as { saveModel?: unknown; save?: unknown } | null
      if (lastModel) {
        delete lastModel.saveModel
        delete lastModel.save
      }
      expect(eng.saveBinary()).toBeNull()
      expect(mjSaveModelTouched).toBe(false)
    })
  })

  // -----------------------------------------------------------------------
  // Simulation stepping
  // -----------------------------------------------------------------------

  describe('step', () => {
    it('throws if no model is loaded', () => {
      expect(() => engine.step()).toThrow('No model loaded')
    })

    it('advances simulation time by one timestep', () => {
      engine.loadModel(SIMPLE_XML)
      const t0 = engine.getTime()
      engine.step()
      expect(engine.getTime()).toBeCloseTo(t0 + 0.002, 10)
    })
  })

  describe('stepN', () => {
    it('advances by N timesteps', () => {
      engine.loadModel(SIMPLE_XML)
      engine.stepN(10)
      expect(engine.getTime()).toBeCloseTo(0.02, 10)
    })
  })

  // -----------------------------------------------------------------------
  // Reset
  // -----------------------------------------------------------------------

  describe('reset', () => {
    it('throws if no model is loaded', () => {
      expect(() => engine.reset()).toThrow('No model loaded')
    })

    it('resets time to zero', () => {
      engine.loadModel(SIMPLE_XML)
      engine.stepN(5)
      expect(engine.getTime()).toBeGreaterThan(0)
      engine.reset()
      expect(engine.getTime()).toBe(0)
    })

    it('zeroes qpos, qvel, and ctrl', () => {
      engine.loadModel(SIMPLE_XML)
      engine.setControl(0, 1.5)
      engine.reset()
      const data = mj.getLastData()!
      expect(data.ctrl[0]).toBe(0)
    })
  })

  // -----------------------------------------------------------------------
  // Controls
  // -----------------------------------------------------------------------

  describe('setControl', () => {
    it('throws if no model is loaded', () => {
      expect(() => engine.setControl(0, 1.0)).toThrow('No model loaded')
    })

    it('sets a single actuator control value', () => {
      engine.loadModel(SIMPLE_XML)
      engine.setControl(0, 3.14)
      const data = mj.getLastData()!
      expect(data.ctrl[0]).toBeCloseTo(3.14)
    })
  })

  describe('getActuatorInfo', () => {
    it('returns one descriptor per actuator with joint mapping + ctrl range', () => {
      const mjCustom = createMockMjModule({
        nu: 2,
        jointNames: ['joint1', 'joint2'],
        actuatorCtrlRanges: [-1, 1, -2, 2],
      })
      const eng = new MuJoCoEngine(mjCustom)
      eng.loadModel(SIMPLE_XML)
      const info = eng.getActuatorInfo()
      expect(info).toHaveLength(2)
      expect(info[0].jointName).toBe('joint1')
      expect(info[0].ctrlRange).toEqual([-1, 1])
      expect(info[1].jointName).toBe('joint2')
      expect(info[1].ctrlRange).toEqual([-2, 2])
    })

    it('classifies bias-feedback actuators as position and open-loop as motor', () => {
      const mjCustom = createMockMjModule({ nu: 2, actuatorBiasTypes: [1, 0] })
      const eng = new MuJoCoEngine(mjCustom)
      eng.loadModel(SIMPLE_XML)
      const info = eng.getActuatorInfo()
      expect(info[0].type).toBe('position')
      expect(info[1].type).toBe('motor')
    })

    it('returns [] when the model has no actuators', () => {
      const mjCustom = createMockMjModule({ nu: 0 })
      const eng = new MuJoCoEngine(mjCustom)
      eng.loadModel(SIMPLE_XML)
      expect(eng.getActuatorInfo()).toEqual([])
    })

    it('is surfaced on the descriptor', () => {
      const mjCustom = createMockMjModule({ nu: 1, jointNames: ['joint1', 'joint2'] })
      const eng = new MuJoCoEngine(mjCustom)
      const desc = eng.loadModel(SIMPLE_XML)
      expect(desc.actuators).toHaveLength(1)
      expect(desc.actuators[0].jointName).toBe('joint1')
    })
  })

  describe('getJointRange', () => {
    it('returns the limit range for a limited joint', () => {
      const mjCustom = createMockMjModule({
        jointNames: ['joint1', 'joint2'],
        jointRanges: [-1.5, 1.5, -0.5, 2.5],
      })
      const eng = new MuJoCoEngine(mjCustom)
      eng.loadModel(SIMPLE_XML)
      expect(eng.getJointRange('joint1')).toEqual([-1.5, 1.5])
      expect(eng.getJointRange('joint2')).toEqual([-0.5, 2.5])
    })

    it('returns null for an unlimited joint', () => {
      const mjCustom = createMockMjModule({ jointNames: ['j'], njnt: 1, jointLimited: [0] })
      const eng = new MuJoCoEngine(mjCustom)
      eng.loadModel(SIMPLE_XML)
      expect(eng.getJointRange('j')).toBeNull()
    })

    it('returns null for an unknown joint', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.getJointRange('nonexistent')).toBeNull()
    })
  })

  describe('setControls', () => {
    it('sets multiple control values from a sparse record', () => {
      const mjCustom = createMockMjModule({ nu: 3 })
      const eng = new MuJoCoEngine(mjCustom)
      eng.loadModel(SIMPLE_XML)
      eng.setControls({ 0: 1.0, 2: -0.5 })
      const data = mjCustom.getLastData()!
      expect(data.ctrl[0]).toBeCloseTo(1.0)
      expect(data.ctrl[1]).toBe(0)
      expect(data.ctrl[2]).toBeCloseTo(-0.5)
    })
  })

  // -----------------------------------------------------------------------
  // Timestep
  // -----------------------------------------------------------------------

  describe('getTimestep / setTimestep', () => {
    it('reads the default timestep', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.getTimestep()).toBeCloseTo(0.002)
    })

    it('changes the timestep', () => {
      engine.loadModel(SIMPLE_XML)
      engine.setTimestep(0.001)
      expect(engine.getTimestep()).toBeCloseTo(0.001)
    })
  })

  // -----------------------------------------------------------------------
  // Gravity
  // -----------------------------------------------------------------------

  describe('setGravity', () => {
    it('updates the gravity vector on the loaded model', () => {
      engine.loadModel(SIMPLE_XML)
      engine.setGravity([0, 0, -3.71])
      engine.step()
    })

    it('throws if no model is loaded', () => {
      expect(() => engine.setGravity([0, 0, -9.81])).toThrow('No model loaded')
    })
  })

  // -----------------------------------------------------------------------
  // setQpos / forward
  // -----------------------------------------------------------------------

  describe('setQpos', () => {
    it('writes values to the qpos array at the given address', () => {
      engine.loadModel(SIMPLE_XML)
      engine.setQpos(0, [1.5, -0.3])
      // Verify by reading joint states — qpos[0] should be 1.5
      const joints = engine.getJointStates()
      expect(joints[0].qpos[0]).toBeCloseTo(1.5)
    })

    it('throws if no model is loaded', () => {
      expect(() => engine.setQpos(0, [1.0])).toThrow('No model loaded')
    })
  })

  describe('forward', () => {
    it('recomputes body transforms after qpos modification', () => {
      engine.loadModel(SIMPLE_XML)
      engine.setQpos(0, [0.5])
      engine.forward()
      // Verify body states are updated (doesn't throw)
      const bodies = engine.getBodyStates()
      expect(bodies.length).toBeGreaterThan(0)
    })

    it('throws if no model is loaded', () => {
      expect(() => engine.forward()).toThrow('No model loaded')
    })
  })

  // -----------------------------------------------------------------------
  // Body states
  // -----------------------------------------------------------------------

  describe('getBodyStates', () => {
    it('throws if no model is loaded', () => {
      expect(() => engine.getBodyStates()).toThrow('No model loaded')
    })

    it('returns one entry per body', () => {
      engine.loadModel(SIMPLE_XML)
      const bodies = engine.getBodyStates()
      expect(bodies).toHaveLength(3)
    })

    it('includes correct names', () => {
      engine.loadModel(SIMPLE_XML)
      const bodies = engine.getBodyStates()
      expect(bodies[0].name).toBe('worldbody')
      expect(bodies[1].name).toBe('link1')
    })

    it('returns position and quaternion arrays', () => {
      engine.loadModel(SIMPLE_XML)
      const bodies = engine.getBodyStates()
      // worldbody at z=0, link1 at z=0.5, link2 at z=1.0 (from mock)
      expect(bodies[0].position).toEqual([0, 0, 0])
      expect(bodies[1].position[2]).toBeCloseTo(0.5)
      // Identity quaternion
      expect(bodies[0].quaternion).toEqual([1, 0, 0, 0])
    })
  })

  // -----------------------------------------------------------------------
  // Joint states
  // -----------------------------------------------------------------------

  describe('getJointStates', () => {
    it('returns one entry per joint', () => {
      engine.loadModel(SIMPLE_XML)
      const joints = engine.getJointStates()
      expect(joints).toHaveLength(2)
    })

    it('has correct joint types', () => {
      engine.loadModel(SIMPLE_XML)
      const joints = engine.getJointStates()
      expect(joints[0].type).toBe('hinge')
      expect(joints[1].type).toBe('hinge')
    })

    it('returns qpos and qvel arrays of correct size', () => {
      engine.loadModel(SIMPLE_XML)
      const joints = engine.getJointStates()
      // Hinge joints have 1 qpos and 1 qvel
      expect(joints[0].qpos).toHaveLength(1)
      expect(joints[0].qvel).toHaveLength(1)
    })

    it('handles mixed joint types', () => {
      const mjMixed = createMockMjModule({
        njnt: 3,
        jointTypes: [0, 1, 3], // free, ball, hinge
        jointNames: ['free_jnt', 'ball_jnt', 'hinge_jnt'],
      })
      const eng = new MuJoCoEngine(mjMixed)
      eng.loadModel(SIMPLE_XML)
      const joints = eng.getJointStates()
      expect(joints[0].type).toBe('free')
      expect(joints[0].qpos).toHaveLength(7)
      expect(joints[0].qvel).toHaveLength(6)
      expect(joints[1].type).toBe('ball')
      expect(joints[1].qpos).toHaveLength(4)
      expect(joints[1].qvel).toHaveLength(3)
      expect(joints[2].type).toBe('hinge')
      expect(joints[2].qpos).toHaveLength(1)
      expect(joints[2].qvel).toHaveLength(1)
    })
  })

  // -----------------------------------------------------------------------
  // Sensor readings
  // -----------------------------------------------------------------------

  describe('getSensorReadings', () => {
    it('returns one entry per sensor', () => {
      engine.loadModel(SIMPLE_XML)
      const sensors = engine.getSensorReadings()
      expect(sensors).toHaveLength(1)
    })

    it('has correct name and dimension', () => {
      engine.loadModel(SIMPLE_XML)
      const sensors = engine.getSensorReadings()
      expect(sensors[0].name).toBe('sensor0')
      expect(sensors[0].data).toHaveLength(3) // dim=3 in mock
    })

    it('returns multiple sensors', () => {
      const mjMulti = createMockMjModule({
        nsensor: 3,
        sensorNames: ['accel', 'gyro', 'force'],
      })
      const eng = new MuJoCoEngine(mjMulti)
      eng.loadModel(SIMPLE_XML)
      const sensors = eng.getSensorReadings()
      expect(sensors).toHaveLength(3)
      expect(sensors.map(s => s.name)).toEqual(['accel', 'gyro', 'force'])
    })

    it('resolves the body a site-based sensor is attached to', () => {
      // Mock default: sensor i → site i → body min(i, nbody-1)
      const mjMulti = createMockMjModule({
        nbody: 4,
        nsensor: 2,
        sensorNames: ['imu_gyro', 'imu_accel'],
      })
      const eng = new MuJoCoEngine(mjMulti)
      eng.loadModel(SIMPLE_XML)
      const readings = eng.getSensorReadings()
      expect(readings[0].bodyId).toBe(0)
      expect(readings[1].bodyId).toBe(1)
      // Direct API
      expect(eng.getSensorBodyId(0)).toBe(0)
      expect(eng.getSensorBodyId(1)).toBe(1)
    })

    it('returns -1 for out-of-range sensor indices', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.getSensorBodyId(-1)).toBe(-1)
      expect(engine.getSensorBodyId(999)).toBe(-1)
    })

    it('populates siteId for site-based sensors', () => {
      const mjMulti = createMockMjModule({
        nbody: 4,
        nsensor: 3,
        sensorNames: ['s0', 's1', 's2'],
      })
      const eng = new MuJoCoEngine(mjMulti)
      eng.loadModel(SIMPLE_XML)
      const readings = eng.getSensorReadings()
      // Default mock: each sensor references site i (objtype=mjOBJ_SITE=6).
      expect(readings[0].siteId).toBe(0)
      expect(readings[1].siteId).toBe(1)
      expect(readings[2].siteId).toBe(2)
    })
  })

  // -----------------------------------------------------------------------
  // Sites (world-frame transforms)
  // -----------------------------------------------------------------------

  describe('getSiteStates', () => {
    it('returns [] when the model has no sites', () => {
      const mjZero = createMockMjModule({ nsensor: 0, nsite: 0 })
      const eng = new MuJoCoEngine(mjZero)
      eng.loadModel(SIMPLE_XML)
      expect(eng.getSiteStates()).toEqual([])
    })

    it('returns one entry per site with default identity transforms', () => {
      const mj2 = createMockMjModule({
        nbody: 3,
        nsensor: 2,
        nsite: 2,
        sensorNames: ['imu_gyro', 'imu_accel'],
      })
      const eng = new MuJoCoEngine(mj2)
      eng.loadModel(SIMPLE_XML)
      const sites = eng.getSiteStates()
      expect(sites).toHaveLength(2)
      // Identity rotation → local +z = world +z.
      expect(sites[0].direction).toEqual([0, 0, 1])
      expect(sites[0].position).toEqual([0, 0, 0])
      expect(sites[0].bodyId).toBe(0)
      expect(sites[1].bodyId).toBe(1)
      expect(sites[0].name).toBe('site0')
    })

    it('reads position and +z direction from site_xpos/site_xmat', () => {
      // Site 0 at (1, 2, 3) with rotation matrix whose +z axis = (0, 1, 0).
      // Row-major 3x3 where third column = (m02, m12, m22) = (0, 1, 0):
      //   m00 m01 m02 = 1 0 0   (arbitrary rows, we only assert on the +z col)
      //   m10 m11 m12 = 0 0 1
      //   m20 m21 m22 = 0 1 0  ← actually we want m22=0, m12=1, m02=0
      // So matrix = [1,0,0, 0,0,1, 0,1,0]  (rotate +z to +y)
      const mjSited = createMockMjModule({
        nbody: 2,
        nsensor: 1,
        nsite: 1,
        siteXpos: [1, 2, 3],
        siteXmat: [1, 0, 0, 0, 0, 1, 0, 1, 0],
      })
      const eng = new MuJoCoEngine(mjSited)
      eng.loadModel(SIMPLE_XML)
      const sites = eng.getSiteStates()
      expect(sites[0].position).toEqual([1, 2, 3])
      expect(sites[0].direction).toEqual([0, 1, 0])
    })

    it('returns [] when data.site_xpos is absent', () => {
      // Simulate an older WASM binding: build a mock but strip site_xpos.
      const mjStripped = createMockMjModule({ nsensor: 1, nsite: 1 })
      const eng = new MuJoCoEngine(mjStripped)
      eng.loadModel(SIMPLE_XML)
      // Erase the data's site_xpos as if the binding didn't expose it.
      const data = mjStripped.getLastData() as unknown as { site_xpos?: Float64Array }
      data.site_xpos = undefined
      expect(eng.getSiteStates()).toEqual([])
    })
  })

  // -----------------------------------------------------------------------
  // Equality constraints (fixed-base weld support)
  // -----------------------------------------------------------------------

  describe('equality constraints', () => {
    it('returns 0 equality count for a model with no constraints', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.getEqualityCount()).toBe(0)
      expect(engine.findEqualityByName('missing')).toBe(-1)
    })

    it('finds a weld equality by name and toggles it active/inactive', () => {
      const mjWeld = createMockMjModule({
        neq: 1,
        equalityNames: ['_simulo_fixed_base'],
      })
      const eng = new MuJoCoEngine(mjWeld)
      eng.loadModel(SIMPLE_XML)

      expect(eng.getEqualityCount()).toBe(1)
      const idx = eng.findEqualityByName('_simulo_fixed_base')
      expect(idx).toBe(0)

      expect(eng.setEqualityActive(idx, true)).toBe(true)
      expect(mjWeld.getLastData()!.eq_active![0]).toBe(1)

      expect(eng.setEqualityActive(idx, false)).toBe(true)
      expect(mjWeld.getLastData()!.eq_active![0]).toBe(0)
    })

    it('updates a weld equality relpose in eq_data', () => {
      const mjWeld = createMockMjModule({
        neq: 1,
        equalityNames: ['_simulo_fixed_base'],
      })
      const eng = new MuJoCoEngine(mjWeld)
      eng.loadModel(SIMPLE_XML)

      const ok = eng.setWeldRelpose(0, [1.5, -0.5, 2], [0, 1, 0, 0])
      expect(ok).toBe(true)

      // mjEQ_WELD layout: anchor1(3), pos(3), quat(4), torquescale(1) → offset 3..9
      const data = mjWeld.getLastModel()!.eq_data!
      expect(Array.from(data.slice(3, 10))).toEqual([1.5, -0.5, 2, 0, 1, 0, 0])
    })
  })

  // -----------------------------------------------------------------------
  // Contacts
  // -----------------------------------------------------------------------

  describe('getContacts', () => {
    it('returns empty array when no contacts', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.getContacts()).toEqual([])
    })

    it('reads contact data from mock contacts', () => {
      const mjDirect = createMockMjModule()
      const eng = new MuJoCoEngine(mjDirect)
      eng.loadModel(SIMPLE_XML)

      // Inject contacts into the existing data instance
      const model = mjDirect.getLastModel()!
      const lastData = mjDirect.getLastData()!
      const contactMock = createMockDataWithContacts(model, [
        { geom1: 0, geom2: 1, pos: [1, 2, 3], normal: [0, 0, 1] },
      ])
      Object.defineProperty(lastData, 'ncon', { value: 1, writable: true })
      Object.defineProperty(lastData, 'contact', { value: contactMock.contact })

      const contacts = eng.getContacts()
      expect(contacts).toHaveLength(1)
      expect(contacts[0].position).toEqual([1, 2, 3])
      expect(contacts[0].normal).toEqual([0, 0, 1])
      expect(contacts[0].geom1).toBe('geom0')
      expect(contacts[0].geom2).toBe('geom1')
    })
  })

  // -----------------------------------------------------------------------
  // Visual geoms
  // -----------------------------------------------------------------------

  describe('getVisualGeoms', () => {
    it('returns visual geometry for all geoms', () => {
      engine.loadModel(SIMPLE_XML)
      const visuals = engine.getVisualGeoms()
      expect(visuals).toHaveLength(2)
    })

    it('maps geom types correctly', () => {
      engine.loadModel(SIMPLE_XML)
      const visuals = engine.getVisualGeoms()
      expect(visuals[0].type).toBe('sphere')
      expect(visuals[1].type).toBe('box')
    })

    it('includes size, position, and quaternion', () => {
      engine.loadModel(SIMPLE_XML)
      const visuals = engine.getVisualGeoms()
      expect(visuals[0].size).toEqual([0.1, 0.1, 0.1])
      expect(visuals[0].localQuat).toEqual([1, 0, 0, 0])
    })

    it('skips unsupported geom types', () => {
      const mjUnsupported = createMockMjModule({
        ngeom: 3,
        geomTypes: [2, 1, 6], // sphere, hfield (unsupported), box
        geomNames: ['g0', 'g1', 'g2'],
      })
      const eng = new MuJoCoEngine(mjUnsupported)
      eng.loadModel(SIMPLE_XML)
      const visuals = eng.getVisualGeoms()
      // hfield (type 1) should be skipped
      expect(visuals).toHaveLength(2)
      expect(visuals[0].type).toBe('sphere')
      expect(visuals[1].type).toBe('box')
    })
  })

  // -----------------------------------------------------------------------
  // SimState
  // -----------------------------------------------------------------------

  describe('getSimState', () => {
    it('returns a complete state snapshot', () => {
      engine.loadModel(SIMPLE_XML)
      engine.stepN(5)
      const state = engine.getSimState(false)

      expect(state.time).toBeCloseTo(0.01)
      expect(state.timestep).toBeCloseTo(0.002)
      expect(state.paused).toBe(false)
      expect(state.bodies).toHaveLength(3)
      expect(state.joints).toHaveLength(2)
      expect(state.sensors).toHaveLength(1)
      expect(state.contacts).toEqual([])
      expect(state.wallTime).toBeGreaterThan(0)
    })

    it('respects the paused flag', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.getSimState(true).paused).toBe(true)
      expect(engine.getSimState(false).paused).toBe(false)
    })

    it('includes sites when the model has site data', () => {
      engine.loadModel(SIMPLE_XML)
      const state = engine.getSimState(false)
      expect(state.sites).toBeDefined()
      expect(state.sites!).toHaveLength(1)
    })

    it('omits sites when the model has none', () => {
      const mjZero = createMockMjModule({ nsensor: 0, nsite: 0 })
      const eng = new MuJoCoEngine(mjZero)
      eng.loadModel(SIMPLE_XML)
      const state = eng.getSimState(false)
      expect(state.sites).toBeUndefined()
    })
  })

  // -----------------------------------------------------------------------
  // Descriptor
  // -----------------------------------------------------------------------

  describe('getDescriptor', () => {
    it('returns a descriptor with the given id', () => {
      engine.loadModel(SIMPLE_XML)
      const desc = engine.getDescriptor('my-robot')
      expect(desc.id).toBe('my-robot')
      expect(desc.bodyCount).toBe(3)
    })
  })

  // -----------------------------------------------------------------------
  // Dispose
  // -----------------------------------------------------------------------

  describe('dispose', () => {
    it('marks the engine as unloaded', () => {
      engine.loadModel(SIMPLE_XML)
      expect(engine.isLoaded()).toBe(true)
      engine.dispose()
      expect(engine.isLoaded()).toBe(false)
    })

    it('causes subsequent operations to throw', () => {
      engine.loadModel(SIMPLE_XML)
      engine.dispose()
      expect(() => engine.step()).toThrow('No model loaded')
      expect(() => engine.getBodyStates()).toThrow('No model loaded')
    })
  })
})

// ---------------------------------------------------------------------------
// Mock factory unit tests
// ---------------------------------------------------------------------------

describe('createMockModel', () => {
  it('creates a model with default dimensions', () => {
    const model = createMockModel()
    expect(model.nbody).toBe(3)
    expect(model.njnt).toBe(2)
    expect(model.ngeom).toBe(2)
    expect(model.nu).toBe(1)
    expect(model.nsensor).toBe(1)
  })

  it('respects custom dimensions', () => {
    const model = createMockModel({ nbody: 5, njnt: 4, nu: 3 })
    expect(model.nbody).toBe(5)
    expect(model.njnt).toBe(4)
    expect(model.nu).toBe(3)
  })

  it('computes nq from joint types', () => {
    // free(7) + hinge(1) = 8
    const model = createMockModel({ njnt: 2, jointTypes: [0, 3] })
    expect(model.nq).toBe(8)
  })

  it('computes nv from joint types', () => {
    // free(6) + ball(3) = 9
    const model = createMockModel({ njnt: 2, jointTypes: [0, 1] })
    expect(model.nv).toBe(9)
  })
})
