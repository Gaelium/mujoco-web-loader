/**
 * T1 S1c — the model-load and per-tick episode pipeline, shared verbatim by
 * the physics worker (browser) and the headless episode drivers (Node).
 * Extracted from SimWorker.worker.ts so the two deployments cannot drift —
 * the worker remains the reference consumer; the bodies below are the
 * worker's originals with module state (engine, meshBounds) turned into
 * parameters.
 *
 * Pure functions over (engine, config): no worker globals, no postMessage,
 * no DOM. (The arm↔cloth bridge functions that once lived here stay with
 * the host app — OPEN-SOURCE-SPLIT.md cut, SPLIT-NOTES D-SP-3.)
 */

import type { MjModule } from './MuJoCoEngine'
import { MuJoCoEngine } from './MuJoCoEngine'
import type { TrajectoryRunner } from './TrajectoryRunner'
import type { EnvironmentConfig } from '../types/environment'
import { primitiveDiagInertia } from './SceneComposer'
import {
  dumpCompiledMJCF,
  findRootLinkInUrdf,
  injectMJCFSensors,
} from './sensorInjection'
import {
  injectMJCFActuators,
  injectMJCFFixedBaseWeld,
  DEFAULT_FIXED_BASE_WELD_NAME,
  type InjectableJoint,
} from './actuatorInjection'
import type {
  URDFSensorConfig,
  URDFActuatorConfig,
  ActuatorDescriptor,
  RobotDescriptor,
  ModelSignature,
  Vec3,
  MjQuat,
} from '../types/simulation'

const FIXED_BASE_WELD_NAME = DEFAULT_FIXED_BASE_WELD_NAME

export interface MeshBoundsInfo {
  /** Full extents [dx, dy, dz] for URDF `<box size="..."/>`. */
  readonly extents: readonly [number, number, number]
  /** Geometric center [cx, cy, cz] in mesh coordinates. */
  readonly center: readonly [number, number, number]
}

/**
 * Compute the axis-aligned bounding box of a binary STL mesh.
 *
 * Binary STL layout:
 *   80 bytes header
 *   4 bytes  triangle count (uint32 LE)
 *   per triangle: 12B normal + 36B vertices (3×3 float32 LE) + 2B attrib
 *
 * @returns Extents and center, or null if the file can't be parsed
 */
export function computeSTLBounds(data: Uint8Array): MeshBoundsInfo | null {
  // Try binary STL first, then ASCII
  return computeBinarySTLBounds(data) ?? computeASCIISTLBounds(data)
}

/** Parse binary STL: 80B header + 4B count + 50B per triangle. */
function computeBinarySTLBounds(data: Uint8Array): MeshBoundsInfo | null {
  if (data.byteLength < 84) return null

  const view = new DataView(data.buffer, data.byteOffset, data.byteLength)
  const numTriangles = view.getUint32(80, true)
  const expectedSize = 84 + numTriangles * 50

  if (data.byteLength !== expectedSize || numTriangles === 0) return null

  let minX = Infinity, minY = Infinity, minZ = Infinity
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity

  for (let i = 0; i < numTriangles; i++) {
    const base = 84 + i * 50 + 12 // skip normal
    for (let v = 0; v < 3; v++) {
      const off = base + v * 12
      const x = view.getFloat32(off, true)
      const y = view.getFloat32(off + 4, true)
      const z = view.getFloat32(off + 8, true)
      if (x < minX) minX = x
      if (y < minY) minY = y
      if (z < minZ) minZ = z
      if (x > maxX) maxX = x
      if (y > maxY) maxY = y
      if (z > maxZ) maxZ = z
    }
  }

  if (!isFinite(minX)) return null

  return {
    extents: [
      Math.max(maxX - minX, 0.001),
      Math.max(maxY - minY, 0.001),
      Math.max(maxZ - minZ, 0.001),
    ],
    center: [
      (minX + maxX) / 2,
      (minY + maxY) / 2,
      (minZ + maxZ) / 2,
    ],
  }
}

/** Parse ASCII STL: `solid ... vertex x y z ... endsolid`. */
function computeASCIISTLBounds(data: Uint8Array): MeshBoundsInfo | null {
  const text = new TextDecoder().decode(data.slice(0, Math.min(data.byteLength, 5)))
  if (!text.startsWith('solid')) return null

  const full = new TextDecoder().decode(data)
  const vertexRe = /vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)/g

  let minX = Infinity, minY = Infinity, minZ = Infinity
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity
  let count = 0
  let m: RegExpExecArray | null

  while ((m = vertexRe.exec(full)) !== null) {
    const x = parseFloat(m[1]), y = parseFloat(m[2]), z = parseFloat(m[3])
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue
    if (x < minX) minX = x; if (y < minY) minY = y; if (z < minZ) minZ = z
    if (x > maxX) maxX = x; if (y > maxY) maxY = y; if (z > maxZ) maxZ = z
    count++
  }

  if (count === 0 || !isFinite(minX)) return null

  return {
    extents: [
      Math.max(maxX - minX, 0.001),
      Math.max(maxY - minY, 0.001),
      Math.max(maxZ - minZ, 0.001),
    ],
    center: [
      (minX + maxX) / 2,
      (minY + maxY) / 2,
      (minZ + maxZ) / 2,
    ],
  }
}
/**
 * Convert a MuJoCo quaternion [w,x,y,z] to URDF RPY string "roll pitch yaw".
 */
function quatToRPY(q: readonly [number, number, number, number]): string {
  const [w, x, y, z] = q
  const sinr = 2 * (w * x + y * z)
  const cosr = 1 - 2 * (x * x + y * y)
  const roll = Math.atan2(sinr, cosr)
  const sinp = 2 * (w * y - z * x)
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp)
  const siny = 2 * (w * z + x * y)
  const cosy = 1 - 2 * (y * y + z * z)
  const yaw = Math.atan2(siny, cosy)
  return `${roll} ${pitch} ${yaw}`
}

/**
 * Pre-process a URDF for the @mujoco/mujoco WASM build.
 *
 * MuJoCo's URDF parser supports a subset of URDF and has WASM-specific
 * limitations that don't apply to the desktop build. This function applies
 * the workarounds needed to load real-world URDFs without hanging or crashing.
 *
 * The preprocessing steps, in order:
 *
 * 1. Inject `<mujoco><compiler balanceinertia="true"/></mujoco>` as a child
 *    of `<robot>`. MuJoCo's URDF parser recognizes this as compiler hints.
 *    The balanceinertia flag prevents fatal errors from URDFs with degenerate
 *    inertias (very common in hand-authored URDFs).
 *
 * 2. Replace `<mesh>` geometry inside `<visual>` and `<collision>` blocks
 *    with bounding-box `<box>` approximations. MuJoCo's WASM URDF parser
 *    hangs indefinitely when loading STL files from the Emscripten VFS — a
 *    known issue with the @mujoco/mujoco WASM build. Bounding boxes are
 *    computed upstream from the raw STL data and substituted here. Three.js
 *    renders the real meshes independently via URDFVisualLoader, so this is
 *    visually lossless. The substitution preserves `<origin>` offsets and
 *    applies any `<mesh scale>` attribute.
 *
 * 3. Synthesize `<collision>` geoms for any link that only has `<visual>`.
 *    URDFs commonly omit collision geometry, but MuJoCo needs collision
 *    shapes to generate contacts. We clone the visual geometry as collision.
 *
 * 4. Inject a floating base joint so the robot can be repositioned at
 *    runtime via setQpos on the free joint's qpos slot.
 *
 * 5. Inject a ground plane and any environment primitives (tables, shelves,
 *    crates) as additional links + joints in the same URDF. MuJoCo's
 *    `<include>` directive treats included files as MJCF, not URDF, so we
 *    cannot compose environment + robot at the MJCF level — everything has
 *    to live in the URDF.
 *
 * To extend this for sensor support: add a step that injects
 * `<mujoco><sensor>...</sensor></mujoco>` blocks as children of `<robot>`.
 *
 * @param urdfXml - Raw URDF XML string (after stripUnsupportedURDFElements)
 * @param env - Current environment config (ground size, primitives)
 * @returns Modified URDF ready for MuJoCo loading
 */
export function preprocessURDF(
  urdfXml: string,
  env: EnvironmentConfig,
  meshBounds: ReadonlyMap<string, MeshBoundsInfo>,
): string {
  let xml = urdfXml

  // Inject MuJoCo compiler directives into the URDF.
  // MuJoCo's URDF parser recognizes an embedded <mujoco> element as a
  // child of <robot> for specifying compiler settings.
  // balanceinertia="true" prevents errors from URDFs with zero/degenerate inertias.
  if (!xml.includes('<mujoco>')) {
    const robotOpenMatch = xml.match(/<robot[^>]*>/)
    if (robotOpenMatch) {
      const insertPos = robotOpenMatch.index! + robotOpenMatch[0].length
      const mujocoBlock = `\n  <mujoco>\n    <compiler balanceinertia="true"/>\n  </mujoco>`
      xml = xml.slice(0, insertPos) + mujocoBlock + xml.slice(insertPos)
    }
  }

  // Replace <visual> and <collision> blocks that contain mesh geometry with
  // bounding-box approximations. MuJoCo's WASM URDF parser hangs when loading
  // STL/OBJ files from the Emscripten VFS. Three.js renders visual meshes
  // independently, so MuJoCo doesn't need them. We compute axis-aligned
  // bounding boxes from the raw STL data and substitute <box> elements.
  //
  // The replacement also adjusts <origin> to account for the mesh's geometric
  // center offset — without this the boxes float or clip because URDF <box>
  // is centered at the collision frame origin, but the mesh vertices may be
  // offset from that origin.
  //
  // Handles both self-closing (<mesh .../>) and paired (<mesh ...></mesh>)
  // elements, and both single- and double-quoted attributes.
  xml = xml.replace(/<(visual|collision)(\s[^>]*)?>[\s\S]*?<\/\1>/g, (match, tag: string) => {
    // Match self-closing <mesh .../> or paired <mesh ...>...</mesh>
    const meshMatch = match.match(/<mesh\s[^>]*(?:\/>|>[\s\S]*?<\/mesh>)/)
    if (!meshMatch) return match // no mesh in this block, keep as-is

    // Support both single and double quoted filename/scale attributes
    const fnMatch = meshMatch[0].match(/filename=["']([^"']*)["']/)
    const scMatch = meshMatch[0].match(/scale=["']([^"']*)["']/)

    // Look up precomputed bounds + center by basename
    let size: [number, number, number] = [0.05, 0.05, 0.05]
    let center: [number, number, number] = [0, 0, 0]
    if (fnMatch) {
      const basename = fnMatch[1].split('/').pop() ?? fnMatch[1]
      const info = meshBounds.get(basename)
      if (info) {
        size = [...info.extents]
        center = [...info.center]
      }
    }

    // Apply mesh scale attribute if present
    if (scMatch) {
      const s = scMatch[1].trim().split(/\s+/).map(Number)
      if (s.length >= 3 && s.every(isFinite)) {
        size[0] *= s[0]; size[1] *= s[1]; size[2] *= s[2]
        center[0] *= s[0]; center[1] *= s[1]; center[2] *= s[2]
      }
    }

    // Parse existing <origin> from the block (if any) and combine with
    // the mesh center offset so the box sits where the mesh actually was.
    const originMatch = match.match(/<origin\s+([\s\S]*?)\/>/)
    let ox = 0, oy = 0, oz = 0
    let rpy = '0 0 0'
    if (originMatch) {
      const xyzMatch = originMatch[1].match(/xyz=["']([^"']*)["']/)
      const rpyMatch = originMatch[1].match(/rpy=["']([^"']*)["']/)
      if (xyzMatch) {
        const p = xyzMatch[1].trim().split(/\s+/).map(Number)
        if (p.length >= 3 && p.every(isFinite)) { ox = p[0]; oy = p[1]; oz = p[2] }
      }
      if (rpyMatch) rpy = rpyMatch[1]
    }

    // Offset the origin by the mesh's geometric center
    const nx = ox + center[0]
    const ny = oy + center[1]
    const nz = oz + center[2]

    const fmt = (v: number): string => v.toFixed(6)
    return `<${tag}>
      <origin xyz="${fmt(nx)} ${fmt(ny)} ${fmt(nz)}" rpy="${rpy}"/>
      <geometry>
        <box size="${fmt(size[0])} ${fmt(size[1])} ${fmt(size[2])}"/>
      </geometry>
    </${tag}>`
  })

  // Step 3: Synthesize collision geometry for visual-only links.
  //
  // Many URDFs (especially those authored for visualization tools like
  // RViz rather than physics simulators) define <visual> blocks on links
  // but omit <collision>. Without collision geoms, MuJoCo creates the
  // body but generates zero contacts for it — the link passes through
  // everything, including the ground and environment objects.
  //
  // For each link that has <visual> but no <collision>, we clone the first
  // visual block's <geometry> and <origin> into a new <collision> block.
  // This is approximate but correct for the common case where visual and
  // collision shapes are intended to be the same (e.g. SO-100 gripper/jaw).
  xml = xml.replace(/<link\s+[^>]*name=["']([^"']*)["'][^>]*>([\s\S]*?)<\/link>/g, (match, _name: string, body: string) => {
    if (body.includes('<collision>')) return match // already has collision
    const visualMatch = body.match(/<visual>([\s\S]*?)<\/visual>/)
    if (!visualMatch) return match // no visual either
    // Extract the geometry and origin from the visual block
    const geomMatch = visualMatch[1].match(/<geometry>[\s\S]*?<\/geometry>/)
    const originMatch = visualMatch[1].match(/<origin\s[\s\S]*?\/>/)
    if (!geomMatch) return match
    const collisionBlock = `\n    <collision>${originMatch ? '\n      ' + originMatch[0] : ''}\n      ${geomMatch[0]}\n    </collision>`
    return match.replace('</link>', `${collisionBlock}\n  </link>`)
  })

  // Parse minimally to find the root link and check for existing floating joints.
  // Supports both single- and double-quoted attributes and any attribute order.
  const jointMatches = [...xml.matchAll(/<joint\s[^>]*type=["']([^"']*)["'][^>]*>/g)]
  const hasFloating = jointMatches.some(m => m[1] === 'floating')

  // Find all child links (links that are children of some joint).
  // Handles self-closing <child link="..."/> and paired <child link="..."></child>.
  const childLinkMatches = [...xml.matchAll(/<child\s+link=["']([^"']*)["']\s*(?:\/>|>[\s\S]*?<\/child>)/g)]
  const childLinks = new Set(childLinkMatches.map(m => m[1]))

  // Find all link names (supports single/double quotes and extra attributes)
  const linkMatches = [...xml.matchAll(/<link\s[^>]*name=["']([^"']*)["']/g)]
  const allLinks = linkMatches.map(m => m[1])

  // Root link = link that is not a child of any joint
  const rootLink = allLinks.find(l => !childLinks.has(l))
  if (!rootLink) return xml

  let injection = ''

  // 1. Floating base (if not already present)
  if (!hasFloating) {
    injection += `
  <link name="_simulo_world"/>
  <joint name="_simulo_floating_base" type="floating">
    <parent link="_simulo_world"/>
    <child link="${rootLink}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>`
  }

  const worldLink = hasFloating ? rootLink : '_simulo_world'
  const gs = env.ground.size

  // 2. Ground plane as a large thin box
  if (env.ground.type !== 'none') {
    injection += `
  <link name="_simulo_ground">
    <collision>
      <geometry>
        <box size="${gs * 2} ${gs * 2} 0.02"/>
      </geometry>
      <origin xyz="0 0 -0.01" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
  <joint name="_simulo_ground_fix" type="fixed">
    <parent link="${worldLink}"/>
    <child link="_simulo_ground"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>`
  }

  // 3. Environment primitives (tables, shelves, crates, etc.)
  for (const prim of env.primitives) {
    const linkName = `_simulo_prim_${prim.id}`
    const [px, py, pz] = prim.position
    const rpy = quatToRPY(prim.quaternion)

    // URDF uses full extents; ScenePrimitive.size has MuJoCo half-extents
    // for box, radius for sphere/cylinder/capsule.
    let geometryXml: string
    switch (prim.shape) {
      case 'box':
        geometryXml = `<box size="${prim.size[0] * 2} ${prim.size[1] * 2} ${prim.size[2] * 2}"/>`
        break
      case 'sphere':
        geometryXml = `<sphere radius="${prim.size[0]}"/>`
        break
      case 'cylinder':
        geometryXml = `<cylinder radius="${prim.size[0]}" length="${prim.size[1] * 2}"/>`
        break
      case 'capsule':
        // URDF doesn't have capsule; approximate as cylinder
        geometryXml = `<cylinder radius="${prim.size[0]}" length="${prim.size[1] * 2}"/>`
        break
      case 'mesh':
        // Imported mesh: approximate with bounding box
        geometryXml = `<box size="${prim.size[0] * 2} ${prim.size[1] * 2} ${prim.size[2] * 2}"/>`
        break
    }

    // Dynamic objects (mass > 0) need a floating joint so they can move
    // under gravity. Static objects use a fixed joint.
    const isDynamic = prim.mass > 0
    const jointType = isDynamic ? 'floating' : 'fixed'
    const massVal = isDynamic ? prim.mass : 0
    const [ixx, iyy, izz] = isDynamic ? primitiveDiagInertia(prim) : [0, 0, 0]

    injection += `
  <link name="${linkName}">
    <collision>
      <geometry>
        ${geometryXml}
      </geometry>
    </collision>
    <inertial>
      <mass value="${massVal}"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="${ixx}" ixy="0" ixz="0" iyy="${iyy}" iyz="0" izz="${izz}"/>
    </inertial>
  </link>
  <joint name="${linkName}_jnt" type="${jointType}">
    <parent link="${worldLink}"/>
    <child link="${linkName}"/>
    <origin xyz="${px} ${py} ${pz}" rpy="${rpy}"/>
  </joint>`
  }

  return xml.replace('</robot>', `${injection}\n</robot>`)
}
/**
 * Enable or disable fixed-base behavior via the weld equality constraint.
 *
 * When enabling, the weld's relpose is set to the requested pose and the
 * freejoint qpos is snapped to match, then velocities are zeroed so the base
 * starts at rest. When disabling, the weld is deactivated and the robot
 * becomes a normal free-floating body.
 *
 * @returns true if the weld-based toggle succeeded, false if the model lacks
 *          the expected weld equality (caller may fall back to legacy behavior).
 */
export function applyFixedBase(
  eng: MuJoCoEngine,
  fixedBase: boolean,
  pos: Vec3,
  quat: MjQuat,
): boolean {
  const weldId = eng.findEqualityByName(FIXED_BASE_WELD_NAME)
  if (weldId < 0) return false

  if (fixedBase) {
    eng.setWeldRelpose(weldId, pos, quat)
    const addr = eng.getFirstFreeJointQposAddr()
    if (addr >= 0) {
      eng.setQpos(addr, [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]])
      eng.zeroFreeJointVel()
    }
  }

  return eng.setEqualityActive(weldId, fixedBase)
}

/**
 * Collect the movable joints of the loaded model in ascending joint-id order,
 * each with its limit range. Skips free/ball joints and helper joints injected
 * by preprocessURDF (`_simulo_*`). The order is the contract for actuator
 * `data.ctrl` indices and the servo profile mapping, so it must be stable.
 */
export function buildInjectableJoints(eng: MuJoCoEngine): InjectableJoint[] {
  const joints: InjectableJoint[] = []
  for (const j of eng.getDescriptor('_').joints) {
    if (j.type !== 'hinge' && j.type !== 'slide') continue
    if (j.name.startsWith('_simulo_')) continue
    joints.push({ name: j.name, range: eng.getJointRange(j.name) })
  }
  return joints
}

/**
 * URDF two-pass injection: dump the just-compiled MJCF, add position actuators
 * + per-joint tuning, a fixed-base weld equality, and IMU/lidar sensors, then
 * recompile. MuJoCo's URDF parser drops all three of these element classes, so
 * this MUST run on the dumped MJCF (where they are native). The weld is always
 * injected (inactive by default) so fixed/free can be toggled at runtime like
 * the MJCF path. Mirrors the cost model of the original sensor-only Pass 2.
 *
 * @returns the recompiled model's descriptor
 * @throws if the dump/inject/recompile fails (caller decides how loud to be)
 */
export function applyURDFPass2(
  eng: MuJoCoEngine,
  mj: MjModule,
  originalUrdf: string,
  actuatorConfig: URDFActuatorConfig | undefined,
  sensorConfig: URDFSensorConfig | undefined,
  vfsPath: string,
): RobotDescriptor {
  const compiledMjcf = dumpCompiledMJCF(mj, eng.getRawModel())
  const rootLink = findRootLinkInUrdf(originalUrdf)
  if (!rootLink) throw new Error('could not detect root link in URDF')

  let edited = compiledMjcf
  // 1. Position actuators (no-op when mode === 'none').
  edited = injectMJCFActuators(edited, buildInjectableJoints(eng), actuatorConfig ?? {})
  // 2. Fixed-base weld (always present, inactive — runtime-toggleable).
  edited = injectMJCFFixedBaseWeld(edited, rootLink, FIXED_BASE_WELD_NAME)
  // 3. IMU + optional lidar sensors (defaults: IMU on).
  const sc = sensorConfig ?? {}
  if (sc.imu !== false || sc.lidar === true) {
    edited = injectMJCFSensors(edited, rootLink, sc)
  }

  return eng.loadModel(edited, vfsPath)
}

/**
 * Drive the model to a home joint-space pose: set each movable joint's qpos and
 * its position-actuator target to `homeQpos[i]` (ascending joint-id order), so
 * the arm starts/returns to a designed rest pose and the position servos hold
 * it. No-op for joints beyond `homeQpos`'s length.
 */
export function applyHomePose(eng: MuJoCoEngine, homeQpos: readonly number[]): void {
  const joints = buildInjectableJoints(eng)
  const n = Math.min(joints.length, homeQpos.length)
  for (let i = 0; i < n; i++) {
    const addr = eng.getJointQposAddr(joints[i].name)
    if (addr >= 0) eng.setQpos(addr, [homeQpos[i]])
    eng.setControl(i, homeQpos[i])
  }
  eng.forward()
}

/*
 * (cut) The M4 G1/G2 arm→cloth bridge — ClothBridgeState, resolveClothBridge,
 * updateClothBridge — moved to the host app's `src/physics/clothBridge.ts`
 * (OPEN-SOURCE-SPLIT.md cut; SPLIT-NOTES D-SP-3).
 */

/**
 * T1 S2a — the compiled model's identity, used by the composed-scene
 * export and its recompile-equivalence gate: a dumped scene.xml must
 * recompile (WASM or native) to the same signature as the live session.
 */
/** Decode `mj_version()`'s integer (e.g. 3007000) to "3.7.0". */
export function formatMujocoVersion(version: number): string {
  const major = Math.floor(version / 1_000_000)
  const minor = Math.floor(version / 1_000) % 1_000
  const patch = version % 1_000
  return `${major}.${minor}.${patch}`
}

export function modelSignature(eng: MuJoCoEngine): ModelSignature {
  const model = eng.getRawModel()
  const descriptor = eng.getDescriptor('_')
  return {
    nq: model.nq,
    nv: model.nv,
    nu: descriptor.actuators.length,
    actuatorNames: descriptor.actuators.map((a) => a.name),
    timestep: eng.getTimestep(),
  }
}

/**
 * M4 G3 (extracted) — one sim-tick of a running key-pose trajectory: advance
 * the rate-limited command by `scaledDtS` (the worker multiplies the cloth's
 * tier-B `trajectoryPaceScale()` into this dt so arm, colliders and pins slow
 * coherently), clamp each ctrl value to its actuator range, and write it to
 * the engine. Returns true when the trajectory has completed.
 */
export function stepTrajectoryTick(
  eng: MuJoCoEngine,
  runner: TrajectoryRunner,
  actuators: readonly ActuatorDescriptor[],
  scaledDtS: number,
): { readonly done: boolean; readonly ctrl: readonly number[] } {
  const result = runner.step(scaledDtS)
  const applied: number[] = []
  for (let i = 0; i < Math.min(result.ctrl.length, actuators.length); i++) {
    const range = actuators[i]?.ctrlRange
    const v =
      range && range[0] < range[1]
        ? Math.min(range[1], Math.max(range[0], result.ctrl[i]))
        : result.ctrl[i]
    eng.setControl(i, v)
    applied.push(v)
  }
  return { done: result.done, ctrl: applied }
}
