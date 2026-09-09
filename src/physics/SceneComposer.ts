/**
 * Generates MJCF XML for the combined simulation scene.
 *
 * SceneComposer takes an EnvironmentConfig and zero or more robot URDF
 * strings and produces a single MJCF XML document that MuJoCo can compile.
 * It uses string-based XML generation (not mjSpec) for simplicity and
 * testability.
 *
 * Runs exclusively inside the physics Web Worker.
 */

import type { EnvironmentConfig, ScenePrimitive } from '../types/environment'
import type { Vec3, MjQuat } from '../types/simulation'

// ---------------------------------------------------------------------------
// Robot placement descriptor
// ---------------------------------------------------------------------------

/** Describes a robot to include in the composed scene. */
export interface RobotPlacement {
  /** Unique model id. */
  readonly id: string
  /** Path to the URDF/MJCF file in the VFS (e.g. '/robots/arm.urdf'). */
  readonly vfsPath: string
  /** World-frame position [x, y, z]. */
  readonly position: Vec3
  /** World-frame orientation as MuJoCo quaternion [w, x, y, z]. */
  readonly quaternion: MjQuat
}

// ---------------------------------------------------------------------------
// XML helpers
// ---------------------------------------------------------------------------

/**
 * Escape special characters for XML attribute values.
 * @param s - Raw string
 * @returns Escaped string safe for XML attributes
 */
export function escapeXmlAttr(s: string): string {
  return s
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&apos;')
}

/**
 * Format a Vec3 as a space-separated string for MJCF attributes.
 * @param v - [x, y, z] tuple
 * @returns "x y z" string
 */
export function vec3Attr(v: Vec3): string {
  return `${v[0]} ${v[1]} ${v[2]}`
}

/**
 * Format an MjQuat as a space-separated string for MJCF attributes.
 * @param q - [w, x, y, z] quaternion
 * @returns "w x y z" string
 */
export function quatAttr(q: MjQuat): string {
  return `${q[0]} ${q[1]} ${q[2]} ${q[3]}`
}

/**
 * Format an RGBA tuple as a space-separated string.
 * @param rgba - [r, g, b, a] values in [0-1]
 * @returns "r g b a" string
 */
export function rgbaAttr(rgba: readonly [number, number, number, number]): string {
  return `${rgba[0]} ${rgba[1]} ${rgba[2]} ${rgba[3]}`
}

// ---------------------------------------------------------------------------
// MJCF fragment generators
// ---------------------------------------------------------------------------

/**
 * Generate the <option> element from physics params.
 * @param config - Environment configuration
 * @returns MJCF <option> XML string
 */
export function generateOptionXml(config: EnvironmentConfig): string {
  const p = config.physics
  return `  <option timestep="${p.timestep}" iterations="${p.iterations}" gravity="${vec3Attr(p.gravity)}" />`
}

/**
 * Generate the ground plane geom inside worldbody.
 * @param config - Environment configuration
 * @returns MJCF ground geom XML string, or empty string for 'none' ground
 */
export function generateGroundXml(config: EnvironmentConfig): string {
  const g = config.ground
  if (g.type === 'none') return ''
  const size = `${g.size} ${g.size} 0.01`
  return `    <geom name="ground" type="plane" size="${size}" rgba="${rgbaAttr(g.color1)}" friction="${config.physics.friction} 0.005 0.0001"/>`
}

/**
 * Compute the diagonal inertia of a dynamic primitive from its mass and
 * shape (uniform density). Capsules use the cylinder formula — the same
 * approximation the URDF injection path already applies to their
 * geometry; meshes use their bounding box, matching their collision
 * geometry.
 * @param prim - Scene primitive definition (mass > 0)
 * @returns [ixx, iyy, izz] in kg·m²
 */
export function primitiveDiagInertia(prim: ScenePrimitive): Vec3 {
  const m = prim.mass
  const [s0, s1, s2] = prim.size
  switch (prim.shape) {
    case 'sphere': {
      const i = (2 / 5) * m * s0 * s0
      return [i, i, i]
    }
    case 'cylinder':
    case 'capsule': {
      // size = [radius, half-height]; full height h = 2*s1
      const r = s0
      const ixx = (m * (3 * r * r + 4 * s1 * s1)) / 12
      return [ixx, ixx, (m * r * r) / 2]
    }
    case 'box':
    case 'mesh':
      // size = half-extents; I_xx = m/12 * (Y² + Z²) with full extents
      return [
        (m / 3) * (s1 * s1 + s2 * s2),
        (m / 3) * (s0 * s0 + s2 * s2),
        (m / 3) * (s0 * s0 + s1 * s1),
      ]
  }
}

/**
 * Generate MJCF XML for a single scene primitive.
 * @param prim - Scene primitive definition
 * @returns MJCF body+geom XML string
 */
export function generatePrimitiveXml(prim: ScenePrimitive): string {
  const isStatic = prim.mass === 0
  const lines: string[] = []
  lines.push(`    <body name="${escapeXmlAttr(prim.name)}" pos="${vec3Attr(prim.position)}" quat="${quatAttr(prim.quaternion)}">`)
  if (!isStatic) {
    const inertia = primitiveDiagInertia(prim)
    lines.push(`      <freejoint/>`)
    lines.push(`      <inertial pos="0 0 0" mass="${prim.mass}" diaginertia="${vec3Attr(inertia)}"/>`)
  }
  // For mesh shapes, use box as collision approximation (bounding box)
  const geomType = prim.shape === 'mesh' ? 'box' : prim.shape
  lines.push(`      <geom name="${escapeXmlAttr(prim.id)}" type="${geomType}" size="${vec3Attr(prim.size)}" rgba="${rgbaAttr(prim.rgba)}"/>`)
  lines.push(`    </body>`)
  return lines.join('\n')
}

/**
 * Generate a top-level <include> element for a robot model.
 *
 * The include is placed at the <mujoco> level (NOT inside <worldbody>)
 * because MuJoCo's URDF-to-MJCF conversion produces its own <worldbody>.
 * When the include is at the top level, MuJoCo merges the included
 * worldbody contents into the main worldbody — this is the only way
 * to combine environment geometry with a URDF robot in one model.
 *
 * @param robot - Robot placement descriptor
 * @returns MJCF include XML string (top-level indentation)
 */
export function generateRobotIncludeXml(robot: RobotPlacement): string {
  return `  <include file="${escapeXmlAttr(robot.vfsPath)}"/>`
}

// ---------------------------------------------------------------------------
// Full scene composition
// ---------------------------------------------------------------------------

/**
 * Compose a complete MJCF XML document from an environment config and
 * optional robot placements.
 * @param config - Environment configuration
 * @param robots - Robot placements to include in the scene (default [])
 * @returns Complete MJCF XML string ready for mj_loadXML
 */
export function composeScene(
  config: EnvironmentConfig,
  robots: readonly RobotPlacement[] = [],
): string {
  const lines: string[] = []

  lines.push('<mujoco model="robot-sim-scene">')
  lines.push('')

  // Compiler settings — meshdir points to /working where load-model writes
  // mesh assets. This ensures URDF mesh references (e.g. "assets/Base.stl")
  // resolve to /working/assets/Base.stl.
  lines.push('  <compiler angle="radian" meshdir="/working"/>')
  lines.push('')

  // Physics options
  lines.push(generateOptionXml(config))
  lines.push('')

  // Default settings for friction
  lines.push('  <default>')
  lines.push(`    <geom friction="${config.physics.friction} 0.005 0.0001"/>`)
  lines.push('  </default>')
  lines.push('')

  // Worldbody
  lines.push('  <worldbody>')

  // Ground
  const groundXml = generateGroundXml(config)
  if (groundXml) {
    lines.push(groundXml)
  }

  // Light (for MuJoCo rendering, also used by Three.js hints)
  const l = config.lighting
  lines.push(`    <light name="main_light" pos="${vec3Attr(l.directionalPosition)}" dir="0 0 -1" diffuse="1 1 1" specular="0.3 0.3 0.3" castshadow="${l.shadows}"/>`)
  lines.push('')

  // Static/dynamic primitives
  for (const prim of config.primitives) {
    lines.push(generatePrimitiveXml(prim))
  }

  lines.push('  </worldbody>')
  lines.push('')

  // Robot includes — placed at <mujoco> level so MuJoCo can merge
  // the URDF's converted worldbody into the scene's worldbody.
  for (const robot of robots) {
    lines.push(generateRobotIncludeXml(robot))
  }

  lines.push('')
  lines.push('</mujoco>')

  return lines.join('\n')
}

/**
 * Compose a standalone MJCF wrapper that includes a single URDF file.
 * This is used when loading a robot directly (without an environment scene).
 * @param urdfVfsPath - Path to the URDF in the VFS
 * @param position - World-frame position (default [0,0,0])
 * @param quaternion - World-frame orientation (default identity)
 * @returns MJCF XML string
 */
export function composeStandaloneRobot(
  urdfVfsPath: string,
  position: Vec3 = [0, 0, 0],
  quaternion: MjQuat = [1, 0, 0, 0],
): string {
  const lines: string[] = []
  lines.push('<mujoco model="standalone-robot">')
  lines.push('  <compiler angle="radian" meshdir="/meshes"/>')
  lines.push(`  <option timestep="0.002" gravity="0 0 -9.81" />`)
  lines.push('  <worldbody>')
  lines.push(`    <geom name="ground" type="plane" size="10 10 0.01" rgba="0.4 0.4 0.4 1"/>`)
  lines.push(`    <body name="robot_base" pos="${vec3Attr(position)}" quat="${quatAttr(quaternion)}">`)
  lines.push(`      <include file="${escapeXmlAttr(urdfVfsPath)}"/>`)
  lines.push('    </body>')
  lines.push('  </worldbody>')
  lines.push('</mujoco>')
  return lines.join('\n')
}
