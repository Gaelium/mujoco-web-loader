/**
 * Sensor injection for URDF uploads.
 *
 * MuJoCo's URDF parser silently drops `<sensor>`, `<site>`, and `<worldbody>`
 * declarations placed inside the `<mujoco>` extension block — they are
 * tolerated but never compiled (verified empirically; see Decision Log
 * D-090 in IMPLEMENTATION_PLAN.md). This blocks the natural approach of
 * declaring sensors directly in the URDF.
 *
 * Workaround: a two-pass load.
 *   1. Compile the URDF normally (existing pipeline).
 *   2. Use `mj_saveLastXML` to dump the compiled MJCF.
 *   3. DOM-edit the MJCF to add `<site>`s on the root body and a `<sensor>`
 *      block referencing them.
 *   4. Recompile the modified MJCF — sensor declarations work natively in
 *      MJCF, so the second pass produces a model with `nsensor > 0`.
 *
 * Cost: one extra `mj_loadXML` call per URDF upload. Acceptable — bypassed
 * entirely on cache hits, and the loading overlay already covers it.
 *
 * This module runs only inside the physics Web Worker.
 */

import type { URDFSensorConfig } from '../types/simulation'
import type { MjModule, MjModelInstance } from './MuJoCoEngine'

/**
 * Find the root link name in a URDF.
 *
 * Root link = the link that is not a child of any joint. Mirrors the
 * detection used in `preprocessURDF` so injection targets the same body
 * the floating-base joint was attached to.
 *
 * @param urdfXml - Raw URDF XML string (after stripUnsupportedURDFElements)
 * @returns Root link name, or null if the URDF is malformed
 */
export function findRootLinkInUrdf(urdfXml: string): string | null {
  const childMatches = [...urdfXml.matchAll(/<child\s+link=["']([^"']*)["']\s*(?:\/>|>[\s\S]*?<\/child>)/g)]
  const childLinks = new Set(childMatches.map(m => m[1]))
  const linkMatches = [...urdfXml.matchAll(/<link\s[^>]*name=["']([^"']*)["']/g)]
  const allLinks = linkMatches.map(m => m[1])
  return allLinks.find(l => !childLinks.has(l)) ?? null
}

/**
 * Dump the most recently compiled model as MJCF using mj_saveLastXML.
 *
 * Probes the WASM module for the `mj_saveLastXML` export and invokes it
 * with the loaded model. The dumped MJCF preserves URDF link names as
 * MJCF body names (verified empirically), so callers can use the URDF
 * root link name to find the target body in the dump.
 *
 * @param mj - The MuJoCo WASM module
 * @param model - The currently loaded MjModelInstance
 * @returns Compiled MJCF text
 * @throws Error if mj_saveLastXML is unavailable or the call fails
 */
export function dumpCompiledMJCF(mj: MjModule, model: MjModelInstance): string {
  const moduleObj = mj as unknown as Record<string, unknown>
  const fn = moduleObj['mj_saveLastXML']
  if (typeof fn !== 'function') {
    throw new Error('mj_saveLastXML not exposed on this WASM build')
  }
  const path = '/working/__sensor_injection_dump.xml'
  ;(fn as (path: string, model: unknown) => unknown)(path, model)
  const bytes = mj.FS.readFile(path)
  // Best-effort cleanup; ignored if the file is already gone.
  try { mj.FS.unlink(path) } catch { /* ignore */ }
  return new TextDecoder().decode(bytes)
}

/**
 * Inject sensor declarations into a compiled MJCF document.
 *
 * Adds an IMU site and three IMU sensors (gyro, accelerometer, framequat)
 * to the body that corresponds to the URDF's root link. Optionally adds
 * a lidar ring with N rangefinder sensors and N sites.
 *
 * Naming mirrors the bundled Go2 MJCF example so the existing TelemetryPanel
 * grouping, IMUArrows visualizer, and LidarVisualizer (which match by name
 * prefix) work on URDFs without any UI changes.
 *
 * Implementation: pure string manipulation. Web Workers don't expose
 * DOMParser, and the compiled MJCF has a known, simple structure
 * (single root `<mujoco>`, nested `<body>`s, optional flat `<sensor>`
 * block) so regex-based editing is sufficient.
 *
 * @param mjcf - Compiled MJCF text from `dumpCompiledMJCF`
 * @param rootBodyName - Body name to attach the IMU site to (URDF root link name)
 * @param config - Sensor configuration (defaults: IMU on, lidar off)
 * @returns MJCF text with sensor declarations added
 * @throws Error if the input isn't MJCF or the target body isn't found
 */
export function injectMJCFSensors(
  mjcf: string,
  rootBodyName: string,
  config: URDFSensorConfig,
): string {
  const imuOn = config.imu !== false
  const lidarOn = config.lidar === true
  if (!imuOn && !lidarOn) return mjcf // nothing to do

  // Smoke-check: compiled MJCF always has a `<mujoco ...>` root open tag.
  if (!/<mujoco\b/.test(mjcf)) {
    throw new Error('compiled MJCF root is not <mujoco>')
  }

  // Locate the target body's opening tag. mj_saveLastXML emits the name
  // attribute as the first attribute (`<body name="X" ...>`), but be liberal:
  // match `name="X"` anywhere within the open tag.
  const bodyOpenRe = new RegExp(
    `<body\\b[^>]*\\bname="${escapeRegex(rootBodyName)}"[^>]*>`,
  )
  const bodyMatch = bodyOpenRe.exec(mjcf)
  if (!bodyMatch) {
    throw new Error(`body name="${rootBodyName}" not found in compiled MJCF`)
  }
  const bodyOpenEnd = bodyMatch.index + bodyMatch[0].length

  // Build sites (inserted after the body's opening tag) and sensors
  // (collected, then appended into a <sensor> block).
  let sitesXml = ''
  let sensorsXml = ''

  if (imuOn) {
    // Lift the site slightly above the body origin so arrows draw clear of
    // most root-link meshes (matches Go2's pattern of placing the IMU site
    // above the base body's geometry).
    sitesXml += `\n      <site name="imu_site" pos="0 0 0.05" size="0.005" rgba="1 1 0 0.5"/>`
    sensorsXml += `\n    <gyro name="imu_gyro" site="imu_site"/>`
    sensorsXml += `\n    <accelerometer name="imu_accel" site="imu_site"/>`
    sensorsXml += `\n    <framequat name="imu_quat" objtype="site" objname="imu_site"/>`
  }

  if (lidarOn) {
    const rays = Math.max(1, Math.floor(config.lidarRays ?? 36))
    const range = config.lidarRange ?? 5
    for (let i = 0; i < rays; i++) {
      const angleDeg = Math.round((i * 360) / rays) % 360
      const padded = String(angleDeg).padStart(3, '0')
      const siteName = `lidar_${padded}`
      const yawRad = (i * 2 * Math.PI) / rays
      // Quaternion product: q_yaw(yawRad) * q_base(0.707107,0,0.707107,0).
      // q_yaw = (cos(yawRad/2), 0, 0, sin(yawRad/2)). Hamilton w,x,y,z:
      //   w =  c * 0.707107   x = -s * 0.707107
      //   y =  c * 0.707107   z =  s * 0.707107
      const c = Math.cos(yawRad / 2)
      const s = Math.sin(yawRad / 2)
      const k = 0.707107
      const qw = (c * k).toFixed(6)
      const qx = (-s * k).toFixed(6)
      const qy = (c * k).toFixed(6)
      const qz = (s * k).toFixed(6)
      sitesXml += `\n      <site name="${siteName}" pos="0 0 0.05" quat="${qw} ${qx} ${qy} ${qz}" size="0.005" rgba="0 0 0 0"/>`
      sensorsXml += `\n    <rangefinder name="${siteName}" site="${siteName}" cutoff="${range}"/>`
    }
  }

  // Step 1: insert sites immediately after the body's opening tag
  let result = mjcf.slice(0, bodyOpenEnd) + sitesXml + mjcf.slice(bodyOpenEnd)

  // Step 2: add or extend the <sensor> block. The compiled MJCF either has
  // a single `<sensor>...</sensor>` block as a direct child of <mujoco>
  // (rare for fresh URDF compiles) or none. Look for one and append; else
  // create a fresh block right before </mujoco>.
  const sensorBlockRe = /<sensor\b[^>]*>([\s\S]*?)<\/sensor>/
  const sensorMatch = sensorBlockRe.exec(result)
  if (sensorMatch) {
    const closeTagStart = sensorMatch.index + sensorMatch[0].length - '</sensor>'.length
    result = result.slice(0, closeTagStart) + sensorsXml + '\n  ' + result.slice(closeTagStart)
  } else {
    const newBlock = `  <sensor>${sensorsXml}\n  </sensor>\n`
    result = result.replace('</mujoco>', `${newBlock}</mujoco>`)
  }

  return result
}

/** Escape a string for use as a regex literal. */
function escapeRegex(s: string): string {
  return s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')
}
