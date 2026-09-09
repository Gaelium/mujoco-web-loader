/**
 * Position-actuator + fixed-base injection for URDF uploads.
 *
 * MuJoCo's URDF parser silently drops `<actuator>`, `<default>`, `<keyframe>`,
 * and `<equality>` declarations placed inside the `<mujoco>` extension block —
 * they are tolerated but never compiled (same class of limitation that
 * `sensorInjection.ts` documents for `<sensor>`/`<site>`; see Decision Log
 * D-090). A raw URDF therefore compiles with **zero actuators**, which leaves
 * the sim driving joints via `qfrc_applied` (raw generalized force) with no
 * damping — an uncontrolled arm that spins/flies apart on play. URDFs are also
 * loaded free-floating, so the base drifts even when the joints are held.
 *
 * Workaround (the same two-pass load `sensorInjection.ts` uses): compile the
 * URDF, dump the compiled MJCF via `mj_saveLastXML`, edit the MJCF — where
 * `<actuator>`/`<equality>` ARE native — then recompile. This module performs
 * that edit:
 *   - tunes each movable joint (`armature`, `frictionloss`, optional `range`),
 *   - appends a `<position>` actuator per movable joint so commanding a joint
 *     angle drives a PD servo to it (1:1 with a real position-controlled servo's
 *     Goal_Position), and
 *   - optionally injects a fixed-base `<weld>` equality so the base can be
 *     pinned to the world (toggled at runtime via `data.eq_active`).
 *
 * The tuning defaults mirror the bundled `trs_so_arm100/so_arm100.xml`, the
 * gold-standard position-controlled arm of this family.
 *
 * Implementation is pure string manipulation: Web Workers don't expose
 * DOMParser, and the compiled MJCF has a known, simple structure. Mirrors the
 * approach (and helpers) in `sensorInjection.ts`.
 *
 * This module runs only inside the physics Web Worker.
 */

/** A movable joint eligible for actuator injection. */
export interface InjectableJoint {
  /** Joint name as it appears in the compiled MJCF. */
  readonly name: string
  /** Joint limit range in radians/metres, or null if the joint is unlimited. */
  readonly range: readonly [number, number] | null
}

/** Tuning for the injected position actuators and their joints. */
export interface ActuatorInjectionOptions {
  /** 'position' injects PD position actuators; 'none' is a no-op. Default: 'position'. */
  readonly mode?: 'position' | 'none'
  /** Position gain (kp) applied to every actuator unless overridden. Default: 50. */
  readonly defaultKp?: number
  /** Damping ratio of the position servo (kv derived from kp). Default: 1 (critically damped). */
  readonly defaultDampratio?: number
  /** Actuator force/torque clamp [lo, hi]. Default: [-3.5, 3.5]. */
  readonly defaultForcerange?: readonly [number, number]
  /** Joint armature (reflected rotor inertia). Default: 0.1. */
  readonly defaultArmature?: number
  /** Joint dry friction loss. Default: 0.1. */
  readonly defaultFrictionloss?: number
  /** Per-joint overrides keyed by joint name. */
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

/** Default tuning, copied from `trs_so_arm100/so_arm100.xml`. */
const DEFAULTS = {
  kp: 50,
  dampratio: 1,
  forcerange: [-3.5, 3.5] as readonly [number, number],
  armature: 0.1,
  frictionloss: 0.1,
}

/** Default name of the fixed-base weld equality (matches the worker's constant). */
export const DEFAULT_FIXED_BASE_WELD_NAME = '_simulo_fixed_base'

/**
 * Inject position actuators (and per-joint tuning) into a compiled MJCF.
 *
 * For each joint in `joints` (which must already exist in the MJCF), adds
 * `armature`/`frictionloss` (and `range` if overridden) to the joint tag and
 * appends a `<position joint="…">` actuator. Actuators are emitted in the order
 * `joints` is given — callers MUST pass them in ascending MuJoCo joint-id order
 * so `data.ctrl[i]` maps deterministically to `joints[i]`.
 *
 * @param mjcf - Compiled MJCF text from `dumpCompiledMJCF`
 * @param joints - Movable joints to actuate, in ascending joint-id order
 * @param opts - Tuning (defaults mirror so_arm100)
 * @returns MJCF text with actuators + joint tuning added (unchanged if mode='none')
 * @throws Error if the input isn't MJCF
 */
export function injectMJCFActuators(
  mjcf: string,
  joints: readonly InjectableJoint[],
  opts: ActuatorInjectionOptions = {},
): string {
  if ((opts.mode ?? 'position') === 'none') return mjcf
  if (joints.length === 0) return mjcf
  if (!/<mujoco\b/.test(mjcf)) {
    throw new Error('compiled MJCF root is not <mujoco>')
  }

  const kp = opts.defaultKp ?? DEFAULTS.kp
  const dampratio = opts.defaultDampratio ?? DEFAULTS.dampratio
  const forcerange = opts.defaultForcerange ?? DEFAULTS.forcerange
  const armature = opts.defaultArmature ?? DEFAULTS.armature
  const frictionloss = opts.defaultFrictionloss ?? DEFAULTS.frictionloss

  let result = mjcf
  let actuatorsXml = ''

  for (const joint of joints) {
    const override = opts.perJoint?.[joint.name]
    const range = override?.range ?? joint.range

    // 1. Tune the joint tag (armature + frictionloss always; range only when
    //    overridden — otherwise keep the range MuJoCo parsed from the URDF).
    result = setJointAttrs(result, joint.name, {
      armature,
      frictionloss,
      range: override?.range ?? null,
    })

    // 2. Build the position actuator. `inheritrange="1"` copies the joint's
    //    range into the actuator's ctrlrange (so control commands are clamped
    //    to the joint limits) — only valid when the joint is limited.
    const aKp = override?.kp ?? kp
    const aFr = override?.forcerange ?? forcerange
    const inherit = range !== null ? ' inheritrange="1"' : ''
    actuatorsXml +=
      `\n    <position name="${joint.name}" joint="${joint.name}"` +
      ` kp="${num(aKp)}" dampratio="${num(dampratio)}"` +
      ` forcerange="${num(aFr[0])} ${num(aFr[1])}"${inherit}/>`
  }

  return appendBlock(result, 'actuator', actuatorsXml)
}

/**
 * Inject a fixed-base weld equality into a compiled MJCF.
 *
 * Adds `<weld name=… body1="<rootBodyName>" active="false"/>` so the worker can
 * pin/unpin the base at runtime via `data.eq_active` (see `applyFixedBase` in
 * the worker). URDF compiles already carry a free joint on the root body
 * (injected by `preprocessURDF`), so only the weld is missing — this supplies
 * it. The byte format matches the MJCF path's `injectMJCFFloatingBase`.
 *
 * @param mjcf - Compiled MJCF text
 * @param rootBodyName - Name of the body carrying the free joint (URDF root link)
 * @param weldName - Equality name (defaults to the worker's weld constant)
 * @returns MJCF with the weld added (unchanged if a weld of that name exists)
 * @throws Error if the input isn't MJCF
 */
export function injectMJCFFixedBaseWeld(
  mjcf: string,
  rootBodyName: string,
  weldName: string = DEFAULT_FIXED_BASE_WELD_NAME,
): string {
  if (!/<mujoco\b/.test(mjcf)) {
    throw new Error('compiled MJCF root is not <mujoco>')
  }
  if (new RegExp(`\\bname="${escapeRegex(weldName)}"`).test(mjcf)) {
    return mjcf // already present
  }
  const weldXml = `\n    <weld name="${weldName}" body1="${rootBodyName}" active="false"/>`
  return appendBlock(mjcf, 'equality', weldXml)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/**
 * Set `armature`/`frictionloss` (and optionally `range`) on a single joint's
 * self-closing tag, replacing existing attributes or inserting new ones.
 */
function setJointAttrs(
  mjcf: string,
  jointName: string,
  attrs: {
    armature: number
    frictionloss: number
    range: readonly [number, number] | null
  },
): string {
  const re = new RegExp(`<joint\\b[^>]*\\bname="${escapeRegex(jointName)}"[^>]*?>`)
  const match = re.exec(mjcf)
  if (!match) return mjcf // joint not in MJCF — nothing to tune

  let tag = match[0]
  tag = setAttr(tag, 'armature', num(attrs.armature))
  tag = setAttr(tag, 'frictionloss', num(attrs.frictionloss))
  if (attrs.range !== null) {
    tag = setAttr(tag, 'range', `${num(attrs.range[0])} ${num(attrs.range[1])}`)
    tag = setAttr(tag, 'limited', 'true')
  }
  return mjcf.slice(0, match.index) + tag + mjcf.slice(match.index + match[0].length)
}

/** Replace `attr="…"` in a tag if present, else insert before its closing `>`/`/>`. */
function setAttr(tag: string, attr: string, value: string): string {
  const re = new RegExp(`(\\s)${attr}="[^"]*"`)
  if (re.test(tag)) return tag.replace(re, `$1${attr}="${value}"`)
  if (tag.endsWith('/>')) return `${tag.slice(0, -2)} ${attr}="${value}"/>`
  return `${tag.slice(0, -1)} ${attr}="${value}">`
}

/**
 * Append `innerXml` to an existing `<blockName>…</blockName>` block, or create a
 * fresh one right before `</mujoco>`. Mirrors `sensorInjection`'s `<sensor>`
 * handling so a compiled MJCF that already has the block isn't duplicated.
 */
function appendBlock(mjcf: string, blockName: string, innerXml: string): string {
  const re = new RegExp(`<${blockName}\\b[^>]*>([\\s\\S]*?)</${blockName}>`)
  const match = re.exec(mjcf)
  if (match) {
    const closeStart = match.index + match[0].length - `</${blockName}>`.length
    return mjcf.slice(0, closeStart) + innerXml + '\n  ' + mjcf.slice(closeStart)
  }
  const block = `  <${blockName}>${innerXml}\n  </${blockName}>\n`
  return mjcf.replace('</mujoco>', `${block}</mujoco>`)
}

/** Format a number without scientific notation or trailing zero noise. */
function num(v: number): string {
  if (Number.isInteger(v)) return String(v)
  return Number(v.toFixed(6)).toString()
}

/** Escape a string for use as a regex literal. */
function escapeRegex(s: string): string {
  return s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')
}
