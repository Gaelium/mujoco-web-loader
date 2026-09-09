import { describe, it, expect } from 'vitest'
import {
  injectMJCFActuators,
  injectMJCFFixedBaseWeld,
  DEFAULT_FIXED_BASE_WELD_NAME,
  type InjectableJoint,
} from './actuatorInjection'

// A minimal compiled-MJCF mirroring what mj_saveLastXML produces for a URDF
// arm after preprocessURDF: a free joint on the root body, two hinge joints,
// one of them already carrying a parsed range.
const SAMPLE_MJCF = `<mujoco model="arm">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base">
      <freejoint name="_simulo_floating_base"/>
      <geom size="0.05 0.05 0.025" type="box"/>
      <body name="upper" pos="0 0 0.05">
        <joint name="shoulder" axis="0 1 0" range="-1.92 1.92"/>
        <geom size="0.025 0.025 0.15" type="box"/>
        <body name="lower" pos="0 0 0.3">
          <joint name="elbow" axis="1 0 0"/>
          <geom size="0.025 0.025 0.15" type="box"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>`

const JOINTS: readonly InjectableJoint[] = [
  { name: 'shoulder', range: [-1.92, 1.92] },
  { name: 'elbow', range: null },
]

describe('injectMJCFActuators', () => {
  it('returns the input unchanged when mode is none', () => {
    expect(injectMJCFActuators(SAMPLE_MJCF, JOINTS, { mode: 'none' })).toBe(SAMPLE_MJCF)
  })

  it('returns the input unchanged when there are no joints', () => {
    expect(injectMJCFActuators(SAMPLE_MJCF, [])).toBe(SAMPLE_MJCF)
  })

  it('throws when the input is not MJCF', () => {
    expect(() => injectMJCFActuators('<robot/>', JOINTS)).toThrow(/not <mujoco>/)
  })

  it('appends one position actuator per joint, in the given order', () => {
    const out = injectMJCFActuators(SAMPLE_MJCF, JOINTS)
    const names = [...out.matchAll(/<position\b[^>]*joint="([^"]+)"/g)].map((m) => m[1])
    expect(names).toEqual(['shoulder', 'elbow'])
  })

  it('uses inheritrange only for joints that have a range', () => {
    const out = injectMJCFActuators(SAMPLE_MJCF, JOINTS)
    const shoulder = /<position\b[^>]*name="shoulder"[^>]*\/>/.exec(out)?.[0] ?? ''
    const elbow = /<position\b[^>]*name="elbow"[^>]*\/>/.exec(out)?.[0] ?? ''
    expect(shoulder).toMatch(/inheritrange="1"/)
    expect(elbow).not.toMatch(/inheritrange/)
  })

  it('applies default kp / dampratio / forcerange from so_arm100', () => {
    const out = injectMJCFActuators(SAMPLE_MJCF, JOINTS)
    expect(out).toMatch(/<position\b[^>]*kp="50"/)
    expect(out).toMatch(/<position\b[^>]*dampratio="1"/)
    expect(out).toMatch(/<position\b[^>]*forcerange="-3.5 3.5"/)
  })

  it('honors per-joint kp / forcerange / range overrides', () => {
    const out = injectMJCFActuators(SAMPLE_MJCF, JOINTS, {
      perJoint: { shoulder: { kp: 120, forcerange: [-10, 10], range: [-1, 1] } },
    })
    const shoulder = /<position\b[^>]*name="shoulder"[^>]*\/>/.exec(out)?.[0] ?? ''
    expect(shoulder).toMatch(/kp="120"/)
    expect(shoulder).toMatch(/forcerange="-10 10"/)
    // The overridden range is written onto the joint tag too.
    const jointTag = /<joint\b[^>]*name="shoulder"[^>]*\/>/.exec(out)?.[0] ?? ''
    expect(jointTag).toMatch(/range="-1 1"/)
    expect(jointTag).toMatch(/limited="true"/)
  })

  it('adds armature + frictionloss to every actuated joint tag', () => {
    const out = injectMJCFActuators(SAMPLE_MJCF, JOINTS)
    const shoulder = /<joint\b[^>]*name="shoulder"[^>]*\/>/.exec(out)?.[0] ?? ''
    const elbow = /<joint\b[^>]*name="elbow"[^>]*\/>/.exec(out)?.[0] ?? ''
    expect(shoulder).toMatch(/armature="0.1"/)
    expect(shoulder).toMatch(/frictionloss="0.1"/)
    expect(elbow).toMatch(/armature="0.1"/)
    expect(elbow).toMatch(/frictionloss="0.1"/)
  })

  it('does not clobber a joint range that was already parsed from the URDF', () => {
    const out = injectMJCFActuators(SAMPLE_MJCF, JOINTS)
    const shoulder = /<joint\b[^>]*name="shoulder"[^>]*\/>/.exec(out)?.[0] ?? ''
    // shoulder keeps its original -1.92 1.92 (no override supplied)
    expect(shoulder).toMatch(/range="-1.92 1.92"/)
    expect((shoulder.match(/range=/g) ?? []).length).toBe(1)
  })

  it('appends to an existing <actuator> block instead of duplicating it', () => {
    const withActuator = SAMPLE_MJCF.replace(
      '</worldbody>',
      '</worldbody>\n  <actuator>\n    <motor name="pre" joint="elbow"/>\n  </actuator>',
    )
    const out = injectMJCFActuators(withActuator, JOINTS)
    expect((out.match(/<actuator\b/g) ?? []).length).toBe(1)
    expect(out).toMatch(/name="pre"/)
    expect(out).toMatch(/<position\b[^>]*name="shoulder"/)
  })
})

describe('injectMJCFFixedBaseWeld', () => {
  it('creates an <equality> block with a weld targeting the root body', () => {
    const out = injectMJCFFixedBaseWeld(SAMPLE_MJCF, 'base')
    expect(out).toMatch(
      new RegExp(`<weld\\b[^>]*name="${DEFAULT_FIXED_BASE_WELD_NAME}"[^>]*body1="base"[^>]*active="false"`),
    )
  })

  it('is idempotent — does not add a second weld of the same name', () => {
    const once = injectMJCFFixedBaseWeld(SAMPLE_MJCF, 'base')
    const twice = injectMJCFFixedBaseWeld(once, 'base')
    expect(twice).toBe(once)
    expect((twice.match(/<weld\b/g) ?? []).length).toBe(1)
  })

  it('adds the weld inside an existing <equality> block', () => {
    const withEq = SAMPLE_MJCF.replace(
      '</worldbody>',
      '</worldbody>\n  <equality>\n    <connect name="pre" body1="upper" anchor="0 0 0"/>\n  </equality>',
    )
    const out = injectMJCFFixedBaseWeld(withEq, 'base')
    expect((out.match(/<equality\b/g) ?? []).length).toBe(1)
    expect(out).toMatch(/name="pre"/)
    expect(out).toMatch(/<weld\b/)
  })

  it('throws when the input is not MJCF', () => {
    expect(() => injectMJCFFixedBaseWeld('<robot/>', 'base')).toThrow(/not <mujoco>/)
  })
})
