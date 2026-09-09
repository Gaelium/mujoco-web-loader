import { describe, it, expect } from 'vitest'
import { findRootLinkInUrdf, injectMJCFSensors } from './sensorInjection'

describe('findRootLinkInUrdf', () => {
  it('returns the link not referenced by any joint child', () => {
    const urdf = `
      <robot name="t">
        <link name="base"/>
        <link name="arm"/>
        <joint name="j" type="revolute">
          <parent link="base"/>
          <child link="arm"/>
        </joint>
      </robot>`
    expect(findRootLinkInUrdf(urdf)).toBe('base')
  })

  it('handles single-quoted attributes', () => {
    const urdf = `<robot><link name='base'/><link name='arm'/><joint type='fixed'><parent link='base'/><child link='arm'/></joint></robot>`
    expect(findRootLinkInUrdf(urdf)).toBe('base')
  })

  it('returns null when there are no links', () => {
    expect(findRootLinkInUrdf('<robot></robot>')).toBeNull()
  })

  it('returns null when every link is a child (cycle / malformed)', () => {
    const urdf = `
      <robot>
        <link name="a"/>
        <link name="b"/>
        <joint type="fixed"><parent link="a"/><child link="a"/></joint>
        <joint type="fixed"><parent link="b"/><child link="b"/></joint>
      </robot>`
    expect(findRootLinkInUrdf(urdf)).toBeNull()
  })

  it('finds the URDF root link from a multi-link compiled-after-preprocess input', () => {
    // Mirrors what preprocessURDF produces — the floating-base joint adds
    // _simulo_world as the new root, but the user's link "base" is the
    // first non-helper link.
    const urdf = `
      <robot>
        <link name="_simulo_world"/>
        <joint name="_simulo_floating_base" type="floating">
          <parent link="_simulo_world"/>
          <child link="base"/>
        </joint>
        <link name="base"/>
        <link name="arm"/>
        <joint name="shoulder" type="revolute">
          <parent link="base"/>
          <child link="arm"/>
        </joint>
      </robot>`
    expect(findRootLinkInUrdf(urdf)).toBe('_simulo_world')
  })
})

describe('injectMJCFSensors', () => {
  // A minimal MJCF mirroring what mj_saveLastXML produces for a multi-body
  // URDF — root <mujoco>, <worldbody>, then nested <body name="...">.
  const SAMPLE_MJCF = `<mujoco model="t">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base">
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <joint name="_simulo_floating_base" type="free"/>
      <geom size="0.05 0.05 0.025" type="box"/>
      <body name="arm" pos="0 0 0.05">
        <joint name="shoulder" axis="0 1 0"/>
        <geom size="0.025 0.025 0.15" type="box"/>
      </body>
    </body>
  </worldbody>
</mujoco>`

  it('returns the input unchanged when both IMU and lidar are off', () => {
    const out = injectMJCFSensors(SAMPLE_MJCF, 'base', { imu: false, lidar: false })
    expect(out).toBe(SAMPLE_MJCF)
  })

  it('injects an IMU site + 3 sensors by default', () => {
    const out = injectMJCFSensors(SAMPLE_MJCF, 'base', {})
    expect(out).toContain('<site')
    expect(out).toMatch(/name="imu_site"/)
    expect(out).toMatch(/<gyro\b[^>]*name="imu_gyro"[^>]*site="imu_site"/)
    expect(out).toMatch(/<accelerometer\b[^>]*name="imu_accel"[^>]*site="imu_site"/)
    expect(out).toMatch(/<framequat\b[^>]*name="imu_quat"[^>]*objname="imu_site"/)
  })

  it('attaches the IMU site as a child of the named body', () => {
    const out = injectMJCFSensors(SAMPLE_MJCF, 'base', { imu: true })
    // The site must appear inside the <body name="base">...</body> block,
    // not before or after it.
    const baseStart = out.indexOf('name="base"')
    const baseEnd = out.indexOf('<body name="arm"')
    const sitePos = out.indexOf('name="imu_site"')
    expect(sitePos).toBeGreaterThan(baseStart)
    expect(sitePos).toBeLessThan(baseEnd)
  })

  it('injects N rangefinder sensors and N sites when lidar is enabled', () => {
    const out = injectMJCFSensors(SAMPLE_MJCF, 'base', { imu: false, lidar: true, lidarRays: 8, lidarRange: 3 })
    const rangefinderCount = (out.match(/<rangefinder\b/g) ?? []).length
    expect(rangefinderCount).toBe(8)
    // Each rangefinder references a distinct site by name.
    const siteRefs = [...out.matchAll(/<rangefinder[^>]*site="(lidar_\d{3})"/g)].map(m => m[1])
    expect(new Set(siteRefs).size).toBe(8)
    // Cutoff range is propagated.
    expect(out).toMatch(/cutoff="3"/)
  })

  it('uses the default 36 rays / 5 m cutoff when lidar config omits them', () => {
    const out = injectMJCFSensors(SAMPLE_MJCF, 'base', { imu: false, lidar: true })
    expect((out.match(/<rangefinder\b/g) ?? []).length).toBe(36)
    expect(out).toMatch(/cutoff="5"/)
  })

  it('injects both IMU and lidar when both are enabled', () => {
    const out = injectMJCFSensors(SAMPLE_MJCF, 'base', { imu: true, lidar: true, lidarRays: 4 })
    expect((out.match(/<gyro\b/g) ?? []).length).toBe(1)
    expect((out.match(/<rangefinder\b/g) ?? []).length).toBe(4)
    expect(out).toMatch(/name="imu_site"/)
    expect(out).toMatch(/name="lidar_000"/)
  })

  it('throws when the named body is not present in the MJCF', () => {
    expect(() => injectMJCFSensors(SAMPLE_MJCF, 'nonexistent', { imu: true }))
      .toThrow(/body name="nonexistent" not found/)
  })

  it('throws when the input is not MJCF (no <mujoco> root)', () => {
    expect(() => injectMJCFSensors('<robot/>', 'base', { imu: true }))
      .toThrow(/not <mujoco>/)
  })

  it('appends to an existing <sensor> block instead of duplicating it', () => {
    const mjcfWithExistingSensor = `<mujoco>
  <worldbody><body name="base"/></worldbody>
  <sensor>
    <jointpos name="pre_existing" joint="shoulder"/>
  </sensor>
</mujoco>`
    const out = injectMJCFSensors(mjcfWithExistingSensor, 'base', { imu: true })
    const sensorBlockCount = (out.match(/<sensor\b/g) ?? []).length
    expect(sensorBlockCount).toBe(1)
    expect(out).toMatch(/pre_existing/)
    expect(out).toMatch(/imu_gyro/)
  })
})
