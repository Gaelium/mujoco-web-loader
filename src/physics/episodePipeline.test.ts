// @vitest-environment node
/**
 * T1 S1c — unit gates for the shared load pipeline. The physics is
 * exercised end-to-end by the worker (browser e2e) and the headless episode
 * driver; here we pin the pure logic: STL bounds, URDF preprocessing
 * substitutions, and ctrl clamping — the pieces that must not drift between
 * the two consumers. (The cloth-bridge cases moved to the host app's
 * `clothBridge.test.ts` with the cut functions — SPLIT-NOTES D-SP-3.)
 */

import { describe, it, expect } from 'vitest'
import {
  computeSTLBounds,
  preprocessURDF,
  stepTrajectoryTick,
  type MeshBoundsInfo,
} from './episodePipeline'
import { TrajectoryRunner } from './TrajectoryRunner'
import type { MuJoCoEngine } from './MuJoCoEngine'
import type { ActuatorDescriptor } from '../types/simulation'
import { DEFAULT_ENVIRONMENT as EMPTY_PRESET } from '../environments/defaultEnvironment'

function binarySTL(vertices: ReadonlyArray<readonly [number, number, number]>): Uint8Array {
  const triCount = vertices.length / 3
  const buf = new ArrayBuffer(84 + triCount * 50)
  const view = new DataView(buf)
  view.setUint32(80, triCount, true)
  let off = 84
  for (let t = 0; t < triCount; t++) {
    off += 12 // normal
    for (let v = 0; v < 3; v++) {
      const [x, y, z] = vertices[t * 3 + v]
      view.setFloat32(off, x, true)
      view.setFloat32(off + 4, y, true)
      view.setFloat32(off + 8, z, true)
      off += 12
    }
    off += 2 // attribute byte count
  }
  return new Uint8Array(buf)
}

const MINIMAL_URDF = `<?xml version="1.0"?>
<robot name="probe">
  <link name="base">
    <visual>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://probe/meshes/base.STL"/></geometry>
    </visual>
    <collision>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://probe/meshes/base.STL"/></geometry>
    </collision>
  </link>
</robot>`

describe('computeSTLBounds', () => {
  it('computes extents and center of a binary STL', () => {
    const stl = binarySTL([
      [0, 0, 0],
      [2, 0, 0],
      [2, 4, 6],
    ])
    const info = computeSTLBounds(stl)
    expect(info).not.toBeNull()
    expect(info?.extents).toEqual([2, 4, 6])
    expect(info?.center).toEqual([1, 2, 3])
  })

  it('returns null for garbage', () => {
    expect(computeSTLBounds(new Uint8Array(10))).toBeNull()
  })
})

describe('preprocessURDF', () => {
  const bounds = new Map<string, MeshBoundsInfo>([
    ['base.STL', { extents: [0.2, 0.3, 0.4], center: [0.01, 0.02, 0.03] }],
  ])

  it('replaces mesh geometry with measured bounding boxes and offsets the origin', () => {
    const xml = preprocessURDF(MINIMAL_URDF, EMPTY_PRESET, bounds)
    expect(xml).not.toContain('<mesh')
    expect(xml).toContain('<box size="0.200000 0.300000 0.400000"/>')
    // origin 0.1 + center 0.01
    expect(xml).toContain('xyz="0.110000 0.020000 0.030000"')
  })

  it('injects the mujoco compiler block once', () => {
    const xml = preprocessURDF(MINIMAL_URDF, EMPTY_PRESET, bounds)
    expect(xml.match(/<mujoco>/g)).toHaveLength(1)
    expect(xml).toContain('balanceinertia')
  })

  it('falls back to default box size for unknown meshes', () => {
    const xml = preprocessURDF(MINIMAL_URDF, EMPTY_PRESET, new Map())
    expect(xml).toContain('<box size="0.050000 0.050000 0.050000"/>')
  })
})

describe('stepTrajectoryTick', () => {
  function mockEngine(): { engine: MuJoCoEngine; written: Array<[number, number]> } {
    const written: Array<[number, number]> = []
    const engine = {
      setControl: (i: number, v: number) => {
        written.push([i, v])
      },
    } as unknown as MuJoCoEngine
    return { engine, written }
  }

  it('writes clamped ctrl values and reports done', () => {
    const runner = TrajectoryRunner.create(
      { id: 't', poses: [{ ctrl: [10, -10] }], maxSpeed: 1000 },
      [0, 0],
    )
    expect(runner.ok).toBe(true)
    if (!runner.ok) return
    const { engine, written } = mockEngine()
    const actuators = [
      { ctrlRange: [-1, 1] },
      { ctrlRange: [0, 0] }, // degenerate range = unclamped
    ] as unknown as readonly ActuatorDescriptor[]
    const result = stepTrajectoryTick(engine, runner.value, actuators, 100)
    expect(written).toEqual([
      [0, 1], // 10 clamped to +1
      [1, -10], // degenerate range passes through
    ])
    expect(result.done).toBe(true)
    expect(result.ctrl).toEqual([1, -10]) // the applied (post-clamp) row
  })
})
