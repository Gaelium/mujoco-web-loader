import { describe, it, expect } from 'vitest'
import { TrajectoryRunner } from './TrajectoryRunner'
import type { TrajectoryConfig } from '../types/simulation'

function make(config: Partial<TrajectoryConfig> & Pick<TrajectoryConfig, 'poses'>, initial: number[]): TrajectoryRunner {
  const created = TrajectoryRunner.create({ id: 't', maxSpeed: 1, ...config }, initial)
  if (!created.ok) throw new Error(created.error)
  return created.value
}

describe('TrajectoryRunner.create validation', () => {
  it('rejects empty pose lists, bad speeds, and length mismatches', () => {
    expect(TrajectoryRunner.create({ id: 't', maxSpeed: 1, poses: [] }, [0]).ok).toBe(false)
    expect(TrajectoryRunner.create({ id: 't', maxSpeed: 0, poses: [{ ctrl: [1] }] }, [0]).ok).toBe(false)
    expect(TrajectoryRunner.create({ id: 't', maxSpeed: 1, poses: [{ ctrl: [1, 2] }] }, [0]).ok).toBe(false)
    expect(TrajectoryRunner.create({ id: 't', maxSpeed: 1, poses: [{ ctrl: [1], dwell: -1 }] }, [0]).ok).toBe(false)
  })
})

describe('TrajectoryRunner stepping', () => {
  it('rate-limits each joint to maxSpeed', () => {
    const r = make({ poses: [{ ctrl: [1, -1] }], maxSpeed: 0.5 }, [0, 0])
    const step = r.step(1)
    expect(step.ctrl[0]).toBeCloseTo(0.5, 9)
    expect(step.ctrl[1]).toBeCloseTo(-0.5, 9)
    expect(step.done).toBe(false)
    expect(r.step(1).done).toBe(true)
  })

  it('a smaller-gap joint arrives early (per-joint clamp, the hardware ramp discipline)', () => {
    const r = make({ poses: [{ ctrl: [1, 0.2] }], maxSpeed: 0.5 }, [0, 0])
    const step = r.step(1)
    expect(step.ctrl[0]).toBeCloseTo(0.5, 9)
    expect(step.ctrl[1]).toBeCloseTo(0.2, 9)
  })

  it('applies a per-pose speed override', () => {
    const r = make({ poses: [{ ctrl: [1], maxSpeed: 2 }], maxSpeed: 0.1 }, [0])
    expect(r.step(0.25).ctrl[0]).toBeCloseTo(0.5, 9)
  })

  it('dwell holds after arrival, including a pose equal to the current command', () => {
    const r = make({ poses: [{ ctrl: [0], dwell: 0.5 }, { ctrl: [1] }] }, [0])
    // Arrives instantly, dwell 0.5 s consumes this tick.
    expect(r.step(0.3).done).toBe(false)
    expect(r.currentPoseIndex).toBe(0)
    // Dwell finishes, then moves 0.8 s' worth toward pose 1.
    const step = r.step(1.0)
    expect(step.ctrl[0]).toBeCloseTo(0.8, 6)
    expect(step.done).toBe(false)
    expect(r.step(1).done).toBe(true)
  })

  it('carries leftover tick time across a pose boundary', () => {
    // Reach [0.5] in 0.5 s, then continue toward [1] with the remaining 0.5 s.
    const r = make({ poses: [{ ctrl: [0.5] }, { ctrl: [1] }] }, [0])
    const step = r.step(1)
    expect(step.ctrl[0]).toBeCloseTo(1, 9)
    expect(step.done).toBe(true)
  })

  it('runs a full multi-pose sequence with dwells and reports done exactly once', () => {
    const r = make(
      {
        poses: [
          { ctrl: [1, 0], dwell: 0.2 },
          { ctrl: [1, 1], dwell: 0.2 },
          { ctrl: [0, 0] },
        ],
      },
      [0, 0],
    )
    let ticks = 0
    while (!r.step(0.1).done) {
      ticks++
      if (ticks > 100) throw new Error('trajectory did not finish')
    }
    // 1 s + 0.2 + 1 s + 0.2 + 1 s ≈ 3.4 s at 0.1 s ticks.
    expect(ticks).toBeGreaterThanOrEqual(32)
    expect(ticks).toBeLessThanOrEqual(36)
    expect(r.step(0.1).done).toBe(true) // stays done, command holds
    expect([...r.step(0.1).ctrl]).toEqual([0, 0])
  })
})
