/**
 * M4 G3 — key-pose trajectory executor. Runs IN the physics worker beside
 * the sim loop: each tick advances a rate-limited joint-space COMMAND toward
 * the current key pose (linear interpolation under per-joint speed limits —
 * the hardware profile's ramp discipline), holds through the pose's dwell,
 * then moves to the next. The caller writes the command to `data.ctrl`.
 *
 * The command (not the physical joint) is what a pose "reached" means —
 * dwells must cover servo settling, exactly like the real Feetech bus.
 * Pure TS: no mujoco-js imports (the worker supplies dt and consumes ctrl).
 */

import type { SimResult, TrajectoryConfig } from '../types/simulation'

export class TrajectoryRunner {
  readonly config: TrajectoryConfig
  private readonly command: number[]
  private poseIndex = 0
  private dwellRemaining: number
  private finished = false

  /** The command has arrived at the current pose (its dwell is running). */
  private atPose = false

  private constructor(config: TrajectoryConfig, initialCommand: readonly number[]) {
    this.config = config
    this.command = [...initialCommand]
    this.dwellRemaining = 0
  }

  /**
   * Validate and create a runner.
   *
   * @param config - Key poses + default speed limit
   * @param initialCommand - Starting command vector (current joint positions
   *   — the ramp starts where the arm is, no first-tick jerk)
   */
  static create(config: TrajectoryConfig, initialCommand: readonly number[]): SimResult<TrajectoryRunner> {
    if (config.poses.length === 0) {
      return { ok: false, error: 'trajectory has no key poses' }
    }
    if (!(config.maxSpeed > 0)) {
      return { ok: false, error: `trajectory maxSpeed must be positive, got ${config.maxSpeed}` }
    }
    for (let i = 0; i < config.poses.length; i++) {
      const pose = config.poses[i]
      if (pose.ctrl.length !== initialCommand.length) {
        return {
          ok: false,
          error: `trajectory pose ${i} has ${pose.ctrl.length} values, expected ${initialCommand.length} (one per actuator)`,
        }
      }
      if (pose.maxSpeed !== undefined && !(pose.maxSpeed > 0)) {
        return { ok: false, error: `trajectory pose ${i} maxSpeed must be positive` }
      }
      if (pose.dwell !== undefined && !(pose.dwell >= 0)) {
        return { ok: false, error: `trajectory pose ${i} dwell must be non-negative` }
      }
    }
    return { ok: true, value: new TrajectoryRunner(config, initialCommand) }
  }

  /** True once the last pose's dwell has elapsed. */
  get done(): boolean {
    return this.finished
  }

  /** Index of the key pose currently being approached/held. */
  get currentPoseIndex(): number {
    return this.poseIndex
  }

  /**
   * Advance the command by one tick.
   *
   * @param dtSeconds - Wall of simulated time this tick covers
   * @returns The rate-limited command vector (read-only view) and done flag
   */
  step(dtSeconds: number): { readonly ctrl: readonly number[]; readonly done: boolean } {
    if (this.finished) {
      return { ctrl: this.command, done: true }
    }
    let remaining = Math.max(dtSeconds, 0)
    while (remaining > 0 && !this.finished) {
      const pose = this.config.poses[this.poseIndex]
      if (!this.atPose) {
        const speed = pose.maxSpeed ?? this.config.maxSpeed
        // Largest per-joint gap decides how much of this tick the move consumes.
        let maxGap = 0
        for (let i = 0; i < this.command.length; i++) {
          maxGap = Math.max(maxGap, Math.abs(pose.ctrl[i] - this.command[i]))
        }
        const step = speed * remaining
        if (step < maxGap) {
          // Partial progress; the tick is spent.
          for (let i = 0; i < this.command.length; i++) {
            const gap = pose.ctrl[i] - this.command[i]
            this.command[i] += Math.sign(gap) * Math.min(Math.abs(gap), step)
          }
          remaining = 0
          break
        }
        // The pose is reached inside this tick; spend the exact travel time.
        for (let i = 0; i < this.command.length; i++) {
          this.command[i] = pose.ctrl[i]
        }
        remaining -= speed > 0 ? maxGap / speed : 0
        this.atPose = true
        this.dwellRemaining = pose.dwell ?? 0
      }
      // At the pose: burn dwell, then advance.
      if (this.dwellRemaining > remaining) {
        this.dwellRemaining -= remaining
        remaining = 0
        break
      }
      remaining -= this.dwellRemaining
      this.dwellRemaining = 0
      this.atPose = false
      if (this.poseIndex + 1 < this.config.poses.length) {
        this.poseIndex++
      } else {
        this.finished = true
      }
    }
    return { ctrl: this.command, done: this.finished }
  }
}
