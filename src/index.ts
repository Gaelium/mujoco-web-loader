/**
 * @robot-sim/mujoco-web-loader — public API.
 *
 * Drop any URDF into MuJoCo-in-the-browser (or Node) and it just works:
 * preprocessing/repair, two-pass actuator + sensor injection, compiled-model
 * caching, scene composition, trajectory driving, and the reusable worker
 * plumbing. Worker construction stays with the consumer — this package never
 * calls `new Worker(...)` and never imports bundler-specific asset URLs.
 */

export * from './types/simulation'
export * from './types/environment'
export * from './environments/defaultEnvironment'
export * from './physics/MuJoCoEngine'
export * from './physics/URDFLoader'
export * from './physics/SceneComposer'
export * from './physics/actuatorInjection'
export * from './physics/sensorInjection'
export * from './physics/binaryCache'
export * from './physics/episodePipeline'
export * from './physics/TrajectoryRunner'
export * from './physics/mujocoWorker'
