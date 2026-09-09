# API reference

Everything exports from the package root (`@robot-sim/mujoco-web-loader`).
This is a map, not a spec — every function carries full JSDoc in source, and
the co-located tests are executable documentation.

## `MuJoCoEngine` (physics/MuJoCoEngine.ts)

The typed wrapper over the raw WASM module. Construct with the module from
`initMujocoModule` (or a direct factory call under Node).

| Area | Methods |
|---|---|
| VFS | `writeToVFS`, `ensureVFSDir`, `listVFSDir` |
| Model lifecycle | `loadModel(xml, path) → RobotDescriptor`, `loadBinary`, `saveBinary`, `dispose`, `isLoaded`, `getRawModel`, `getVersion` |
| Stepping | `step`, `stepN(count)`, `reset`, `forward`, `hasKeyframes`, `resetToKeyframe` |
| State out | `getSimState(paused) → SimState`, `getBodyStates`, `getJointStates`, `getSensorReadings`, `getTime`, `getNumDOF` |
| Control in | `setControl(i, v)`, `setControls(map)`, `setAppliedForce(s)`, `clearAppliedForces` |
| Joints/qpos | `getJointQposAddr`, `getFirstFreeJointQposAddr`, `setQpos`, `getQposValue`, `zeroFreeJointVel`, `getJointRange` |
| Equalities | `findEqualityByName`, `setEqualityActive`, `setWeldRelpose`, `getEqualityCount` |
| Physics params | `getTimestep`, `setTimestep`, `setGravity` |
| Cameras | `findCameraByName`, `setCameraLocalPose` |

`MjModule` / `MjModelInstance` — the structural types for the raw WASM
surface; the `__mocks__/mujoco-js` factory satisfies them for tests.

## URDF loading & repair

| Export | Role |
|---|---|
| `parseURDF(xml)` | `SimResult<URDFParseResult>` — name, links, joints, mesh paths (DOMParser with a worker-safe fallback) |
| `validateURDF(xml)` | Structural `URDFValidationError[]` for upload UIs |
| `stripUnsupportedURDFElements(xml)` | Remove elements MuJoCo rejects — always the first step |
| `preprocessURDF(xml, env, meshBounds)` | The repair pass — see [pipeline.md](./pipeline.md) |
| `computeSTLBounds(bytes)` | `MeshBoundsInfo` (extents + center) from binary/ASCII STL |
| `normalizeMeshPath`, `extractMeshPaths`, `getMeshFilenames` | `package://`-style path handling |

## Two-pass injection

| Export | Role |
|---|---|
| `applyURDFPass2(engine, mj, urdf, actuatorConfig?, sensorConfig?, vfsPath)` | dump → inject actuators + weld + sensors → recompile |
| `injectMJCFActuators(mjcf, joints, config)` | PD position actuators (defaults: kp 50, dampratio 1, forcerange ±3.5) |
| `injectMJCFFixedBaseWeld(mjcf, rootBody, name)` / `DEFAULT_FIXED_BASE_WELD_NAME` | The runtime-toggleable base weld |
| `injectMJCFSensors(mjcf, rootLink, config)` | IMU (default on) + lidar ring (default off) |
| `dumpCompiledMJCF(mj, model)` | `mj_saveLastXML` on the live compiled model |
| `findRootLinkInUrdf(xml)` | Root-link detection shared by the injectors |
| `buildInjectableJoints(engine)` | Movable joints in ctrl order (the mapping contract) |
| `applyFixedBase(engine, fixed, pos, quat)` | Toggle the weld + snap the freejoint |
| `applyHomePose(engine, homeQpos)` | qpos + position targets to a rest pose |

## MJCF & scene composition

`stripMJCFForPhysics`, `injectMJCFFloatingBase`, `injectMJCFEnvironment` —
the MJCF-input path. `composeScene(env, robots?)` /
`composeStandaloneRobot` — environment-only or include-based scenes; XML
fragment helpers (`generateGroundXml`, `generatePrimitiveXml`,
`primitiveDiagInertia`, `escapeXmlAttr`, …) export individually.

## Runtime plumbing

`initMujocoModule(options)`, `installWriteI53ToI64Polyfill(mj)`,
`patchMujocoWasmMemoryMax(bytes)` — see
[worker-integration.md](./worker-integration.md).

## Caching

`hashXml(version, xml)`, `getCachedBinary(key)`, `putCachedBinary(key,
bytes)` — two-tier compiled-model cache (memory + IndexedDB, LRU 16
entries / 100 MB, graceful without IndexedDB).

## Trajectories

`TrajectoryRunner.create(config, initialCtrl)` — rate-limited key-pose
interpolation with per-joint speed limits and dwells;
`stepTrajectoryTick(engine, runner, actuators, dt)` applies one tick with
ctrl-range clamping.

## Types (`types/simulation`, `types/environment`)

The vendored spine: `Vec3`, `MjQuat` (MuJoCo `[w,x,y,z]`!), `BodyState`,
`JointState`, `ActuatorDescriptor`, `SensorReading`, `SiteState`,
`CameraDescriptor`/`CameraState`, `ContactPoint`, `RobotDescriptor`,
`SimState`, `ModelSignature`, `URDFActuatorConfig`, `URDFSensorConfig`,
`TrajectoryConfig`, `SimResult<T>`, `EnvironmentConfig` (+
`DEFAULT_ENVIRONMENT`). Quaternion convention: MuJoCo is `[w,x,y,z]`,
Three.js is `[x,y,z,w]` — convert at your render boundary (the example app
shows the one-liner).
