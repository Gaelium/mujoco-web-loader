# The load pipeline, end to end

What actually happens between "user hands you a URDF" and "a compiled,
actuated, sensor-equipped MuJoCo model is stepping". Every stage below is a
public export; the [example app](../example/main.ts) runs exactly this
sequence in ~40 lines.

```
URDF text + mesh bytes
  │  computeSTLBounds(bytes)            per-mesh AABB extents + center
  │  engine.writeToVFS(...)             meshes into Emscripten VFS
  ▼
stripUnsupportedURDFElements(xml)       remove elements MuJoCo rejects
  ▼
preprocessURDF(xml, env, meshBounds)    the repair pass (Pass 1 input)
  ▼
engine.loadModel(xml, vfsPath)          MuJoCo compile #1
  ▼
applyURDFPass2(engine, mj, originalUrdf, actuatorConfig, sensorConfig, path)
  │  dumpCompiledMJCF (mj_saveLastXML)  the compiled model, as native MJCF
  │  injectMJCFActuators                PD position actuator per movable joint
  │  injectMJCFFixedBaseWeld            runtime-toggleable weld (inactive)
  │  injectMJCFSensors                  IMU (+ optional lidar) on the root link
  ▼
engine.loadModel(edited, pass2Path)     MuJoCo compile #2 → RobotDescriptor
  ▼
applyFixedBase / applyHomePose          optional: pin base, drive rest pose
```

## Stage notes

### `computeSTLBounds`

Parses binary STL directly (80-byte header + triangle records) with an ASCII
fallback, producing full extents `[dx,dy,dz]` and the geometric center in
mesh coordinates. Degenerate axes clamp to 1 mm. These bounds feed the mesh
substitution below — measure them from the *raw bytes you were given*, not
from a render pipeline.

### `preprocessURDF(urdfXml, env, meshBounds)`

The repair pass, in order:

1. **Compiler hints** — injects
   `<mujoco><compiler balanceinertia="true"/></mujoco>` into `<robot>`.
   Degenerate inertias are endemic in hand-authored URDFs; without this
   they're fatal compile errors.
2. **Mesh → measured bounding box** — every `<mesh>` inside
   `<visual>`/`<collision>` becomes a `<box>` sized from `meshBounds`
   (fallback 5 cm cube), with the block's `<origin>` offset by the mesh's
   geometric center and any `<mesh scale>` applied. This is the workaround
   for the WASM build's VFS mesh-load hang. It is *visually* lossless only
   if you render the real meshes yourself — the physics deliberately runs
   on the boxes, and downstream consumers must compare against exactly this
   model (see "Model identity" below).
3. **Collision synthesis** — links with `<visual>` but no `<collision>` get
   the visual geometry cloned as collision, so RViz-style URDFs actually
   generate contacts.
4. **Floating base** — injects a world link + floating joint above the root
   link (skipped if the URDF already has a floating joint), so the robot
   can be repositioned via qpos.
5. **Ground + environment** — a ground box and the `env.primitives`
   (tables, crates…) are injected as links *in the same URDF*, because
   MuJoCo's `<include>` treats included files as MJCF — URDF scenes cannot
   be composed at the MJCF level. `DEFAULT_ENVIRONMENT` is a 10 m grid
   ground with default physics; pass your own `EnvironmentConfig` to
   change it. Note `env.ground.size`/`type` are behavior-relevant inputs.

### `applyURDFPass2` — the two-pass trick

MuJoCo's URDF parser silently drops `<actuator>`, `<equality>`, and
`<sensor>` declarations. So after compile #1, the *compiled* model is dumped
back to XML via `mj_saveLastXML` — where those three element classes are
native — edited, and recompiled:

- **Actuators** (`injectMJCFActuators`): one PD position actuator per
  movable hinge/slide joint, in ascending joint-id order (that order is the
  `data.ctrl` contract). Defaults mirror a hobby-class servo arm: kp 50,
  damping ratio 1 (critically damped), forcerange ±3.5, armature 0.1,
  frictionloss 0.1; per-joint overrides via `URDFActuatorConfig.perJoint`.
  Each actuator's `ctrlRange` equals its joint's limit range — the same
  bounds a real servo's min/max ticks enforce. `mode: 'none'` skips
  injection entirely (torque control via `qfrc_applied`).
- **Fixed-base weld** (`injectMJCFFixedBaseWeld`): always injected,
  *inactive* — `applyFixedBase(engine, true, pos, quat)` toggles it at
  runtime through `data.eq_active`, no recompile. Physically correct: with
  the weld active a fixed base's accelerometer reads (0, 0, +9.81) instead
  of a free-fall artifact.
- **Sensors** (`injectMJCFSensors`): IMU on the root link (gyro +
  accelerometer + framequat) by default; optional horizontal lidar ring
  (`lidar: true`, default 36 rays, 5 m cutoff).

A Pass-2 failure throws — surface it loudly. A URDF arm without actuators
has no joint damping and flies apart on play.

### Helper stages

- `buildInjectableJoints(engine)` — movable joints in ascending id order
  with their ranges, skipping the injected `_simulo_*` helpers. This order
  is the actuator/servo mapping contract; treat it as stable.
- `applyHomePose(engine, homeQpos)` — writes each joint's qpos *and* its
  position target so servos hold the rest pose (used at load and reset).
- `stepTrajectoryTick(engine, runner, actuators, dt)` — one tick of a
  `TrajectoryRunner` key-pose script: rate-limited command, per-actuator
  ctrl-range clamp, write. Returns `{ done, ctrl }`.

## The MJCF path

MJCF input skips the URDF machinery and uses three string transforms
instead: `stripMJCFForPhysics` (remove mesh assets/geoms + textures that
hang the WASM build), `injectMJCFFloatingBase` (freejoint + the same
`_simulo_fixed_base` weld; keyframes get the freejoint's qpos/qvel
prepended), `injectMJCFEnvironment` (primitives into `<worldbody>`;
MuJoCo's half-extent sizes map directly). `composeScene(env)` builds an
environment-only scene when no robot is loaded.

## The compiled-model cache

`hashXml(mujocoVersion, finalXml)` keys a two-tier cache
(`getCachedBinary`/`putCachedBinary`): in-memory Map, then IndexedDB with
LRU eviction (16 entries / 100 MB). Cache hits skip MuJoCo's XML compiler —
typically the single biggest cost of a load. The version in the key makes a
library upgrade self-invalidating. **Contract:** where IndexedDB doesn't
exist (Node), reads miss and writes no-op past the memory tier — pinned by
`binaryCache.node.test.ts`. Gate the whole cache on
`installWriteI53ToI64Polyfill(mj)` succeeding (see
[worker-integration.md](./worker-integration.md)): without that helper,
`mj_saveModel` aborts the WASM runtime.

## Model identity

`modelSignature(engine)` returns `{nq, nv, nu, actuatorNames, timestep}` —
enough to assert that a dumped scene recompiles to the same model. The
platform this package was extracted from gates every export on that
recompile-identity, and holds a WASM→native replay agreement of ~1.6e-7 rad
through this exact pipeline. If you re-serialize models, gate on the
signature; XML text precision is behaviorally visible.
