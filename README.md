# mujoco-web-loader

*Drop any URDF into MuJoCo-in-the-browser and it just works.*


https://github.com/user-attachments/assets/57d12009-6587-496c-9ca8-6c66a016a17b


Real-world URDFs break bare MuJoCo WASM. This is the robustness layer that
turns a broken robot file into a controllable, sensor-equipped, physically
simulated model — extracted from a production browser robotics platform and
pinned by its test suite.

## The problem

- MuJoCo's WASM build **hangs indefinitely** loading STL/OBJ meshes from the
  Emscripten VFS.
- MuJoCo's URDF parser **silently drops** `<actuator>`, `<equality>`, and
  `<sensor>` declarations — your arm has no servos and flies apart on play.
- Hand-authored URDFs ship degenerate inertias, visual-only links with no
  collision geometry, and `package://` mesh paths.
- The single-threaded WASM build still calls `pthread_create` during model
  compilation and dies without SharedArrayBuffer.

## The fix

- **Preprocess** (`preprocessURDF`): compiler hints, measured bounding-box
  substitution for meshes (visually lossless — render the real meshes
  yourself), synthesized collision geoms for visual-only links, floating
  base + ground/environment injection.
- **Two-pass injection** (`applyURDFPass2`): compile the URDF, dump the
  compiled MJCF via `mj_saveLastXML`, inject PD position actuators, a
  runtime-toggleable fixed-base weld, and IMU/lidar sensors natively, then
  recompile. See "The two-pass trick" below.
- **Worker plumbing** (`initMujocoModule`): the pthread stub, the 2→4 GiB
  WASM memory-max patch, the `writeI53ToI64` polyfill, abort hooks. You keep
  worker construction; this package supplies the module init. The WASM URL
  is a config parameter — no bundler lock-in.
- **Compiled-model cache** (`binaryCache`): hash(version + XML) → `.mjb`,
  memory + IndexedDB tiers; degrades gracefully to memory-only where
  IndexedDB doesn't exist (Node) — a documented, tested contract.
- **Dual-target**: the exact same code runs in a browser worker and under
  Node. The platform this was extracted from holds a WASM→native replay
  identity of 1.6e-7 rad through this pipeline.

## Quickstart

```ts
import {
  MuJoCoEngine, initMujocoModule, stripUnsupportedURDFElements,
  preprocessURDF, applyURDFPass2, computeSTLBounds, DEFAULT_ENVIRONMENT,
  type MeshBoundsInfo,
} from '@robot-sim/mujoco-web-loader'
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url' // your bundler's URL

const mj = await initMujocoModule({ wasmUrl })
const engine = new MuJoCoEngine(mj)
engine.ensureVFSDir('/working')
engine.ensureVFSDir('/working/assets')

const meshBounds = new Map<string, MeshBoundsInfo>()
for (const [name, bytes] of stlFiles) {          // your URDF's mesh files
  engine.writeToVFS(`/working/assets/${name}`, bytes)
  const info = computeSTLBounds(bytes)
  if (info) meshBounds.set(name, info)
}

const stripped = stripUnsupportedURDFElements(urdfXml)
engine.loadModel(preprocessURDF(stripped, DEFAULT_ENVIRONMENT, meshBounds), '/working/model.xml')
const robot = applyURDFPass2(engine, mj, urdfXml, {}, undefined, '/working/model_pass2.xml')
// robot.actuators now has a PD position actuator per movable joint.
engine.stepN(8) // …and step in your sim loop
```

## The two-pass trick

MuJoCo's URDF parser accepts an embedded `<mujoco>` extension block — but
silently drops `<actuator>`, `<equality>`, and `<sensor>` declarations
inside it. One-pass loading therefore cannot give a URDF arm servos. The
loader instead compiles the URDF (Pass 1), asks MuJoCo to serialize the
*compiled* model back out via `mj_saveLastXML` — where those elements are
native MJCF — injects actuators, a fixed-base weld (inactive, runtime
toggleable via `data.eq_active`), and sensors into that dump, and
recompiles (Pass 2). The result is byte-stable enough that the originating
platform gates on recompile-identity of the dumped scene.

## Example app

`example/` is a minimal Vite app: pick a `.urdf` (plus its STL meshes) with
a file input and watch it simulate.

```sh
npm install
npm run example:dev
```

It works with any URDF; the AM-ARM200 (Li Yiteng & Wu Zhiyong, Apache-2.0)
is a good test subject.

## Documentation

- [The load pipeline, end to end](./docs/pipeline.md) — every repair stage,
  the two-pass injection internals, the cache, model identity.
- [Worker integration & runtime environments](./docs/worker-integration.md) —
  `initMujocoModule`, the abort contract, bundler notes, Node usage.
- [API reference](./docs/api.md) — the full export map, module by module.

## Run the tests

```sh
npm install
npm run typecheck && npm run test:run
```

The suite ships with the package — mock-driven engine tests plus the
Node-contract tests for the cache's no-IndexedDB path.

## Built by

**Simulo**, the browser robotics platform → https://simulo.dev

## License

Apache-2.0 (matches MuJoCo upstream). See [LICENSE](./LICENSE) and
[NOTICE](./NOTICE.md).
