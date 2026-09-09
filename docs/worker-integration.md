# Worker integration & runtime environments

The package never constructs a `Worker` and never imports bundler-specific
asset URLs — those stay with you. This doc covers what the package *does*
own (module init, the WASM build's workarounds) and the contracts you keep.

## `initMujocoModule(options)`

One call replaces the fragile parts of bringing up `@mujoco/mujoco`:

```ts
const mj = await initMujocoModule({
  wasmUrl,                       // your bundler's URL for mujoco.wasm
  print:    (t) => console.log(`[MuJoCo] ${t}`),
  printErr: (t) => console.error(`[MuJoCo] ${t}`),
  onAbort:  (what) => { /* see the abort contract below */ },
  // patchMemoryMax: false       // opt out of the 2→4 GiB patch
})
const engine = new MuJoCoEngine(mj)
```

What it handles, in order:

1. **Dynamic import** of `@mujoco/mujoco` — the WASM fetch happens in your
   execution context (worker or main thread), never at module-parse time.
2. **`locateFile` capture** — Emscripten's resolved WASM URL is preferred;
   `options.wasmUrl` is the fallback. Under bundlers whose wasm transform
   bypasses `locateFile` (Vite production builds), the fallback is the one
   that fires — treat `wasmUrl` as required in bundled browser builds.
3. **The pthread stub** — MuJoCo 3.x's single-threaded build still calls
   `pthread_create` during model compilation; without SharedArrayBuffer
   that's a fatal `std::system_error`. The `instantiateWasm` hook patches
   `__pthread_create_js` to a success no-op *before* instantiation (the
   import table is immutable afterwards). The engine silently runs
   single-threaded.
4. **The memory-max patch** (`patchMujocoWasmMemoryMax`) — rewrites the
   WASM binary's memory-import maximum from 32768 pages (2 GiB) to 65536
   (4 GiB) so large models can grow the arena. This alone is *valid* — the
   JS-side `WebAssembly.Memory` may still declare a 2 GiB max (growth just
   caps there). To actually reach 4 GiB you must also raise the JS side
   (Emscripten's `getHeapMax` / the `Memory` the glue creates) with a
   build-time transform of `mujoco.js`; the originating platform does this
   with a small Vite plugin. Both sides must agree or instantiation fails
   validation.

## The abort contract (yours to keep)

Emscripten `abort()` fires when MuJoCo calls `mju_error()` internally —
missing meshes, malformed XML. **No JS exception is thrown; the runtime is
simply dead**, and every later call into the module may hang forever.

`onAbort` is the notification. Recovery is the consumer's job, and the
proven pattern is fail-fast:

```ts
let wasmAborted = false
// in onAbort:  wasmAborted = true; fail the in-flight request explicitly
// before every subsequent operation:  if (wasmAborted) throw/refuse
```

In a worker, that means rejecting every message after abort and telling the
main thread to restart the worker. Don't retry into a dead runtime.

## `mj_saveModel` and the polyfill

The current `@mujoco/mujoco` build omits `writeI53ToI64` from its exported
runtime methods, and even *reading* `mj.mj_saveModel` transitively hits the
poisoned getter and aborts. Call
`installWriteI53ToI64Polyfill(mj)` immediately after init; the boolean it
returns is your "binary cache available" flag:

```ts
const cacheAvailable = installWriteI53ToI64Polyfill(mj)
// gate engine.saveBinary()/loadBinary() + the binaryCache on this flag
```

If upstream ships the helper, the function detects it and returns true with
no patching — the cache activates with no code change.

## Bundler notes (Vite)

- `import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url'` — pass it as
  `options.wasmUrl`.
- `worker: { format: 'es' }` — the Emscripten glue references `Worker`
  internally and the default iife worker format rejects its top-level
  await.
- `optimizeDeps: { exclude: ['@mujoco/mujoco'] }` — esbuild pre-bundling
  breaks the glue's `locateFile`/`instantiateWasm` hooks.

The [example app's vite config](../example/vite.config.ts) is the minimal
working reference.

## Node

The same engine code runs under Node (the originating platform's headless
runners and this package's own `.mujoco` gates depend on it). Two Node
specifics:

- `initMujocoModule`'s `instantiateWasm` hook uses `fetch(url)` — fine for
  http(s) URLs, but under plain Node you don't need the hook at all:
  call the factory directly and Emscripten loads the wasm from disk:

  ```ts
  const mod = await import('@mujoco/mujoco')
  const mj = await mod.default({ print: () => {}, printErr: () => {} })
  const engine = new MuJoCoEngine(mj)
  ```

- `binaryCache` degrades to memory-only where `indexedDB` is undefined —
  documented and tested (`binaryCache.node.test.ts`). No shimming needed.

## Worker architecture (recommended shape)

Keep MuJoCo off the main thread. The shape that works: a worker module that
owns `initMujocoModule` + one `MuJoCoEngine`, speaks a typed
request/response protocol over `postMessage`, runs the sim loop on
`setTimeout` (no rAF in workers), and applies the abort contract above.
Construct it yourself:

```ts
new Worker(new URL('./sim.worker.ts', import.meta.url), { type: 'module' })
```

That call deliberately lives in *your* code — worker construction is
bundler-specific and this package stays bundler-agnostic.
