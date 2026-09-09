/**
 * Reusable MuJoCo-WASM worker plumbing, extracted verbatim from the app's
 * `SimWorker.worker.ts` (OPEN-SOURCE-SPLIT.md — the one genuine refactor of
 * the loader extraction). Function bodies are the worker's originals; only
 * module-init gained an options object so the WASM URL is a config parameter
 * instead of a bundler-specific `?url` import.
 *
 * What lives here:
 * - `stripMJCFForPhysics` — remove mesh/texture references that hang the
 *   WASM build's VFS loader.
 * - `injectMJCFFloatingBase` — freejoint + `_simulo_fixed_base` weld
 *   injection for runtime-toggleable fixed base.
 * - `injectMJCFEnvironment` — environment primitives into `<worldbody>`.
 * - `installWriteI53ToI64Polyfill` — unblocks `mj_saveModel` on builds that
 *   omit the Emscripten helper from EXPORTED_RUNTIME_METHODS.
 * - `patchMujocoWasmMemoryMax` — lift the memory-import max from 2 GiB to
 *   4 GiB so large models can grow the arena.
 * - `initMujocoModule` — the init dance: dynamic import, locateFile URL
 *   capture, print/printErr capture, the abort hook, and the
 *   `instantiateWasm` hook that stubs `__pthread_create_js` and applies the
 *   memory patch before instantiation.
 *
 * The abort-RECOVERY contract stays with the consumer: after `onAbort` fires
 * the WASM runtime is dead and every subsequent call may hang — a consumer
 * must fail fast (the app worker keeps a `wasmAborted` flag and rejects all
 * further requests). Worker construction (`new Worker(new URL(...))`) also
 * stays with the consumer — this package never constructs workers.
 */

import type { MjModule } from './MuJoCoEngine'
import type { EnvironmentConfig } from '../types/environment'

// ---------------------------------------------------------------------------
// MJCF mesh stripping
// ---------------------------------------------------------------------------

/**
 * Strip mesh assets and mesh-referencing geoms from an MJCF XML string.
 *
 * MuJoCo's WASM build hangs when loading mesh files (STL/OBJ) from the
 * Emscripten VFS — the same issue that affects URDFs. For URDFs we replace
 * mesh geometry with bounding boxes. For MJCF we take a simpler approach:
 * remove `<mesh>` asset definitions entirely, and replace any geom that
 * references a mesh (via the `mesh="..."` attribute) with a tiny invisible
 * box. This prevents MuJoCo from attempting to load mesh files while
 * preserving the body hierarchy and geom-to-body associations.
 *
 * Three.js renders the real meshes independently using the original XML
 * stored in the simulation store, so this is visually lossless.
 *
 * @param mjcfXml - Raw MJCF XML string (a single file, not the scene)
 * @returns Preprocessed XML with mesh references removed
 */
export function stripMJCFForPhysics(mjcfXml: string): string {
  let xml = mjcfXml

  // Remove <mesh file="..." .../> elements from the <asset> section.
  // These cause MuJoCo to read from VFS which hangs in WASM.
  xml = xml.replace(/<mesh\s[^>]*file="[^"]*"[^>]*\/>/g, '')

  // Replace <geom ... mesh="name" .../> with <geom ... type="box" size="0.001 0.001 0.001" />.
  // Geoms that reference a mesh (via mesh="..." attribute without explicit type)
  // would fail to compile after mesh assets are removed. Replace them with tiny
  // box geoms that preserve body association and transforms. The tiny size makes
  // them invisible in the viewport and negligible for collision.
  xml = xml.replace(
    /<geom\s([^>]*)\bmesh="([^"]*)"([^>]*)\/>/g,
    (_match, before: string, _meshName: string, after: string) => {
      // Strip the mesh attribute and any existing type attribute
      const attrs = (before + after)
        .replace(/\bmesh="[^"]*"/g, '')
        .replace(/\btype="[^"]*"/g, '')
        .trim()
      return `<geom type="box" size="0.001 0.001 0.001" ${attrs}/>`
    },
  )

  // Remove <texture .../> elements — procedural texture generation
  // (especially large skybox textures) hangs the MuJoCo WASM build.
  xml = xml.replace(/<texture\s[^>]*\/>/g, '')

  // Strip texture references from materials so they don't reference
  // removed textures. Keep materials for their rgba/reflectance.
  xml = xml.replace(/\btexture="[^"]*"/g, '')
  xml = xml.replace(/\btexuniform="[^"]*"/g, '')
  xml = xml.replace(/\btexrepeat="[^"]*"/g, '')

  return xml
}

/**
 * Extract the name of the first body declared inside `<worldbody>`.
 * Used to identify the robot's root body so we can target it with a weld
 * equality constraint for fixed-base support.
 */
function findFirstWorldbodyBodyName(mjcfXml: string): string | null {
  const match = mjcfXml.match(/<worldbody>[\s\S]*?<body\s+[^>]*name="([^"]+)"/)
  return match ? match[1] : null
}

/**
 * Inject a floating base joint + weld equality into the robot's root body.
 *
 * The freejoint lets the base move freely under gravity. The weld equality
 * (named `_simulo_fixed_base`) is the mechanism for pinning the base to
 * the world when fixedBase=true — MuJoCo's `data.eq_active` flag toggles it
 * at runtime, so switching fixed/free never requires recompilation.
 *
 * Physically correct fixed-base behavior falls out of the constraint solver:
 * with the weld active, the base's qacc is zero in steady state, which makes
 * accelerometer readings read the expected `(0, 0, +9.81)` upright instead of
 * the free-fall artifact the previous save/restore hack produced.
 *
 * Keyframes are updated to prepend the freejoint's default qpos/qvel values.
 *
 * @param mjcfXml - MJCF XML (already stripped of meshes/textures)
 * @returns MJCF with floating base + weld equality injected
 */
export function injectMJCFFloatingBase(mjcfXml: string): string {
  // Skip if the model already has a free joint
  if (/<freejoint/.test(mjcfXml) || /\btype="free"/.test(mjcfXml)) {
    return mjcfXml
  }

  // Skip if the first worldbody-rooted body already has a joint of any
  // kind. Adding a freejoint would push the body past 6 DOFs and MuJoCo
  // rejects the model. This protects test fixtures and hand-authored MJCFs
  // (pendulums, single-joint manipulators) where the root body owns its
  // own anchor joint.
  const firstBodyContent = mjcfXml.match(
    /<worldbody>[\s\S]*?<body\b[^>]*>([\s\S]*?)(?=<body\b|<\/body>)/,
  )
  if (firstBodyContent && /<joint\b/.test(firstBodyContent[1])) {
    return mjcfXml
  }

  let xml = mjcfXml.replace(
    /(<worldbody>[\s\S]*?<body\s[^>]*>)/,
    '$1\n      <freejoint name="_simulo_floating_base"/>',
  )

  // Prepend freejoint default qpos (pos [0,0,0] + quat [1,0,0,0])
  xml = xml.replace(
    /(<key\s[^>]*\bqpos=")([^"]*)/g,
    '$10 0 0 1 0 0 0 $2',
  )

  // Prepend freejoint default qvel (6 DOFs)
  xml = xml.replace(
    /(<key\s[^>]*\bqvel=")([^"]*)/g,
    '$10 0 0 0 0 0 $2',
  )

  // Inject a weld equality pinning the root body to the world.
  // body2 is omitted (defaults to world). Initial active flag is off —
  // we enable it at load time based on effectiveFixedBase.
  const rootBody = findFirstWorldbodyBodyName(xml)
  if (rootBody && !/<equality>[\s\S]*?_simulo_fixed_base/.test(xml)) {
    const weldBlock = `\n  <equality>\n    <weld name="_simulo_fixed_base" body1="${rootBody}" active="false"/>\n  </equality>`
    if (/<\/equality>/.test(xml)) {
      // Model already has an <equality> block — add the weld inside it.
      xml = xml.replace(
        /<\/equality>/,
        `  <weld name="_simulo_fixed_base" body1="${rootBody}" active="false"/>\n  </equality>`,
      )
    } else {
      // No existing <equality> block — add one right before </mujoco>.
      xml = xml.replace(/<\/mujoco>/, `${weldBlock}\n</mujoco>`)
    }
  }

  return xml
}

/**
 * Inject environment primitives into an MJCF XML string.
 *
 * For MJCF models, environment primitives (tables, crates, etc.) are added
 * as `<body>` elements inside `<worldbody>`. Dynamic primitives (mass > 0)
 * get a `<freejoint/>` so they fall under gravity. Static ones are fixed
 * to the world (no joint = welded to parent).
 *
 * MuJoCo uses half-extents for box sizes, which matches ScenePrimitive.size
 * directly (unlike URDF which uses full extents).
 *
 * @param mjcfXml - Composed MJCF XML (already stripped of meshes/textures)
 * @param env - Current environment configuration
 * @returns MJCF with environment primitives injected
 */
export function injectMJCFEnvironment(mjcfXml: string, env: EnvironmentConfig): string {
  if (env.primitives.length === 0) return mjcfXml

  let injection = ''
  for (const prim of env.primitives) {
    const [px, py, pz] = prim.position
    const [qw, qx, qy, qz] = prim.quaternion
    const isDynamic = prim.mass > 0

    let geomAttrs: string
    switch (prim.shape) {
      case 'box':
        geomAttrs = `type="box" size="${prim.size[0]} ${prim.size[1]} ${prim.size[2]}"`
        break
      case 'sphere':
        geomAttrs = `type="sphere" size="${prim.size[0]}"`
        break
      case 'cylinder':
        geomAttrs = `type="cylinder" size="${prim.size[0]} ${prim.size[1]}"`
        break
      case 'capsule':
        geomAttrs = `type="capsule" size="${prim.size[0]} ${prim.size[1]}"`
        break
      case 'mesh':
        // Approximate imported meshes with bounding box
        geomAttrs = `type="box" size="${prim.size[0]} ${prim.size[1]} ${prim.size[2]}"`
        break
    }

    const rgba = `rgba="${prim.rgba[0]} ${prim.rgba[1]} ${prim.rgba[2]} ${prim.rgba[3]}"`
    const massAttr = isDynamic ? `mass="${prim.mass}"` : ''

    injection += `
    <body name="_simulo_prim_${prim.id}" pos="${px} ${py} ${pz}" quat="${qw} ${qx} ${qy} ${qz}">
      ${isDynamic ? '<freejoint/>' : ''}
      <geom ${geomAttrs} ${rgba} ${massAttr}/>
    </body>`
  }

  // Inject before the closing </worldbody> tag
  return mjcfXml.replace('</worldbody>', `${injection}\n  </worldbody>`)
}

// ---------------------------------------------------------------------------
// writeI53ToI64 polyfill
// ---------------------------------------------------------------------------

/**
 * Install a JS-level polyfill for `writeI53ToI64` on the MuJoCo WASM module.
 *
 * This Emscripten runtime helper writes a 53-bit integer (a JavaScript safe
 * integer) into a 64-bit slot in WASM linear memory. The current
 * `@mujoco/mujoco` package does not include it in EXPORTED_RUNTIME_METHODS,
 * so accessing `mj.writeI53ToI64` invokes a poisoned `abort()` getter.
 * Without it, the Embind glue for `mj_saveModel` cannot run — and reading
 * `mj.mj_saveModel` itself transitively triggers the same abort.
 *
 * The Emscripten reference implementation is:
 *
 *   function writeI53ToI64(ptr, num) {
 *     HEAPU32[ptr >> 2] = num;
 *     HEAPU32[(ptr + 4) >> 2] = (num - HEAPU32[ptr >> 2]) / 4294967296;
 *   }
 *
 * — i.e. write the unsigned-truncated low 32 bits, then derive the upper
 * 32 bits from the difference. Exact for all integers in [-2^53, 2^53].
 *
 * The polyfill resolves the Uint32 view on every call so that memory
 * growth (which detaches the previous ArrayBuffer) is handled correctly.
 *
 * @returns true if a real or polyfilled implementation is now in place
 */
export function installWriteI53ToI64Polyfill(mj: unknown): boolean {
  const mjAny = mj as Record<string, unknown>
  const desc = Object.getOwnPropertyDescriptor(mjAny, 'writeI53ToI64')
  if (desc && typeof desc.value === 'function') {
    return true // upstream ships it; nothing to do
  }
  if (desc && desc.configurable === false) {
    console.warn('[SimWorker] writeI53ToI64 descriptor not configurable; cannot polyfill')
    return false
  }

  // Resolve a stable accessor for the WASM linear-memory buffer. Try
  // `mj.wasmMemory` first (Emscripten's standard exposure for the memory
  // object); fall back to reading `mj.HEAPU32.buffer` if that property is
  // a real own-data property; bail out otherwise.
  let getBuffer: (() => ArrayBuffer | SharedArrayBuffer) | null = null
  const memDesc = Object.getOwnPropertyDescriptor(mjAny, 'wasmMemory')
  if (
    memDesc &&
    typeof memDesc.value === 'object' &&
    memDesc.value !== null &&
    'buffer' in (memDesc.value as object)
  ) {
    const wasmMem = memDesc.value as WebAssembly.Memory
    getBuffer = () => wasmMem.buffer
  }
  if (!getBuffer) {
    const heapDesc = Object.getOwnPropertyDescriptor(mjAny, 'HEAPU32')
    if (heapDesc && heapDesc.value instanceof Uint32Array) {
      // Capture the typed array's buffer; re-derive the view per call so
      // memory growth (which replaces the buffer) is handled.
      const wasmMem = (heapDesc.value as Uint32Array).buffer
      getBuffer = () => wasmMem
    }
  }
  if (!getBuffer) {
    console.warn('[SimWorker] Cannot polyfill writeI53ToI64: WASM memory not accessible')
    return false
  }

  const memAccess = getBuffer
  Object.defineProperty(mjAny, 'writeI53ToI64', {
    configurable: true,
    writable: true,
    enumerable: false,
    value: function writeI53ToI64Polyfill(ptr: number, num: number): void {
      const view = new Uint32Array(memAccess())
      const idx = ptr >>> 2
      const lower = num >>> 0
      view[idx] = lower
      view[idx + 1] = (num - view[idx]) / 4294967296
    },
  })
  return true
}

// ---------------------------------------------------------------------------
// WASM memory-max patch
// ---------------------------------------------------------------------------

/**
 * Patch the WASM binary's memory-import declaration to lift the maximum
 * from 32768 pages (2 GiB) to 65536 pages (4 GiB — wasm32's architectural
 * ceiling). Required so large humanoid models can grow MuJoCo's arena past
 * 2 GiB without aborting the runtime.
 *
 * The matching JS-side patch (raising `getHeapMax()` and the default
 * `WebAssembly.Memory` `maximum`) lives in the `mujoco-memory-patch` Vite
 * plugin in the consumer's vite.config.ts. Both must agree, since WASM
 * validation rejects an imported memory whose `maximum` exceeds what the
 * module declared.
 *
 * Parses the imports section in place; modifies a single ULEB128 byte
 * (32768 = `0x80 0x80 0x02`, 65536 = `0x80 0x80 0x04`, same length) and
 * returns a fresh Uint8Array. If the expected pattern isn't found (e.g.
 * upstream changed), returns the input unchanged and logs a warning.
 */
export function patchMujocoWasmMemoryMax(bytes: Uint8Array): Uint8Array {
  // WASM magic + version check.
  if (
    bytes.length < 8 ||
    bytes[0] !== 0x00 ||
    bytes[1] !== 0x61 ||
    bytes[2] !== 0x73 ||
    bytes[3] !== 0x6d
  ) {
    console.warn('[SimWorker] WASM patch: not a wasm module; skipping')
    return bytes
  }

  const readULEB = (off: number): { value: number; size: number } => {
    let result = 0
    let shift = 0
    let i = 0
    while (i < 10) {
      const b = bytes[off + i]
      result |= (b & 0x7f) << shift
      i++
      if ((b & 0x80) === 0) return { value: result, size: i }
      shift += 7
    }
    throw new Error('ULEB128 overflow')
  }

  let off = 8 // past magic (4) + version (4)
  while (off < bytes.length) {
    const sectionId = bytes[off]
    off += 1
    const sectionSize = readULEB(off)
    off += sectionSize.size
    const sectionEnd = off + sectionSize.value

    if (sectionId !== 2) {
      // Imports = section id 2
      off = sectionEnd
      continue
    }

    const importCount = readULEB(off)
    off += importCount.size

    for (let i = 0; i < importCount.value; i++) {
      const modLen = readULEB(off); off += modLen.size + modLen.value
      const nmLen = readULEB(off); off += nmLen.size + nmLen.value
      const kind = bytes[off]; off += 1

      if (kind === 0x00) {
        // function: type index
        off += readULEB(off).size
      } else if (kind === 0x01) {
        // table: reftype + limits
        off += 1
        const flags = bytes[off]; off += 1
        off += readULEB(off).size
        if (flags & 1) off += readULEB(off).size
      } else if (kind === 0x02) {
        // memory: flags + initial [+ maximum]
        const flags = bytes[off]; off += 1
        off += readULEB(off).size // initial
        if (flags & 1) {
          const maxOff = off
          const max = readULEB(off)
          // Only patch if it's exactly the 3-byte encoding of 32768.
          if (max.value === 32768 && max.size === 3) {
            const out = new Uint8Array(bytes)
            out[maxOff] = 0x80
            out[maxOff + 1] = 0x80
            out[maxOff + 2] = 0x04 // 65536 in ULEB128
            return out
          }
          off += max.size
        }
      } else if (kind === 0x03) {
        // global: valtype + mut
        off += 2
      }
    }
    // Imports parsed; no memory match found.
    break
  }

  console.warn('[SimWorker] WASM patch: memory-import max=32768 not found; leaving 2GB cap in place')
  return bytes
}

// ---------------------------------------------------------------------------
// Module init
// ---------------------------------------------------------------------------

/** Hooks + config for `initMujocoModule`. */
export interface MujocoInitOptions {
  /**
   * URL of `mujoco.wasm`, supplied by the consumer's bundler (e.g. a Vite
   * `?url` import). Used as the fallback when Emscripten's `locateFile`
   * doesn't yield a usable URL — which is the normal production case under
   * bundlers whose wasm transform bypasses `locateFile`. Effectively
   * required in bundled browser builds.
   */
  readonly wasmUrl?: string
  /** MuJoCo stdout capture — critical for seeing compilation errors. */
  readonly print?: (text: string) => void
  /** MuJoCo stderr capture — errors are printed here before abort() fires. */
  readonly printErr?: (text: string) => void
  /**
   * Emscripten abort() hook. Fires when MuJoCo calls mju_error() internally
   * (missing meshes, malformed XML, etc.) — this kills the WASM runtime
   * silently, no JS exception. After it fires, ALL subsequent calls into the
   * module may hang: consumers must fail fast (see module docs).
   */
  readonly onAbort?: (what: unknown) => void
  /**
   * Lift the WASM memory-import maximum from 2 GiB to 4 GiB before
   * instantiation (see `patchMujocoWasmMemoryMax`). Default: true.
   */
  readonly patchMemoryMax?: boolean
}

/**
 * Load and instantiate the `@mujoco/mujoco` WASM module with the workarounds
 * the current build needs, extracted verbatim from the app worker's `init`
 * handler:
 *
 * MuJoCo 3.x's single-threaded WASM build still contains C++ ThreadPool code
 * that calls pthread_create during model compilation. The C++ threading
 * infrastructure is not conditionally compiled out, so even the
 * single-threaded build attempts to create threads. Without
 * SharedArrayBuffer (which requires Cross-Origin-Isolation headers),
 * Emscripten's pthread stub returns EAGAIN (6), causing a fatal C++
 * std::system_error that kills the WASM runtime.
 *
 * Fix: the instantiateWasm hook below patches __pthread_create_js in the
 * WASM import table with a no-op returning 0 (success) BEFORE the module is
 * instantiated. This must happen before WebAssembly.instantiate() because
 * the import table is immutable after compilation. locateFile runs first to
 * capture the WASM URL, then instantiateWasm fetches, patches, and
 * instantiates. MuJoCo's ThreadPool constructor "succeeds" (creating zero
 * threads) and the engine degrades to single-threaded execution silently.
 *
 * @param options - WASM URL + stdout/stderr/abort hooks (all optional)
 * @returns the instantiated module, ready for `new MuJoCoEngine(mj)`
 */
export async function initMujocoModule(options: MujocoInitOptions = {}): Promise<MjModule> {
  // Dynamic import so the WASM binary is only fetched inside the consumer's
  // execution context (worker or Node process).
  const mujocoModule = await import('@mujoco/mujoco')

  const wasmUrls: string[] = []
  const mj = await mujocoModule.default({
    locateFile(path: string, scriptDir: string) {
      const url = scriptDir + path
      if (path.endsWith('.wasm')) wasmUrls.push(url)
      return url
    },
    print(text: string) {
      options.print?.(text)
    },
    printErr(text: string) {
      options.printErr?.(text)
    },
    onAbort(what: unknown) {
      options.onAbort?.(what)
    },
    instantiateWasm(
      info: WebAssembly.Imports,
      receiveInstance: (inst: WebAssembly.Instance, mod: WebAssembly.Module) => void,
    ): Record<string, never> {
      const env = info['env'] as Record<string, unknown> | undefined
      if (env && '__pthread_create_js' in env) {
        env['__pthread_create_js'] = () => 0
      }
      const url = wasmUrls[0] ?? options.wasmUrl
      if (url === undefined) {
        console.error('[SimWorker] WASM load failed: no wasm URL (locateFile yielded none and options.wasmUrl not set)')
        return {} as Record<string, never>
      }
      void fetch(url)
        .then((r) => {
          if (!r.ok) throw new Error(`WASM fetch ${r.status}: ${url}`)
          return r.arrayBuffer()
        })
        .then((buf) => {
          // Lift the WASM memory-import maximum from 2GB to 4GB. The
          // JS-side equivalent (Module.wasmMemory + getHeapMax) is
          // patched by the `mujoco-memory-patch` Vite plugin.
          const patched =
            options.patchMemoryMax === false
              ? new Uint8Array(buf)
              : patchMujocoWasmMemoryMax(new Uint8Array(buf))
          return WebAssembly.instantiate(patched, info)
        })
        .then((result) => {
          // `WebAssembly.instantiate(BufferSource, ...)` returns a
          // `WebAssemblyInstantiatedSource`, but TS picks the Module
          // overload here under newer lib types — hence the two-step
          // cast through `unknown`.
          const src = result as unknown as WebAssembly.WebAssemblyInstantiatedSource
          receiveInstance(src.instance, src.module)
        })
        .catch((err) => console.error('[SimWorker] WASM load failed:', err))
      return {} as Record<string, never>
    },
  }) as unknown as MjModule

  return mj
}
