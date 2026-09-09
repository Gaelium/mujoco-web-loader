import { dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
import { defineConfig } from 'vite'

export default defineConfig({
  root: dirname(fileURLToPath(import.meta.url)),
  base: './',
  build: {
    // Top-level await in main.ts (module init) needs a modern target.
    target: 'esnext',
    outDir: 'dist',
    emptyOutDir: true,
  },
  worker: {
    // The Emscripten glue references Worker internally; the default iife
    // worker format rejects its top-level await.
    format: 'es',
  },
  optimizeDeps: {
    // esbuild pre-bundling breaks the mujoco glue (it needs its
    // locateFile/instantiateWasm hooks intact).
    exclude: ['@mujoco/mujoco'],
  },
})
