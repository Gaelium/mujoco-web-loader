import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    // jsdom mirrors the app suite these tests were extracted from; individual
    // files opt into node via `// @vitest-environment node` (the pipeline and
    // binary-cache contract tests).
    environment: 'jsdom',
    include: ['src/**/*.test.ts'],
  },
})
