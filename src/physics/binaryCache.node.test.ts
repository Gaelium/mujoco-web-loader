// @vitest-environment node
/**
 * The documented no-IndexedDB contract (OPEN-SOURCE-SPLIT.md dual-target
 * cut, SPLIT-NOTES SP-F2): under plain Node there is no `indexedDB` global,
 * and the cache must degrade gracefully — the in-memory tier keeps working,
 * reads miss instead of throwing, and writes are silent no-ops beyond
 * memory. The headless twins run the loader under exactly this environment.
 */

import { describe, it, expect, beforeEach } from 'vitest'
import {
  hashXml,
  getCachedBinary,
  putCachedBinary,
  _clearMemoryCacheForTests,
} from './binaryCache'

beforeEach(() => {
  _clearMemoryCacheForTests()
})

describe('binaryCache without IndexedDB (Node contract)', () => {
  it('runs in an environment with no indexedDB global', () => {
    expect(typeof indexedDB).toBe('undefined')
  })

  it('getCachedBinary misses (null) instead of throwing', async () => {
    await expect(getCachedBinary('never-written')).resolves.toBeNull()
  })

  it('putCachedBinary resolves without throwing', async () => {
    await expect(putCachedBinary('k', new Uint8Array([9]))).resolves.toBeUndefined()
  })

  it('the memory tier still round-trips within the process', async () => {
    const key = hashXml('3.7.0', '<mujoco/>')
    await putCachedBinary(key, new Uint8Array([1, 2, 3]))
    const loaded = await getCachedBinary(key)
    expect(Array.from(loaded ?? [])).toEqual([1, 2, 3])
  })
})
