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

describe('binaryCache.hashXml', () => {
  it('produces stable hashes for identical inputs', () => {
    const a = hashXml('315', '<mujoco/>')
    const b = hashXml('315', '<mujoco/>')
    expect(a).toBe(b)
  })

  it('changes when the version changes', () => {
    const v1 = hashXml('315', '<mujoco/>')
    const v2 = hashXml('316', '<mujoco/>')
    expect(v1).not.toBe(v2)
  })

  it('changes when the xml changes', () => {
    const a = hashXml('315', '<mujoco><body/></mujoco>')
    const b = hashXml('315', '<mujoco><body name="x"/></mujoco>')
    expect(a).not.toBe(b)
  })

  it('encodes the input length so collisions across lengths are impossible', () => {
    // Different inputs that happen to share an FNV hash are still
    // distinguishable because the length suffix differs.
    const a = hashXml('v', 'abc')
    const b = hashXml('v', 'ab')
    expect(a).not.toBe(b)
  })
})

describe('binaryCache memory tier', () => {
  it('round-trips a stored binary', async () => {
    const key = hashXml('test', '<m/>')
    const bytes = new Uint8Array([1, 2, 3, 4, 5])
    await putCachedBinary(key, bytes)
    const loaded = await getCachedBinary(key)
    expect(loaded).not.toBeNull()
    expect(Array.from(loaded ?? [])).toEqual([1, 2, 3, 4, 5])
  })

  it('returns null for an unknown key', async () => {
    const loaded = await getCachedBinary('does-not-exist')
    // IndexedDB may or may not exist in the test env; either way the result
    // for a never-written key is null.
    expect(loaded).toBeNull()
  })

  it('keeps separate entries for distinct keys', async () => {
    await putCachedBinary('a', new Uint8Array([1]))
    await putCachedBinary('b', new Uint8Array([2]))
    const a = await getCachedBinary('a')
    const b = await getCachedBinary('b')
    expect(a?.[0]).toBe(1)
    expect(b?.[0]).toBe(2)
  })
})
