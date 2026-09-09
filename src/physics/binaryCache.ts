/**
 * Two-tier cache for compiled MuJoCo binary models.
 *
 * Tier 1: in-memory `Map` keyed by hash(MuJoCo version + final XML).
 *   Survives the lifetime of the worker. Hot path — synchronous lookup.
 *
 * Tier 2: IndexedDB store at `simulo-mjb-cache.binaries`.
 *   Survives across page reloads. First-load-after-refresh hits IDB.
 *
 * The cache key embeds the MuJoCo version so a library upgrade auto-invalidates
 * stale entries (binary models are not portable across versions).
 *
 * Eviction is simplistic: the in-memory cache is unbounded but each entry is
 * small (kilobytes for typical robots), and the IDB store keeps a trailing
 * `accessedAt` timestamp so an LRU sweep can prune to a fixed budget. The
 * sweep runs lazily after each write when the store grows past a threshold.
 *
 * Runs inside the physics worker — uses the global IndexedDB API directly.
 */

const DB_NAME = 'simulo-mjb-cache'
const DB_VERSION = 1
const STORE_NAME = 'binaries'

/** Maximum number of cached binaries kept in IndexedDB. */
const MAX_CACHE_ENTRIES = 16
/** Approximate byte budget (per-entry, total) for the IDB cache. */
const MAX_CACHE_BYTES = 100 * 1024 * 1024

interface CacheEntry {
  /** Cache key (hash of mujoco version + final XML). */
  readonly key: string
  /** Compiled binary bytes. */
  readonly bytes: Uint8Array
  /** Wall-clock time of last read or write — used for LRU eviction. */
  accessedAt: number
}

const memoryCache = new Map<string, Uint8Array>()

/** Open (or create) the IDB database. Returns null if IndexedDB is unavailable. */
function openDb(): Promise<IDBDatabase | null> {
  return new Promise((resolve) => {
    if (typeof indexedDB === 'undefined') {
      resolve(null)
      return
    }
    const req = indexedDB.open(DB_NAME, DB_VERSION)
    req.onupgradeneeded = () => {
      const db = req.result
      if (!db.objectStoreNames.contains(STORE_NAME)) {
        db.createObjectStore(STORE_NAME, { keyPath: 'key' })
      }
    }
    req.onsuccess = () => resolve(req.result)
    req.onerror = () => {
      console.warn('[binaryCache] IDB open failed:', req.error)
      resolve(null)
    }
  })
}

function promisify<T>(req: IDBRequest<T>): Promise<T> {
  return new Promise((resolve, reject) => {
    req.onsuccess = () => resolve(req.result)
    req.onerror = () => reject(req.error ?? new Error('IDB request failed'))
  })
}

/**
 * Compute a fast non-cryptographic hash of a string. Used to key cached
 * binaries by their source XML. Collisions are virtually impossible for
 * the small set of XMLs the user will load in practice.
 */
export function hashXml(version: string, xml: string): string {
  let h = 2166136261 // FNV-1a 32-bit offset basis
  const composite = `${version}\u0000${xml}`
  for (let i = 0; i < composite.length; i++) {
    h ^= composite.charCodeAt(i)
    h = Math.imul(h, 16777619) // FNV prime
  }
  return (h >>> 0).toString(36) + '-' + composite.length.toString(36)
}

/** Look up a cached binary. Checks memory first, then IDB. */
export async function getCachedBinary(key: string): Promise<Uint8Array | null> {
  const mem = memoryCache.get(key)
  if (mem) return mem

  const db = await openDb()
  if (!db) return null
  try {
    const tx = db.transaction(STORE_NAME, 'readwrite')
    const store = tx.objectStore(STORE_NAME)
    const entry = (await promisify(store.get(key))) as CacheEntry | undefined
    if (!entry) {
      db.close()
      return null
    }
    // Bump accessedAt so the LRU sweep keeps recently-used entries.
    entry.accessedAt = Date.now()
    store.put(entry)
    await new Promise<void>((resolve) => {
      tx.oncomplete = () => resolve()
      tx.onerror = () => resolve()
    })
    db.close()
    memoryCache.set(key, entry.bytes)
    return entry.bytes
  } catch (err) {
    console.warn('[binaryCache] IDB read failed:', err)
    db.close()
    return null
  }
}

/**
 * Store a binary in the cache. Always writes to memory; opportunistically
 * persists to IDB in the background. Failures are non-fatal.
 */
export async function putCachedBinary(
  key: string,
  bytes: Uint8Array,
): Promise<void> {
  memoryCache.set(key, bytes)

  const db = await openDb()
  if (!db) return
  try {
    const tx = db.transaction(STORE_NAME, 'readwrite')
    const store = tx.objectStore(STORE_NAME)
    const entry: CacheEntry = { key, bytes, accessedAt: Date.now() }
    store.put(entry)
    await new Promise<void>((resolve) => {
      tx.oncomplete = () => resolve()
      tx.onerror = () => resolve()
    })
    await evictIfOverBudget(db)
  } catch (err) {
    console.warn('[binaryCache] IDB write failed:', err)
  } finally {
    db.close()
  }
}

/**
 * Drop the least-recently-used entries until the cache fits inside both
 * the entry-count and byte-size budgets. Cheap to run after each write
 * because the typical store size is well under the limits.
 */
async function evictIfOverBudget(db: IDBDatabase): Promise<void> {
  try {
    const tx = db.transaction(STORE_NAME, 'readwrite')
    const store = tx.objectStore(STORE_NAME)
    const all = (await promisify(store.getAll())) as CacheEntry[]
    if (all.length <= MAX_CACHE_ENTRIES) {
      const totalBytes = all.reduce((sum, e) => sum + e.bytes.byteLength, 0)
      if (totalBytes <= MAX_CACHE_BYTES) return
    }
    all.sort((a, b) => a.accessedAt - b.accessedAt) // oldest first
    let totalBytes = all.reduce((sum, e) => sum + e.bytes.byteLength, 0)
    let count = all.length
    for (const entry of all) {
      if (count <= MAX_CACHE_ENTRIES && totalBytes <= MAX_CACHE_BYTES) break
      store.delete(entry.key)
      totalBytes -= entry.bytes.byteLength
      count--
    }
    await new Promise<void>((resolve) => {
      tx.oncomplete = () => resolve()
      tx.onerror = () => resolve()
    })
  } catch (err) {
    console.warn('[binaryCache] eviction sweep failed:', err)
  }
}

/** Test-only: drop everything from the in-memory cache. */
export function _clearMemoryCacheForTests(): void {
  memoryCache.clear()
}
