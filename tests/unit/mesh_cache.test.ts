import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import * as THREE from 'three';
import {
  acquireMeshTemplate,
  clearMeshCache,
  meshCacheStats,
  setMeshCacheIdleLimit,
  DEFAULT_MESH_CACHE_IDLE_LIMIT,
} from '@lightphyx/engine';

// One triangle; enough for OBJLoader to produce a Mesh with a BufferGeometry.
const OBJ = 'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n';

/** In-memory static server: url -> { body, sig }. Rewriting bumps the sig. */
const files = new Map<string, { body: string; sig: string }>();
let fetches = 0;

function installFetch(): void {
  vi.stubGlobal('fetch', async (input: string | URL, init?: RequestInit) => {
    const url = String(input);
    const f = files.get(url);
    if (!f) return new Response(null, { status: 404 });
    const headers = { 'last-modified': f.sig, 'content-length': String(f.body.length) };
    if (init?.method === 'HEAD') return new Response(null, { status: 200, headers });
    fetches++;
    return new Response(f.body, { status: 200, headers });
  });
}

/** Resolve the template and count geometry dispose events on it. */
async function watchDispose(group: Promise<THREE.Group>): Promise<() => number> {
  let n = 0;
  (await group).traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (mesh.isMesh) mesh.geometry.addEventListener('dispose', () => n++);
  });
  return () => n;
}

const manager = new THREE.LoadingManager();
const flush = () => new Promise((r) => setTimeout(r, 0));

beforeEach(() => {
  files.clear();
  fetches = 0;
  installFetch();
  clearMeshCache();
  setMeshCacheIdleLimit(DEFAULT_MESH_CACHE_IDLE_LIMIT);
});
afterEach(() => vi.unstubAllGlobals());

describe('mesh template cache', () => {
  it('shares one template between references and keeps it while referenced', async () => {
    files.set('/a.obj', { body: OBJ, sig: 't1' });
    const h1 = await acquireMeshTemplate('/a.obj', manager);
    const h2 = await acquireMeshTemplate('/a.obj', manager);
    expect(await h1.group).toBe(await h2.group);
    expect(fetches).toBe(1);
    const disposed = await watchDispose(h1.group);

    setMeshCacheIdleLimit(0); // no idle slots at all
    h1.release();
    await flush();
    expect(disposed()).toBe(0); // h2 still holds it
    expect(meshCacheStats()).toEqual({ entries: 1, referenced: 1, idle: 0 });

    h2.release();
    h2.release(); // idempotent
    await flush();
    expect(disposed()).toBe(1);
    expect(meshCacheStats().entries).toBe(0);
  });

  it('keeps idle templates warm up to the limit and evicts LRU-first', async () => {
    for (const n of ['a', 'b', 'c']) files.set(`/${n}.obj`, { body: OBJ, sig: 't' });
    setMeshCacheIdleLimit(2);
    const a = await acquireMeshTemplate('/a.obj', manager);
    const b = await acquireMeshTemplate('/b.obj', manager);
    const c = await acquireMeshTemplate('/c.obj', manager);
    const aDisposed = await watchDispose(a.group);
    const bDisposed = await watchDispose(b.group);
    a.release();
    b.release();
    c.release(); // three idle > limit 2: 'a' (least recently used) goes
    await flush();
    expect(aDisposed()).toBe(1);
    expect(bDisposed()).toBe(0);
    expect(meshCacheStats()).toEqual({ entries: 2, referenced: 0, idle: 2 });

    // A warm hit costs no fetch.
    const again = await acquireMeshTemplate('/b.obj', manager);
    expect(await again.group).toBe(await b.group);
    expect(fetches).toBe(3);
    again.release();
  });

  it('a rewritten file replaces the template and frees the stale one after its last user', async () => {
    files.set('/a.obj', { body: OBJ, sig: 't1' });
    const old = await acquireMeshTemplate('/a.obj', manager);
    const oldDisposed = await watchDispose(old.group);

    files.set('/a.obj', { body: OBJ, sig: 't2' });
    const fresh = await acquireMeshTemplate('/a.obj', manager);
    expect(await fresh.group).not.toBe(await old.group);
    expect(meshCacheStats().entries).toBe(1); // only the new one is cached
    await flush();
    expect(oldDisposed()).toBe(0); // still cloned by `old`'s owner

    old.release();
    await flush();
    expect(oldDisposed()).toBe(1);
    fresh.release();
  });

  it('clearMeshCache frees idle templates now and referenced ones on release', async () => {
    files.set('/a.obj', { body: OBJ, sig: 't' });
    files.set('/b.obj', { body: OBJ, sig: 't' });
    const a = await acquireMeshTemplate('/a.obj', manager);
    const b = await acquireMeshTemplate('/b.obj', manager);
    const aDisposed = await watchDispose(a.group);
    const bDisposed = await watchDispose(b.group);
    a.release();
    clearMeshCache();
    await flush();
    expect(aDisposed()).toBe(1);
    expect(bDisposed()).toBe(0);
    expect(meshCacheStats().entries).toBe(0);
    b.release();
    await flush();
    expect(bDisposed()).toBe(1);
  });

  it('an uncached template is private and freed on release', async () => {
    files.set('/a.obj', { body: OBJ, sig: 't' });
    const h = await acquireMeshTemplate('/a.obj', manager, false);
    const disposed = await watchDispose(h.group);
    expect(meshCacheStats().entries).toBe(0);
    h.release();
    await flush();
    expect(disposed()).toBe(1);
  });

  it('a failed load is not cached and releasing it is harmless', async () => {
    const h = await acquireMeshTemplate('/missing.obj', manager);
    await expect(h.group).rejects.toThrow(/404/);
    await flush();
    expect(meshCacheStats().entries).toBe(0);
    h.release();
  });
});
