import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';

export interface LoadURDFOptions {
  /** Parse <collision> geometry (required for physics). Default true. */
  parseCollision?: boolean;
  fetchOptions?: RequestInit;
  /** Share one cache across loads of the same mesh URLs. Default true. */
  useMeshCache?: boolean;
}

/** Finite fallback for a joint limit declared as ±inf (effectively unbounded). */
const FINITE_LIMIT = 1e6;

async function assetSignature(url: string, manager: THREE.LoadingManager): Promise<string> {
  try {
    const res = await fetch(manager.resolveURL(url), { method: 'HEAD' });
    if (!res.ok) return '';
    return `${res.headers.get('last-modified') ?? ''}|${res.headers.get('content-length') ?? ''}`;
  } catch {
    return '';
  }
}

function dirnameURL(url: string): string {
  const i = url.lastIndexOf('/');
  return i >= 0 ? url.slice(0, i + 1) : '';
}

async function loadOBJWithMTL(url: string, manager: THREE.LoadingManager): Promise<THREE.Group> {
  const res = await fetch(manager.resolveURL(url));
  if (!res.ok) throw new Error(`Failed to fetch mesh ${url}: ${res.status}`);
  const text = await res.text();
  const dir = dirnameURL(url);

  // Resolve the MTL from the OBJ's own `mtllib` statement (guessing
  // "<name>.mtl" from the OBJ filename is a known-fragile pattern).
  const mtlName = /^[ \t]*mtllib[ \t]+(.+?)[ \t]*$/m.exec(text)?.[1];
  let materials: MTLLoader.MaterialCreator | undefined;
  if (mtlName) {
    const mtlLoader = new MTLLoader(manager);
    mtlLoader.setPath(dir);
    mtlLoader.setResourcePath(dir);
    try {
      materials = await mtlLoader.loadAsync(mtlName);
      materials.preload();
    } catch {
      materials = undefined; // missing/broken MTL: fall back to default material
    }
  }

  const objLoader = new OBJLoader(manager);
  if (materials) objLoader.setMaterials(materials);
  const group = objLoader.parse(text);

  // Color textures must be sampled as sRGB or rendering looks washed out.
  group.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh) return;
    const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (const m of mats) {
      const map = (m as THREE.MeshPhongMaterial).map;
      if (map) map.colorSpace = THREE.SRGBColorSpace;
    }
  });
  return group;
}

/**
 * Template cache: mesh URL -> loaded prototype group (instances are clones),
 * tagged with the file's last-modified/content-length signature so a mesh
 * rewritten at the same path during the page's lifetime is reloaded instead
 * of served stale.
 *
 * Clones share the template's BufferGeometry / Material objects, and three.js
 * keeps the GPU copies of those alive until `.dispose()` is called. Without an
 * owner, every distinct mesh ever loaded stayed resident in the GPU process
 * for the page's lifetime (measured: +3 GB per browser over a ~1.5 h training
 * run that exported ~1300 unique OBJs). Templates are therefore ref-counted:
 * `loadURDF` acquires one ref per mesh reference, `Env.removeRobot` releases
 * them, and a template is disposed once nobody references it AND it has been
 * evicted — either by a signature change (rewritten file) or by the bounded
 * idle LRU below.
 */
interface TemplateEntry {
  url: string;
  sig: string;
  group: Promise<THREE.Group>;
  /** Robots currently holding clones of this template. */
  refs: number;
  /** No longer the cache's entry for `url`: dispose when refs reach 0. */
  evicted: boolean;
  /** Monotonic LRU stamp (bumped on every acquire). */
  lastUse: number;
}

const meshCache = new Map<string, TemplateEntry>();
let useStamp = 0;

/** Idle (unreferenced) templates kept warm for reuse. Configurable. */
export const DEFAULT_MESH_CACHE_IDLE_LIMIT = 64;
let idleLimit = DEFAULT_MESH_CACHE_IDLE_LIMIT;

/** Free the GPU resources of a template group (geometry, materials, textures). */
function disposeGroup(group: THREE.Group): void {
  const materials = new Set<THREE.Material>();
  group.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh) return;
    mesh.geometry?.dispose();
    for (const m of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
      if (m) materials.add(m);
    }
  });
  for (const m of materials) {
    // MTL materials carry textures (map, bumpMap, ...); each is a GPU object.
    for (const v of Object.values(m as unknown as Record<string, unknown>)) {
      if ((v as THREE.Texture | null)?.isTexture) (v as THREE.Texture).dispose();
    }
    m.dispose();
  }
}

function disposeEntry(entry: TemplateEntry): void {
  // A rejected load never produced a group: nothing to free.
  entry.group.then(disposeGroup, () => undefined);
}

/** Drop `entry` from the cache; free it now if unreferenced, else on last release. */
function evict(entry: TemplateEntry): void {
  if (meshCache.get(entry.url) === entry) meshCache.delete(entry.url);
  if (entry.evicted) return;
  entry.evicted = true;
  if (entry.refs === 0) disposeEntry(entry);
}

/** Evict least-recently-used idle templates beyond the idle limit. */
function trimIdle(): void {
  const idle = [...meshCache.values()].filter((e) => e.refs === 0);
  if (idle.length <= idleLimit) return;
  idle.sort((a, b) => a.lastUse - b.lastUse);
  for (const entry of idle.slice(0, idle.length - idleLimit)) evict(entry);
}

export interface MeshTemplateHandle {
  group: Promise<THREE.Group>;
  /** Give back this reference. Idempotent. */
  release(): void;
}

/**
 * Acquire one reference to the template for `url` (loading it on a miss or a
 * signature change). The caller owns exactly one `release()`.
 */
export async function acquireMeshTemplate(
  url: string,
  manager: THREE.LoadingManager,
  useCache = true,
): Promise<MeshTemplateHandle> {
  let entry: TemplateEntry;
  if (!useCache) {
    // Private template: freed as soon as its single owner lets go.
    entry = { url, sig: '', group: loadOBJWithMTL(url, manager), refs: 0, evicted: true, lastUse: 0 };
  } else {
    const sig = await assetSignature(url, manager);
    const hit = meshCache.get(url);
    if (hit && hit.sig === sig) {
      entry = hit;
    } else {
      // Rewritten file (or first load): the stale template is evicted and
      // freed once the robots still cloning it are removed.
      if (hit) evict(hit);
      const p = loadOBJWithMTL(url, manager);
      entry = { url, sig, group: p, refs: 0, evicted: false, lastUse: 0 };
      meshCache.set(url, entry);
      // Do not cache failures, or a transient error would poison every retry.
      p.catch(() => evict(entry));
    }
  }
  entry.refs++;
  entry.lastUse = ++useStamp;
  let released = false;
  return {
    group: entry.group,
    release: () => {
      if (released) return;
      released = true;
      entry.refs = Math.max(0, entry.refs - 1);
      if (entry.refs === 0 && entry.evicted) disposeEntry(entry);
      else trimIdle();
    },
  };
}

/** Bound on unreferenced templates kept for reuse; excess is evicted LRU-first. */
export function setMeshCacheIdleLimit(limit: number): void {
  idleLimit = Math.max(0, Math.floor(limit));
  trimIdle();
}

export function meshCacheStats(): { entries: number; referenced: number; idle: number } {
  let referenced = 0;
  for (const e of meshCache.values()) if (e.refs > 0) referenced++;
  return { entries: meshCache.size, referenced, idle: meshCache.size - referenced };
}

/**
 * Evict every template. Unreferenced ones are freed immediately; referenced
 * ones once their robots are removed.
 */
export function clearMeshCache(): void {
  for (const entry of [...meshCache.values()]) evict(entry);
}

/** Robots loaded by `loadURDF` carry this so `Env.removeRobot` can free meshes. */
export const RELEASE_MESHES = 'releaseMeshes';

/** Release the mesh template references held by a robot. Idempotent. */
export function releaseRobotMeshes(robot: THREE.Object3D): void {
  const release = robot.userData[RELEASE_MESHES] as (() => void) | undefined;
  if (release) {
    delete robot.userData[RELEASE_MESHES];
    release();
  }
}

/**
 * Load a URDF (with OBJ+MTL+texture meshes) into a URDFRobot.
 * Resolves only after every referenced mesh has finished loading.
 */
export async function loadURDF(url: string, opts: LoadURDFOptions = {}): Promise<URDFRobot> {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.parseCollision = opts.parseCollision ?? true;
  if (opts.fetchOptions) loader.fetchOptions = opts.fetchOptions;

  const pending: Promise<void>[] = [];
  const handles: MeshTemplateHandle[] = [];
  const releaseAll = () => {
    for (const h of handles) h.release();
    handles.length = 0;
  };
  loader.loadMeshCb = (meshUrl, mgr, _material, onLoad) => {
    // A mesh failure must propagate — silently returning a link with no
    // geometry/collider would let dataset generation proceed on a broken
    // scene with no error. Re-throw through the pending promise so loadURDF
    // rejects rather than resolving a half-loaded robot.
    pending.push(
      acquireMeshTemplate(meshUrl, mgr, opts.useMeshCache ?? true).then(async (handle) => {
        handles.push(handle);
        onLoad((await handle.group).clone());
      }),
    );
  };

  // Texture loads (kicked off by MTLLoader.preload) register on this manager
  // but are NOT part of the mesh promises — wait for them too so a capture
  // immediately after load_urdf is deterministic, not occasionally untextured.
  let assetsStarted = false;
  const managerIdle = new Promise<void>((resolve, reject) => {
    manager.onStart = () => {
      assetsStarted = true;
    };
    manager.onLoad = () => resolve();
    manager.onError = (u) => reject(new Error(`asset failed to load: ${u}`));
  });

  let robot: URDFRobot;
  try {
    robot = await loader.loadAsync(url);
    // loadAsync resolves when the URDF is parsed; meshes may still be in flight.
    // Any mesh failing rejects here — no partial robot escapes.
    await Promise.all(pending);
    // Drain outstanding texture loads. `assetsStarted` guards the "no async
    // assets" case so this never hangs when the manager was never used.
    if (assetsStarted) await managerIdle;
  } catch (err) {
    // A half-loaded robot must not pin its templates forever. Meshes still
    // in flight settle later; release whatever they acquired then too.
    await Promise.allSettled(pending);
    releaseAll();
    throw err;
  }
  // The robot owns one ref per mesh reference; Env.removeRobot gives them back.
  robot.userData[RELEASE_MESHES] = releaseAll;

  // Non-finite joint limits are legal URDF (some ArtVIP prismatics use ±inf)
  // but make clamping a no-op, letting inf/NaN flow into the transform. Clamp
  // them to a large finite bound so the limit stays effective and can never
  // seed a NaN. Warn so a genuinely-unbounded joint is visible.
  for (const [name, joint] of Object.entries(robot.joints)) {
    const lim = joint.limit as { lower: number; upper: number };
    if (!Number.isFinite(Number(lim.lower)) || !Number.isFinite(Number(lim.upper))) {
      console.warn(
        `[urdf] joint '${name}' has a non-finite limit ` +
          `[${lim.lower}, ${lim.upper}] — clamping to ±${FINITE_LIMIT}`,
      );
      if (!Number.isFinite(Number(lim.lower))) lim.lower = -FINITE_LIMIT;
      if (!Number.isFinite(Number(lim.upper))) lim.upper = FINITE_LIMIT;
    }
  }

  // Collision geometry participates in physics, never in rendering — and
  // never in raycasting (its poses can be wrong in the wild, see DESIGN §5.2).
  robot.traverse((o) => {
    if ((o as { isURDFCollider?: boolean }).isURDFCollider) {
      o.visible = false;
      o.traverse((m) => {
        m.raycast = () => {};
      });
    }
  });
  return robot;
}
