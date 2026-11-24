import * as THREE from 'three';

/**
 * CAD-style feature-edge outlines (the same EdgesGeometry overlay the WebUI
 * offers) for the headless capture path. One LineSegments per renderable mesh,
 * parented so it follows joint motion, cached and reused across frames.
 *
 * Outlines are hidden by default and only made visible around the RGB pass —
 * the seg and depth passes render with them hidden, so segmentation IDs and
 * depth values are byte-for-byte identical whether outlines are on or off.
 */
/**
 * Page-wide cache of edge geometry keyed by SOURCE geometry (+ threshold).
 * URDF loads clone the cached mesh template, and clones share the template's
 * BufferGeometry — so a workload that reloads a scene per frame (the evo
 * oracle renders a fresh flattened URDF for every view) gets its edges from
 * here instead of recomputing EdgesGeometry for every mesh every frame
 * (~18 ms/frame at 512² on a 20-part cabinet). Entries are ref-counted by
 * the LineSegments using them and LRU-evicted only once unreferenced.
 */
interface EdgeEntry {
  edges: THREE.EdgesGeometry;
  refs: number;
  /** Source geometry was disposed: nothing can hit this key again. */
  orphaned: boolean;
}
const edgeCache = new Map<string, EdgeEntry>();
const EDGE_CACHE_MAX = 512;

function acquireEdges(geometry: THREE.BufferGeometry, angleThreshold: number): THREE.EdgesGeometry {
  const key = `${geometry.uuid}:${angleThreshold}`;
  let entry = edgeCache.get(key);
  if (entry) {
    edgeCache.delete(key); // LRU touch
  } else {
    const created: EdgeEntry = {
      edges: new THREE.EdgesGeometry(geometry, angleThreshold),
      refs: 0,
      orphaned: false,
    };
    entry = created;
    // The mesh template cache disposes a source geometry once no robot clones
    // it; its edges would otherwise sit here (up to EDGE_CACHE_MAX of them,
    // MBs each) waiting for a key that can never be requested again.
    geometry.addEventListener('dispose', () => {
      created.orphaned = true;
      if (created.refs > 0) return; // freed on the last releaseEdges()
      if (edgeCache.get(key) === created) edgeCache.delete(key);
      created.edges.dispose();
    });
    // Evict the least-recently-used unreferenced entries beyond the cap.
    if (edgeCache.size >= EDGE_CACHE_MAX) {
      for (const [k, e] of edgeCache) {
        if (e.refs > 0) continue;
        e.edges.dispose();
        edgeCache.delete(k);
        if (edgeCache.size < EDGE_CACHE_MAX) break;
      }
    }
  }
  entry.refs++;
  edgeCache.set(key, entry);
  return entry.edges;
}

function releaseEdges(edges: THREE.EdgesGeometry): void {
  for (const [k, e] of edgeCache) {
    if (e.edges === edges) {
      e.refs = Math.max(0, e.refs - 1);
      if (e.refs === 0 && e.orphaned) {
        edgeCache.delete(k);
        e.edges.dispose();
      }
      return;
    }
  }
  edges.dispose(); // not cached (shouldn't happen) — free it outright
}

export class Outliner {
  private readonly material = new THREE.LineBasicMaterial({ color: 0x14161a });
  /** mesh.uuid -> its outline line segments. */
  private readonly lines = new Map<string, THREE.LineSegments>();
  private readonly angleThreshold: number;

  constructor(angleThreshold = 20) {
    this.angleThreshold = angleThreshold;
  }

  /**
   * Build/refresh outline geometry for every renderable mesh under root
   * (segId-tagged: link visuals + spawned objects, never collision copies).
   * Adds outlines for new meshes and disposes those whose mesh is gone.
   */
  sync(root: THREE.Object3D): void {
    const live = new Set<string>();
    root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh || mesh.userData.segId === undefined) return;
      live.add(mesh.uuid);
      if (this.lines.has(mesh.uuid)) return;
      const line = new THREE.LineSegments(
        acquireEdges(mesh.geometry, this.angleThreshold),
        this.material,
      );
      line.visible = false;
      line.raycast = () => {}; // never intercept picking
      line.userData.isOutline = true;
      mesh.add(line); // inherits the mesh's world transform (follows joints)
      this.lines.set(mesh.uuid, line);
    });
    // Drop outlines whose source mesh was removed.
    for (const [uuid, line] of [...this.lines]) {
      if (live.has(uuid)) continue;
      line.parent?.remove(line);
      releaseEdges(line.geometry as THREE.EdgesGeometry);
      this.lines.delete(uuid);
    }
  }

  setVisible(visible: boolean): void {
    for (const line of this.lines.values()) line.visible = visible;
  }

  dispose(): void {
    for (const line of this.lines.values()) {
      line.parent?.remove(line);
      releaseEdges(line.geometry as THREE.EdgesGeometry);
    }
    this.lines.clear();
    this.material.dispose();
  }
}
