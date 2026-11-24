import * as THREE from 'three';
import { idToRGB } from '../protocol/shared.js';

/**
 * Per-env segmentation registry: stable integer ids (1..) for links and
 * spawned objects, id 0 = background. Provides the flat exact-color
 * materials for the ID pass (colors set via setRGB so ColorManagement's
 * sRGB→linear conversion never touches them — bit-exactness contract).
 */
export class SegRegistry {
  private nextId = 1;
  private readonly labels = new Map<number, string>();
  private readonly materials = new Map<string, THREE.MeshBasicMaterial>();
  /** Shared id-0 material for stray meshes that never got a label. */
  readonly backgroundMaterial: THREE.MeshBasicMaterial;

  constructor() {
    this.backgroundMaterial = this.makeMaterial(0, THREE.FrontSide);
  }

  /** Label every Mesh under root with a fresh id. Returns the id. */
  assign(label: string, root: THREE.Object3D): number {
    const id = this.nextId++;
    this.labels.set(id, label);
    root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh) mesh.userData.segId = id;
    });
    return id;
  }

  release(id: number): void {
    this.labels.delete(id);
    // Dispose the GPU materials for this id too — otherwise repeated
    // spawn/capture/reset cycles in a long-lived env leak one material per
    // episode (ids are monotonic, so the key never gets reused).
    for (const key of [...this.materials.keys()]) {
      if (key.startsWith(`${id}:`)) {
        this.materials.get(key)!.dispose();
        this.materials.delete(key);
      }
    }
  }

  labelMap(): Record<number, string> {
    return Object.fromEntries(this.labels);
  }

  /** Exact-color material for a mesh (side copied so silhouettes match RGB). */
  idMaterialFor(mesh: THREE.Mesh): THREE.MeshBasicMaterial {
    const id = (mesh.userData.segId as number | undefined) ?? 0;
    const beauty = Array.isArray(mesh.material) ? mesh.material[0] : mesh.material;
    const side = (beauty as THREE.Material | undefined)?.side ?? THREE.FrontSide;
    const key = `${id}:${side}`;
    let mat = this.materials.get(key);
    if (!mat) {
      mat = this.makeMaterial(id, side);
      this.materials.set(key, mat);
    }
    return mat;
  }

  private makeMaterial(id: number, side: THREE.Side): THREE.MeshBasicMaterial {
    const mat = new THREE.MeshBasicMaterial({
      fog: false,
      side,
      transparent: false,
      blending: THREE.NoBlending,
    });
    const [r, g, b] = idToRGB(id);
    mat.color.setRGB(r / 255, g / 255, b / 255); // working-space, no conversion
    return mat;
  }
}
