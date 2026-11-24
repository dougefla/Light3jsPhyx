import * as THREE from 'three';

/** Create a Z-up perspective camera (URDF world convention). */
export function createCamera(fov = 50, aspect = 1): THREE.PerspectiveCamera {
  const camera = new THREE.PerspectiveCamera(fov, aspect, 0.01, 100);
  camera.up.set(0, 0, 1);
  return camera;
}

/**
 * Place the camera on a 3/4 view that frames the object's bounding box.
 * Returns the box center (useful as an orbit-controls target).
 */
export function frameObject(
  camera: THREE.PerspectiveCamera,
  object: THREE.Object3D,
  fill = 0.8,
): THREE.Vector3 {
  const box = new THREE.Box3().setFromObject(object);
  if (box.isEmpty()) return new THREE.Vector3();
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const radius = size.length() / 2;
  const dist = radius / (fill * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)));
  const dir = new THREE.Vector3(1, -1, 0.6).normalize();
  camera.position.copy(center).addScaledVector(dir, dist);
  // Fit the clip planes to the object rather than using a fixed 10^4 ratio,
  // which erodes z-buffer precision and z-fights at distance.
  const margin = radius * 0.05 + 0.01;
  camera.near = Math.max(1e-3, dist - radius - margin);
  camera.far = dist + radius + margin;
  camera.lookAt(center);
  camera.updateProjectionMatrix();
  return center;
}
