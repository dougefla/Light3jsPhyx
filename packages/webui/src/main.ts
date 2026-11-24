import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import GUI from 'lil-gui';
import { Env, createCamera, frameObject, type URDFRobot, type URDFJoint } from '@lightphyx/engine';
import { PointerURDFDragControls } from 'urdf-loader/src/URDFDragControls.js';
import { runPageHost } from './pagehost.js';

// Test/automation hook: window.__lp.loaded flips once the URDF is in.
declare global {
  interface Window {
    __lp?: {
      env: Env;
      renderer: THREE.WebGLRenderer;
      THREE: typeof THREE;
      loaded: boolean;
      setOutlines?: (v: boolean) => void;
      drag?: unknown;
      camera?: THREE.PerspectiveCamera;
    };
  }
}

const pageParams = new URLSearchParams(location.search);
const pageId = pageParams.get('page');

if (pageId) {
  // Headless host mode: this page is a pool member driven over WebSocket.
  void runPageHost(pageId);
} else {
  void runInteractive();
}

async function runInteractive() {

  const canvas = document.getElementById('app') as HTMLCanvasElement;
  const hint = document.getElementById('hint') as HTMLDivElement;

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);

  const env = new Env();
  env.scene.background = new THREE.Color(0x1a1d21);

  // Z-up world: lay the grid into the XY plane.
  const grid = new THREE.GridHelper(4, 40, 0x445566, 0x2a3138);
  grid.rotation.x = Math.PI / 2;
  env.scene.add(grid);

  const camera = createCamera(50, window.innerWidth / window.innerHeight);
  camera.position.set(2, -2, 1.5);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;

  function resize() {
    renderer.setSize(window.innerWidth, window.innerHeight);
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
  resize();

  // ---- per-part outlines (CAD-style "shaded with edges") -------------------
  // Feature edges per mesh, so every link/part carries its own edge lines —
  // restores depth cues that pure uniform-ambient shading flattens out.
  const outlineMaterial = new THREE.LineBasicMaterial({ color: 0x14161a });
  const outlineLines: THREE.LineSegments[] = [];
  let outlinesEnabled = false;

  function clearOutlines() {
    for (const lines of outlineLines) {
      lines.parent?.remove(lines);
      lines.geometry.dispose();
    }
    outlineLines.length = 0;
  }

  function refreshOutlines() {
    clearOutlines();
    if (!outlinesEnabled) return;
    const meshes: THREE.Mesh[] = [];
    env.scene.traverse((o) => {
      const mesh = o as THREE.Mesh;
      // segId marks renderable content (link visuals + spawned objects) and
      // naturally excludes hidden collision copies and helpers.
      if (mesh.isMesh && mesh.userData.segId !== undefined) meshes.push(mesh);
    });
    for (const mesh of meshes) {
      const edges = new THREE.EdgesGeometry(mesh.geometry, 20);
      const lines = new THREE.LineSegments(edges, outlineMaterial);
      lines.raycast = () => {}; // never intercept joint-drag picking
      mesh.add(lines); // inherits the part's transforms, follows joints
      outlineLines.push(lines);
    }
  }

  function setOutlines(v: boolean) {
    outlinesEnabled = v;
    refreshOutlines();
  }

  // ---- drag-to-articulate ---------------------------------------------------
  // Click-drag a part to move its joint (urdf-loader's PointerURDFDragControls).
  // While playing, drags feed joint TARGETS so contents get pushed physically;
  // when paused they apply immediately.
  let loadedRobot: URDFRobot | null = null;
  let dragControls: PointerURDFDragControls | null = null;
  let dragValue = 0;
  const hoverMaterial = new THREE.LineBasicMaterial({ color: 0x4da3ff });
  const hoverLines: THREE.LineSegments[] = [];

  function clearHover() {
    for (const lines of hoverLines) {
      lines.parent?.remove(lines);
      lines.geometry.dispose();
    }
    hoverLines.length = 0;
    canvas.style.cursor = '';
  }

  function showHover(joint: URDFJoint) {
    clearHover();
    const meshes: THREE.Mesh[] = [];
    joint.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh && mesh.userData.segId !== undefined) meshes.push(mesh);
    });
    for (const mesh of meshes) {
      const lines = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 20), hoverMaterial);
      lines.raycast = () => {};
      mesh.add(lines);
      hoverLines.push(lines);
    }
    canvas.style.cursor = 'grab';
  }

  function disableDrag() {
    dragControls?.dispose();
    dragControls = null;
    clearHover();
    controls.enabled = true;
  }

  function enableDrag(robot: URDFRobot) {
    disableDrag();
    const dc = new PointerURDFDragControls(robot, camera, canvas);
    dc.onHover = showHover;
    dc.onUnhover = clearHover;
    dc.onDragStart = (joint) => {
      controls.enabled = false;
      dragValue = Number(joint.angle);
      canvas.style.cursor = 'grabbing';
    };
    dc.onDragEnd = () => {
      controls.enabled = true;
      canvas.style.cursor = dc.hovered ? 'grab' : '';
    };
    dc.updateJoint = (joint, angle) => {
      // The lib passes joint.angle + delta; re-derive delta and accumulate on
      // our own value so a physics-chased (lagging) joint still tracks the
      // pointer's full intent.
      dragValue += angle - Number(joint.angle);
      env.setJoint(joint.urdfName, dragValue, { immediate: !physParams.play });
    };
    dragControls = dc;
    if (window.__lp) window.__lp.drag = dc;
  }

  // Releasing the button outside the canvas must still end the drag.
  window.addEventListener('mouseup', () => dragControls?.setGrabbed(false));

  // ---- drag spawned objects (balls/boxes) -----------------------------------
  // Pointer-spring drag: the object is pulled toward the pointer on a
  // camera-facing plane through the grab point. Collisions keep resolving,
  // so you can drop things into drawers or press them against walls; a fast
  // release throws the object. When paused, dragging teleports instead.
  const objRaycaster = new THREE.Raycaster();
  const objMouse = new THREE.Vector2();
  const objDragPlane = new THREE.Plane();
  const objDragPoint = new THREE.Vector3();
  let draggedObject: string | null = null;

  function spawnedMeshes(): THREE.Mesh[] {
    const out: THREE.Mesh[] = [];
    for (const o of env.scene.children) {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh && mesh.userData.segId !== undefined && mesh.name !== 'ground') out.push(mesh);
    }
    return out;
  }

  function updateObjMouse(e: MouseEvent) {
    const rect = canvas.getBoundingClientRect();
    objMouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    objMouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  }

  canvas.addEventListener('mousedown', (e) => {
    updateObjMouse(e);
    objRaycaster.setFromCamera(objMouse, camera);
    const hit = objRaycaster.intersectObjects(spawnedMeshes(), false)[0];
    if (!hit) return;
    draggedObject = hit.object.name;
    objDragPlane.setFromNormalAndCoplanarPoint(
      camera.getWorldDirection(new THREE.Vector3()),
      hit.point,
    );
    env.dragObject(draggedObject, hit.point.toArray() as [number, number, number]);
    controls.enabled = false;
    canvas.style.cursor = 'grabbing';
    e.stopImmediatePropagation(); // objects win over joint-drag when in front
  });

  canvas.addEventListener('mousemove', (e) => {
    if (draggedObject) {
      updateObjMouse(e);
      objRaycaster.setFromCamera(objMouse, camera);
      if (objRaycaster.ray.intersectPlane(objDragPlane, objDragPoint)) {
        const p = objDragPoint.toArray() as [number, number, number];
        if (physParams.play) env.dragObject(draggedObject, p);
        else env.setObjectPose(draggedObject, p);
      }
      e.stopImmediatePropagation();
      return;
    }
    // hover affordance for grabbable objects (joint hover manages its own)
    updateObjMouse(e);
    objRaycaster.setFromCamera(objMouse, camera);
    if (objRaycaster.intersectObjects(spawnedMeshes(), false).length > 0) {
      canvas.style.cursor = 'grab';
    } else if (canvas.style.cursor === 'grab' && !dragControls?.hovered) {
      canvas.style.cursor = '';
    }
  });

  function endObjectDrag() {
    if (!draggedObject) return;
    if (physParams.play) env.dragObject(draggedObject, null);
    draggedObject = null;
    controls.enabled = true;
    canvas.style.cursor = '';
  }
  window.addEventListener('mouseup', endObjectDrag);

  const gui = new GUI({ title: 'light-3js-phyx' });
  const viewFolder = gui.addFolder('View');
  const viewParams = { grid: true, outlines: false, 'drag joints': true };
  viewFolder.add(viewParams, 'grid').onChange((v: boolean) => (grid.visible = v));
  viewFolder.add(viewParams, 'outlines').onChange(setOutlines);
  viewFolder.add(viewParams, 'drag joints').onChange((v: boolean) => {
    if (v && loadedRobot) enableDrag(loadedRobot);
    else disableDrag();
  });

  // ---- physics --------------------------------------------------------------
  await env.enablePhysics();
  env.addGround(0, { visual: false }); // the grid is the ground's visual

  const physParams = { play: new URLSearchParams(location.search).get('play') !== '0' };
  let dropCount = 0;

  function dropPos(): [number, number, number] {
    const target = env.robot ? new THREE.Box3().setFromObject(env.robot) : null;
    const jitter = 0.03 * ((dropCount % 5) - 2);
    dropCount++;
    if (target && !target.isEmpty()) {
      const c = target.getCenter(new THREE.Vector3());
      return [c.x + jitter, c.y + jitter, target.max.z + 0.25];
    }
    return [jitter, jitter, 1];
  }

  const physFolder = gui.addFolder('Physics');
  physFolder.add(physParams, 'play');
  physFolder.add(
    {
      'drop ball': () => {
        env.spawn({ shape: 'sphere', size: [0.035], pos: dropPos() });
        refreshOutlines();
      },
    },
    'drop ball',
  );
  physFolder.add(
    {
      'drop box': () => {
        env.spawn({ shape: 'box', size: [0.07, 0.07, 0.07], pos: dropPos() });
        refreshOutlines();
      },
    },
    'drop box',
  );

  let jointFolder: GUI | null = null;
  const jointValues: Record<string, number> = {};

  async function loadFromURL(urdfURL: string) {
    hint.hidden = false;
    hint.textContent = `loading ${urdfURL} …`;
    try {
      const robot = await env.loadURDF(urdfURL);
      hint.hidden = true;
      loadedRobot = robot;
      refreshOutlines();
      if (viewParams['drag joints']) enableDrag(robot);
      if (window.__lp) window.__lp.loaded = true;

      controls.target.copy(frameObject(camera, robot));
      controls.update();

      jointFolder = gui.addFolder('Joints');
      for (const j of env.listJoints()) {
        jointValues[j.name] = j.value;
        const lower = j.type === 'continuous' ? -Math.PI : j.lower;
        const upper = j.type === 'continuous' ? Math.PI : j.upper;
        jointFolder
          .add(jointValues, j.name, lower, upper, (upper - lower) / 200)
          .onChange((v: number) => env.setJoint(j.name, v, { immediate: !physParams.play }));
      }
      jointFolder.add(
        {
          reset: () => {
            env.resetJoints();
          },
        },
        'reset',
      );
    } catch (err) {
      hint.hidden = false;
      hint.textContent = `failed to load URDF:\n${String(err)}`;
      throw err;
    }
  }

  window.__lp = { env, renderer, THREE, loaded: false, setOutlines, camera };

  const params = new URLSearchParams(location.search);
  const urdfParam = params.get('urdf');
  if (urdfParam) {
    void loadFromURL(urdfParam);
  } else {
    hint.hidden = false;
    hint.textContent =
      'no URDF loaded\n\nopen with ?urdf=<url>\n' +
      'dev example: ?urdf=/@fs/<absolute-path>/scene.urdf';
  }

  const clock = new THREE.Clock();
  renderer.setAnimationLoop(() => {
    const dt = Math.min(clock.getDelta(), 0.05);
    if (physParams.play) env.step(dt);
    if (jointFolder && loadedRobot) {
      for (const j of env.listJoints()) jointValues[j.name] = j.value;
      for (const c of jointFolder.controllers) c.updateDisplay();
    }
    controls.update();
    renderer.render(env.scene, camera);
  });
}
