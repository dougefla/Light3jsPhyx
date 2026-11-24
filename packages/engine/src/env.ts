import * as THREE from 'three';
import type { URDFRobot, URDFJoint } from 'urdf-loader';
import { loadURDF, releaseRobotMeshes, type LoadURDFOptions } from './urdf/load.js';
import { initRapier } from './physics/rapier.js';
import { SegRegistry } from './passes/seg.js';
import {
  PhysicsWorld,
  type PhysicsOptions,
  type RobotPhysicsOptions,
  type DriveOptions,
  type SpawnOptions,
  type ObjectState,
} from './physics/world.js';

export interface JointInfo {
  name: string;
  type: URDFJoint['jointType'];
  lower: number;
  upper: number;
  value: number;
}

export interface EnvOptions {
  /** Uniform ambient light intensity. Default 1.0. */
  ambientIntensity?: number;
}

/** Throw if any coordinate is inf/NaN — they silently poison scene matrices. */
function assertFinite(values: readonly number[], what: string): void {
  for (const v of values) {
    if (!Number.isFinite(v)) throw new Error(`${what} must be finite, got [${values.join(', ')}]`);
  }
}

let nextRobotId = 0;

/**
 * One isolated simulation environment: a scene holding articulated robots.
 * World coordinates follow URDF conventions: Z-up, meters, radians.
 * Physics (M2) attaches to this class; rendering lives outside (renderer is
 * shared across envs in headless mode).
 */
export class Env {
  readonly scene = new THREE.Scene();
  readonly robots = new Map<string, URDFRobot>();
  readonly seg = new SegRegistry();
  private readonly segIds = new Map<string, number>();
  private readonly ambient: THREE.AmbientLight;
  private physicsWorld: PhysicsWorld | null = null;
  private groundMesh: THREE.Mesh | null = null;
  /** Per-robot physics mode, kept so enablePhysics() after loadURDF() can honour it. */
  private readonly robotPhysics = new Map<string, RobotPhysicsOptions>();

  constructor(opts: EnvOptions = {}) {
    // Physically-based lighting divides Lambert diffuse by π; multiply it back
    // so intensity 1.0 means "rendered color == albedo" (uniform WYSIWYG light).
    this.ambient = new THREE.AmbientLight(0xffffff, (opts.ambientIntensity ?? 1.0) * Math.PI);
    this.scene.add(this.ambient);
  }

  /** The first (usually only) robot, for single-object workflows. */
  get robot(): URDFRobot | null {
    return this.robots.values().next().value ?? null;
  }

  async loadURDF(
    url: string,
    opts: LoadURDFOptions & {
      name?: string;
      pos?: [number, number, number];
      yaw?: number;
      /** Physics mode for this robot (dynamic links, drive gains); see RobotPhysicsOptions. */
      physics?: RobotPhysicsOptions;
    } = {},
  ): Promise<URDFRobot> {
    const robot = await loadURDF(url, opts);
    const name = opts.name ?? (robot.robotName || `robot_${nextRobotId++}`);
    if (this.robots.has(name)) throw new Error(`Robot name already in use: ${name}`);
    robot.name = name;
    if (opts.pos) robot.position.set(...opts.pos);
    if (opts.yaw) robot.rotation.z = opts.yaw;
    this.robots.set(name, robot);
    this.scene.add(robot);
    this.scene.updateMatrixWorld(true);
    // One segmentation id per link, in URDF document order.
    for (const [linkName, link] of Object.entries(robot.links)) {
      for (const child of link.children) {
        if (!(child as { isURDFVisual?: boolean }).isURDFVisual) continue;
        const key = `${name}/${linkName}`;
        const id = this.segIds.get(key) ?? this.seg.assign(key, child);
        this.segIds.set(key, id);
        child.traverse((o) => {
          const mesh = o as THREE.Mesh;
          if (mesh.isMesh) mesh.userData.segId = id;
        });
      }
    }
    this.robotPhysics.set(name, opts.physics ?? {});
    this.physicsWorld?.addRobot(robot, opts.physics ?? {});
    return robot;
  }

  removeRobot(name: string): boolean {
    const robot = this.robots.get(name);
    if (!robot) return false;
    this.physicsWorld?.removeRobot(robot);
    this.scene.remove(robot);
    this.robots.delete(name);
    this.robotPhysics.delete(name);
    // Give back the mesh template refs so their GPU buffers can be freed.
    releaseRobotMeshes(robot);
    for (const [key, id] of [...this.segIds]) {
      if (key.startsWith(`${name}/`)) {
        this.seg.release(id);
        this.segIds.delete(key);
      }
    }
    return true;
  }

  // ---- physics ------------------------------------------------------------

  get physics(): PhysicsWorld | null {
    return this.physicsWorld;
  }

  /** Turn physics on (idempotent). Already-loaded robots are registered. */
  async enablePhysics(opts: PhysicsOptions = {}): Promise<void> {
    if (this.physicsWorld) return;
    await initRapier();
    this.physicsWorld = new PhysicsWorld(opts);
    for (const robot of this.robots.values()) {
      this.physicsWorld.addRobot(robot, this.robotPhysics.get(robot.name) ?? {});
    }
  }

  /** True when the robot was loaded with dynamic links (physics on). */
  isDynamic(robotName?: string): boolean {
    return this.physicsWorld?.isDynamic(this.resolveRobot(robotName)) ?? false;
  }

  /**
   * Set PD gains of a dynamic robot's joint motor. stiffness 0 makes the
   * joint passive (a door that only moves when pushed). Throws for
   * kinematic robots or without physics.
   */
  setJointDrive(name: string, opts: DriveOptions, robotName?: string): void {
    for (const v of [opts.stiffness, opts.damping]) {
      if (v !== undefined && !(Number.isFinite(v) && v >= 0)) {
        throw new Error(`Joint '${name}' drive gains must be finite and ≥ 0`);
      }
    }
    this.requirePhysics().setJointDrive(this.resolveRobot(robotName), name, opts);
  }

  private requirePhysics(): PhysicsWorld {
    if (!this.physicsWorld) throw new Error('Physics not enabled: call enablePhysics() first');
    return this.physicsWorld;
  }

  /**
   * Add the ground plane at height z (physics collider + visual disc).
   * Idempotent: the physics collider is created once and can't move, so a
   * second call must not leave a stale visual plane at the old height while
   * collisions stay at the first z (visual/physics desync). Repeat calls only
   * reposition the existing visual plane.
   */
  addGround(z = 0, opts: { visual?: boolean; halfExtent?: number } = {}): void {
    const physics = this.requirePhysics();
    physics.addGround(z, opts.halfExtent ?? 10);
    if (!(opts.visual ?? true)) return;
    const size = (opts.halfExtent ?? 10) * 2;
    if (this.groundMesh) {
      this.groundMesh.position.z = z;
      return;
    }
    this.groundMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(size, size),
      // polygonOffset pushes the ground back in depth so an object resting
      // exactly at z (bottom face coplanar with the plane) always wins the
      // depth tie — removes coplanar z-fighting independent of clip planes.
      new THREE.MeshPhongMaterial({
        color: 0x808080,
        polygonOffset: true,
        polygonOffsetFactor: 1,
        polygonOffsetUnits: 1,
      }),
    );
    this.groundMesh.name = 'ground';
    this.groundMesh.position.z = z;
    this.scene.add(this.groundMesh);
    this.seg.assign('ground', this.groundMesh);
  }

  /** Drop a dynamic object into the scene. Returns its name. */
  spawn(opts: SpawnOptions): string {
    assertFinite(opts.pos, `spawn '${opts.name ?? '?'}' pos`);
    assertFinite(opts.size, `spawn '${opts.name ?? '?'}' size`);
    const { name, mesh } = this.requirePhysics().spawn(opts, this.scene);
    this.segIds.set(`obj:${name}`, this.seg.assign(`obj:${name}`, mesh));
    return name;
  }

  /** Drag a spawned object toward a world point; null releases the drag. */
  dragObject(name: string, point: [number, number, number] | null): void {
    if (point) assertFinite(point, `drag '${name}' point`);
    this.requirePhysics().setDragTarget(name, point);
  }

  /** Teleport a spawned object (zeroes its velocity). */
  setObjectPose(name: string, pos: [number, number, number], quat?: [number, number, number, number]): void {
    assertFinite(pos, `object '${name}' pos`);
    if (quat) assertFinite(quat, `object '${name}' quat`);
    this.requirePhysics().setObjectPose(name, pos, quat);
  }

  removeObject(name: string): boolean {
    const removed = this.requirePhysics().removeObject(name, this.scene);
    if (removed) {
      const id = this.segIds.get(`obj:${name}`);
      if (id !== undefined) {
        this.seg.release(id);
        this.segIds.delete(`obj:${name}`);
      }
    }
    return removed;
  }

  /**
   * Advance the simulation by dt (seconds). Physics runs in fixed 1/60
   * substeps; without physics this is a no-op (joints apply instantly).
   */
  step(dt: number, n = 1): void {
    if (!this.physicsWorld) return;
    for (let i = 0; i < n; i++) this.physicsWorld.step(dt);
  }

  /** Currently-touching collider pairs (updated during step()). */
  contacts(): Array<[string, string]> {
    return this.physicsWorld?.contacts() ?? [];
  }

  /**
   * Link pairs of one robot currently touching each other (self-collision),
   * without the `link:robot/` prefix. Needs `enablePhysics({ selfCollision:
   * true })`; otherwise (or without physics) always empty. Joint-adjacent
   * pairs are excluded by the physics layer.
   */
  selfContacts(robotName?: string): Array<[string, string]> {
    if (!this.physicsWorld) return [];
    return this.physicsWorld.selfContacts(this.resolveRobot(robotName).name);
  }

  /** Joint values plus dynamic object poses. */
  getState(): { joints: Record<string, number>; objects: Record<string, ObjectState> } {
    const joints: Record<string, number> = {};
    for (const robot of this.robots.values()) {
      for (const j of this.listJoints(robot.name)) {
        joints[this.robots.size > 1 ? `${robot.name}/${j.name}` : j.name] = j.value;
      }
    }
    return { joints, objects: this.physicsWorld?.objectStates() ?? {} };
  }

  private resolveRobot(robotName?: string): URDFRobot {
    const robot = robotName ? this.robots.get(robotName) : this.robot;
    if (!robot) throw new Error(robotName ? `No robot named ${robotName}` : 'No robot loaded');
    return robot;
  }

  /** Movable joints (fixed joints are omitted), in URDF document order. */
  listJoints(robotName?: string): JointInfo[] {
    const robot = this.resolveRobot(robotName);
    const infos: JointInfo[] = [];
    for (const [name, joint] of Object.entries(robot.joints)) {
      if (joint.jointType === 'fixed') continue;
      infos.push({
        name,
        type: joint.jointType,
        lower: Number(joint.limit.lower),
        upper: Number(joint.limit.upper),
        value: Number(joint.jointValue[0] ?? 0),
      });
    }
    return infos;
  }

  /**
   * Set a joint value (clamped to limits). With physics enabled this sets a
   * target the joint chases during step() under the velocity limit — pass
   * `immediate: true` to teleport (skips collision sweep, use for resets).
   * Without physics the value applies instantly.
   */
  setJoint(
    name: string,
    value: number,
    opts: { robotName?: string; immediate?: boolean } = {},
  ): boolean {
    // A non-finite joint value produces NaN scene matrices and NaN physics
    // bodies with no error — the render goes silently black. Reject at the
    // single funnel every joint write passes through (set_joint(s), reset).
    if (!Number.isFinite(value)) {
      throw new Error(`Joint '${name}' value must be finite, got ${value}`);
    }
    const robot = this.resolveRobot(opts.robotName);
    if (!robot.joints[name]) throw new Error(`No joint named ${name}`);
    if (this.physicsWorld && !opts.immediate) {
      this.physicsWorld.setJointTarget(robot, name, value);
      return true;
    }
    if (this.physicsWorld) {
      this.physicsWorld.setJointImmediate(robot, name, value);
      return true;
    }
    const changed = robot.setJointValue(name, value);
    if (changed) this.scene.updateMatrixWorld(true);
    return changed;
  }

  setJoints(
    values: Record<string, number>,
    opts: { robotName?: string; immediate?: boolean } = {},
  ): boolean {
    let changed = false;
    for (const [name, value] of Object.entries(values)) {
      changed = this.setJoint(name, value, opts) || changed;
    }
    return changed;
  }

  /** True when every joint has reached its requested target (physics mode). */
  jointsSettled(): boolean {
    return this.physicsWorld?.settled() ?? true;
  }

  getJoint(name: string, robotName?: string): number {
    const robot = this.resolveRobot(robotName);
    const joint = robot.joints[name];
    if (!joint) throw new Error(`No joint named ${name}`);
    return Number(joint.jointValue[0] ?? 0);
  }

  /** Reset all joints to 0 (teleport; clamped into limits where needed). */
  resetJoints(robotName?: string): void {
    const robot = this.resolveRobot(robotName);
    for (const [name, joint] of Object.entries(robot.joints)) {
      if (joint.jointType === 'fixed') continue;
      const lower = Number(joint.limit.lower);
      const upper = Number(joint.limit.upper);
      const zero = THREE.MathUtils.clamp(0, lower, upper);
      this.setJoint(name, zero, { robotName: robot.name, immediate: true });
    }
    this.scene.updateMatrixWorld(true);
  }

  dispose(): void {
    for (const name of [...this.robots.keys()]) this.removeRobot(name);
    this.physicsWorld?.dispose();
    this.physicsWorld = null;
    if (this.groundMesh) {
      this.scene.remove(this.groundMesh);
      this.groundMesh = null;
    }
  }
}
