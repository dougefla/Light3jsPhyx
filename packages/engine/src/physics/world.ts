import * as THREE from 'three';
import type RAPIER_NS from '@dimforge/rapier3d-compat';
import { RAPIER } from './rapier.js';
import type { URDFRobot, URDFLink } from 'urdf-loader';
import { Articulation, extractTrimesh, type DriveOptions } from './articulation.js';

type RigidBody = RAPIER_NS.RigidBody;

export type { DriveOptions } from './articulation.js';

/** Per-robot physics mode, chosen at addRobot()/loadURDF() time. */
export interface RobotPhysicsOptions extends DriveOptions {
  /**
   * false (default): links are kinematic bodies driven from FK — exact,
   * cheap, and nothing pushes back (self-collision is detect-only).
   * true: links are dynamic bodies joined by motored Rapier joints
   * (Isaac-style articulation). set_joint drives a PD motor; contacts
   * between links are solved, so a motored drawer pushes a passive door
   * open and a blocked joint stalls. The root link stays fixed.
   */
  dynamic?: boolean;
  /** Mass estimate density for dynamic links, kg/m³ (default 300). */
  density?: number;
  /** Motor model for dynamic links (default 'acceleration'). */
  motorModel?: 'acceleration' | 'force';
}

export interface SpawnOptions {
  shape: 'sphere' | 'box' | 'cylinder';
  /** sphere: [radius] · box: [sx, sy, sz] full extents · cylinder: [radius, height] */
  size: number[];
  pos: [number, number, number];
  quat?: [number, number, number, number];
  /** Explicit mass in kg; otherwise density (default 500 kg/m³) applies. */
  mass?: number;
  density?: number;
  friction?: number;
  restitution?: number;
  color?: number;
  name?: string;
}

export interface ObjectState {
  pos: [number, number, number];
  quat: [number, number, number, number];
}

export interface PhysicsOptions {
  gravity?: [number, number, number];
  /** Fixed integration timestep. Default 1/60. */
  fixedDt?: number;
  /**
   * Largest joint change applied in a single substep before the change is
   * spread across the substeps of one step() call (anti-tunneling).
   */
  maxJointDelta?: number;
  /**
   * Which URDF geometry becomes the link colliders. Default 'visual':
   * guarantees physics matches what is rendered, and survives datasets with
   * broken <collision> origins (ArtVIP ships collision origins of 0,0,0
   * while visual origins carry real offsets — same mesh, different pose).
   * Use 'collision' for datasets with proper simplified collision meshes.
   */
  collisionFrom?: 'visual' | 'collision';
  /**
   * Detect link–link contacts (robot self-collision, and links of different
   * robots touching). Default off: Rapier skips kinematic–kinematic pairs, and
   * trimesh-vs-trimesh narrow phase costs real time on dense meshes. When on,
   * link pairs joined directly by a joint (parent/child) are never reported —
   * they overlap by construction in most URDFs (drawer in cabinet, hinge
   * leaves) and would flood contacts() with permanent false positives.
   * Links are hollow trimeshes: a pair counts as touching while their
   * surfaces cross, not when one is fully swallowed by the other.
   */
  selfCollision?: boolean;
}

const _pos = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _mat = new THREE.Matrix4();

/**
 * Physics attachment for one Env. URDF links become kinematicPositionBased
 * bodies (trimesh colliders) driven from the three.js FK pose each substep
 * via setNextKinematic* — Rapier derives velocities from those, which is what
 * makes a moving drawer push/carry dynamic bodies. Spawned objects are
 * dynamic bodies with primitive colliders and CCD.
 */
export class PhysicsWorld {
  readonly world: RAPIER_NS.World;
  readonly fixedDt: number;
  private readonly maxJointDelta: number;
  private readonly eventQueue: RAPIER_NS.EventQueue;

  private readonly linkBodies: Array<{ link: URDFLink; body: RigidBody }> = [];
  /** Robots registered with dynamic links (see RobotPhysicsOptions.dynamic). */
  private readonly articulations = new Map<URDFRobot, Articulation>();
  /** Body-handle pairs ("lo:hi") of links joined by a joint — never a contact. */
  private readonly adjacentBodies = new Set<string>();
  private readonly hooks: RAPIER_NS.PhysicsHooks;
  private readonly objects = new Map<string, { body: RigidBody; mesh: THREE.Mesh }>();
  private readonly colliderNames = new Map<number, string>();
  private readonly activeContacts = new Set<string>();
  /** Joint values currently applied to scene graph + kinematic bodies. */
  private readonly applied = new Map<URDFRobot, Map<string, number>>();
  /** Joint values requested via setJointTarget, chased during step(). */
  private readonly targets = new Map<URDFRobot, Map<string, number>>();

  /** Pointer-spring drag constraints: object name -> world-space target. */
  private readonly dragTargets = new Map<string, { x: number; y: number; z: number }>();
  private accumulator = 0;
  private nextObjectId = 0;
  private groundBody: RigidBody | null = null;

  private readonly collisionFrom: 'visual' | 'collision';
  readonly selfCollision: boolean;

  constructor(opts: PhysicsOptions = {}) {
    const [gx, gy, gz] = opts.gravity ?? [0, 0, -9.81];
    this.world = new RAPIER.World({ x: gx, y: gy, z: gz });
    this.fixedDt = opts.fixedDt ?? 1 / 60;
    this.world.timestep = this.fixedDt;
    this.maxJointDelta = opts.maxJointDelta ?? 0.02;
    this.collisionFrom = opts.collisionFrom ?? 'visual';
    this.eventQueue = new RAPIER.EventQueue(true);
    this.selfCollision = opts.selfCollision ?? false;
    // Rapier only calls the filter for pairs where a collider carries
    // FILTER_CONTACT_PAIRS (set on link colliders when selfCollision is on,
    // and on every dynamic-link collider), so plain worlds pay nothing.
    // Returning null drops the pair before the narrow phase: no
    // trimesh–trimesh work for joint-adjacent links at all.
    this.hooks = {
      filterContactPair: (_c1, _c2, b1, b2) =>
        this.adjacentBodies.has(pairKey(b1, b2)) ? null : RAPIER.SolverFlags.COMPUTE_IMPULSE,
      filterIntersectionPair: () => true,
    };
  }

  /**
   * Register a robot. Default: every link becomes a kinematic body with
   * trimesh colliders. `opts.dynamic`: links become a motored dynamic
   * articulation instead (see RobotPhysicsOptions).
   */
  addRobot(robot: URDFRobot, opts: RobotPhysicsOptions = {}): void {
    if (this.applied.has(robot)) throw new Error('Robot already registered with physics');
    if (opts.dynamic) {
      const art = new Articulation(
        this.world,
        robot,
        { ...opts, collisionFrom: this.collisionFrom },
        (handle, linkName) => this.colliderNames.set(handle, `link:${robot.name}/${linkName}`),
      );
      for (const key of art.adjacentPairs) this.adjacentBodies.add(key);
      this.articulations.set(robot, art);
      this.applied.set(robot, snapshotJointValues(robot));
      this.targets.set(robot, snapshotJointValues(robot));
      return;
    }
    robot.updateMatrixWorld(true);
    const bodyOfLink = new Map<URDFLink, RigidBody>();
    for (const [linkName, link] of Object.entries(robot.links)) {
      const desc = RAPIER.RigidBodyDesc.kinematicPositionBased();
      link.getWorldPosition(_pos);
      link.getWorldQuaternion(_quat);
      desc.setTranslation(_pos.x, _pos.y, _pos.z);
      desc.setRotation({ x: _quat.x, y: _quat.y, z: _quat.z, w: _quat.w });
      const body = this.world.createRigidBody(desc);

      const linkInverse = _mat.copy(link.matrixWorld).invert().clone();
      const flag = this.collisionFrom === 'visual' ? 'isURDFVisual' : 'isURDFCollider';
      let colliders = 0;
      for (const child of link.children) {
        if (!(child as unknown as Record<string, unknown>)[flag]) continue;
        child.traverse((node) => {
          const mesh = node as THREE.Mesh;
          if (!mesh.isMesh) return;
          const { vertices, indices } = extractTrimesh(mesh, linkInverse);
          if (indices.length === 0) return;
          const colDesc = RAPIER.ColliderDesc.trimesh(
            vertices,
            indices,
            RAPIER.TriMeshFlags.FIX_INTERNAL_EDGES,
          );
          colDesc.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
          if (this.selfCollision) {
            colDesc.setActiveCollisionTypes(
              RAPIER.ActiveCollisionTypes.DEFAULT | RAPIER.ActiveCollisionTypes.KINEMATIC_KINEMATIC,
            );
            colDesc.setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS);
          }
          const collider = this.world.createCollider(colDesc, body);
          this.colliderNames.set(collider.handle, `link:${robot.name}/${linkName}`);
          colliders++;
        });
      }
      if (colliders > 0) {
        this.linkBodies.push({ link, body });
        bodyOfLink.set(link, body);
      } else this.world.removeRigidBody(body);
    }
    if (this.selfCollision) {
      for (const [parent, child] of adjacentLinkPairs(robot)) {
        const a = bodyOfLink.get(parent);
        const b = bodyOfLink.get(child);
        if (a && b) this.adjacentBodies.add(pairKey(a.handle, b.handle));
      }
    }
    this.applied.set(robot, snapshotJointValues(robot));
    this.targets.set(robot, snapshotJointValues(robot));
  }

  /** True when the robot's links are dynamic bodies (RobotPhysicsOptions.dynamic). */
  isDynamic(robot: URDFRobot): boolean {
    return this.articulations.has(robot);
  }

  /** Change the PD gains of a dynamic robot's joint motor (stiffness 0 = passive). */
  setJointDrive(robot: URDFRobot, name: string, opts: DriveOptions): void {
    const art = this.articulations.get(robot);
    if (!art) throw new Error('Joint drives only exist for robots loaded with dynamic: true');
    const target = this.applied.get(robot)?.get(name);
    if (target === undefined) throw new Error(`No joint named ${name}`);
    art.setDrive(name, opts, target);
  }

  removeRobot(robot: URDFRobot): void {
    const art = this.articulations.get(robot);
    if (art) {
      for (const handle of art.colliderHandles) {
        this.purgeContactsFor(this.colliderNames.get(handle));
        this.colliderNames.delete(handle);
      }
      for (const key of art.adjacentPairs) this.adjacentBodies.delete(key);
      art.dispose();
      this.articulations.delete(robot);
    }
    for (let i = this.linkBodies.length - 1; i >= 0; i--) {
      const entry = this.linkBodies[i]!;
      let owned = false;
      entry.link.traverseAncestors((a) => {
        if (a === robot) owned = true;
      });
      if (entry.link === robot) owned = true;
      if (!owned) continue;
      for (let c = 0; c < entry.body.numColliders(); c++) {
        const handle = entry.body.collider(c).handle;
        this.purgeContactsFor(this.colliderNames.get(handle));
        this.colliderNames.delete(handle);
      }
      const h = entry.body.handle;
      for (const key of this.adjacentBodies) {
        if (key.startsWith(`${h}:`) || key.endsWith(`:${h}`)) this.adjacentBodies.delete(key);
      }
      this.world.removeRigidBody(entry.body);
      this.linkBodies.splice(i, 1);
    }
    this.applied.delete(robot);
    this.targets.delete(robot);
  }

  /**
   * Drop any active-contact keys that reference a removed collider name.
   * Rapier emits the "stopped" event only on the NEXT step, by which point
   * the handle→name mapping is gone and the key would no longer match — so
   * clear it here or contacts() reports the pair forever (issue #8).
   */
  private purgeContactsFor(name: string | undefined): void {
    if (!name) return;
    for (const key of [...this.activeContacts]) {
      const [a, b] = key.split('\0'); // keys are name-a \0 name-b (see step drain)
      if (a === name || b === name) this.activeContacts.delete(key);
    }
  }

  /**
   * Request a joint value. The kinematic links chase the target during
   * step() at min(URDF velocity limit, maxJointDelta/substep) — scene graph
   * and physics stay in lockstep, and walls can never teleport past objects.
   */
  setJointTarget(robot: URDFRobot, name: string, value: number): void {
    const t = this.targets.get(robot);
    if (!t) throw new Error('Robot not registered with physics');
    const joint = robot.joints[name];
    if (!joint) throw new Error(`No joint named ${name}`);
    if (!joint.ignoreLimits && joint.jointType !== 'continuous') {
      value = Math.min(Math.max(value, Number(joint.limit.lower)), Number(joint.limit.upper));
    }
    t.set(name, value);
  }

  /** Teleport a joint: applies instantly to scene graph and physics pose. */
  setJointImmediate(robot: URDFRobot, name: string, value: number): void {
    robot.setJointValue(name, value);
    robot.updateMatrixWorld(true);
    const v = Number(robot.joints[name]?.jointValue[0] ?? value);
    this.applied.get(robot)?.set(name, v);
    this.targets.get(robot)?.set(name, v);
    const art = this.articulations.get(robot);
    if (art) {
      art.teleportToSceneGraph();
      art.setMotorTarget(name, v);
      return;
    }
    // Hard teleport, not a scheduled kinematic move: setNextKinematic* would
    // derive a velocity from the pose delta and sweep the link on the next
    // step, flinging any resting object. setTranslation/setRotation snap the
    // body's pose with no velocity — the collision-skipping reset contract.
    this.teleportLinkBodies();
  }

  /**
   * True when every joint command has been fully issued and, for dynamic
   * robots, every link body has come to rest (a blocked motor settles
   * against the obstacle — it never "reaches" the target).
   */
  settled(): boolean {
    for (const [robot, t] of this.targets) {
      const a = this.applied.get(robot)!;
      for (const [name, v] of t) if (a.get(name) !== v) return false;
    }
    for (const art of this.articulations.values()) if (!art.atRest()) return false;
    return true;
  }

  /** Snap kinematic bodies to their FK pose with no derived velocity. */
  private teleportLinkBodies(): void {
    for (const { link, body } of this.linkBodies) {
      link.getWorldPosition(_pos);
      link.getWorldQuaternion(_quat);
      body.setTranslation({ x: _pos.x, y: _pos.y, z: _pos.z }, false);
      body.setRotation({ x: _quat.x, y: _quat.y, z: _quat.z, w: _quat.w }, false);
    }
  }

  private driveLinkBodies(): void {
    for (const { link, body } of this.linkBodies) {
      link.getWorldPosition(_pos);
      link.getWorldQuaternion(_quat);
      body.setNextKinematicTranslation({ x: _pos.x, y: _pos.y, z: _pos.z });
      body.setNextKinematicRotation({ x: _quat.x, y: _quat.y, z: _quat.z, w: _quat.w });
    }
  }

  addGround(z = 0, halfExtent = 10): void {
    if (this.groundBody) return;
    const desc = RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, z - 0.05);
    this.groundBody = this.world.createRigidBody(desc);
    const colDesc = RAPIER.ColliderDesc.cuboid(halfExtent, halfExtent, 0.05);
    colDesc.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    const collider = this.world.createCollider(colDesc, this.groundBody);
    this.colliderNames.set(collider.handle, 'ground');
  }

  spawn(opts: SpawnOptions, scene: THREE.Scene): { name: string; mesh: THREE.Mesh } {
    const name = opts.name ?? `obj_${this.nextObjectId++}`;
    if (this.objects.has(name)) throw new Error(`Object name already in use: ${name}`);

    const [x, y, z] = opts.pos;
    const desc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z).setCcdEnabled(true);
    if (opts.quat) desc.setRotation(quatXYZW(opts.quat));
    const body = this.world.createRigidBody(desc);

    let colDesc: RAPIER_NS.ColliderDesc;
    let geometry: THREE.BufferGeometry;
    switch (opts.shape) {
      case 'sphere': {
        const [r = 0.05] = opts.size;
        colDesc = RAPIER.ColliderDesc.ball(r);
        geometry = new THREE.SphereGeometry(r, 32, 16);
        break;
      }
      case 'box': {
        const [sx = 0.1, sy = 0.1, sz = 0.1] = opts.size;
        colDesc = RAPIER.ColliderDesc.cuboid(sx / 2, sy / 2, sz / 2);
        geometry = new THREE.BoxGeometry(sx, sy, sz);
        break;
      }
      case 'cylinder': {
        const [r = 0.05, h = 0.1] = opts.size;
        // Rapier and three both build Y-axis cylinders; rotate both to Z-up.
        const rot = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0));
        colDesc = RAPIER.ColliderDesc.cylinder(h / 2, r).setRotation(quatXYZW([rot.x, rot.y, rot.z, rot.w]));
        geometry = new THREE.CylinderGeometry(r, r, h, 32).applyQuaternion(rot);
        break;
      }
      default:
        throw new Error(`Unknown shape: ${String(opts.shape)}`);
    }
    if (opts.mass !== undefined) colDesc.setMass(opts.mass);
    else colDesc.setDensity(opts.density ?? 500);
    colDesc.setFriction(opts.friction ?? 0.6);
    colDesc.setRestitution(opts.restitution ?? 0.1);
    colDesc.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    const collider = this.world.createCollider(colDesc, body);
    this.colliderNames.set(collider.handle, `obj:${name}`);

    const mesh = new THREE.Mesh(
      geometry,
      new THREE.MeshPhongMaterial({ color: opts.color ?? colorFor(name) }),
    );
    mesh.name = name;
    mesh.position.set(x, y, z);
    if (opts.quat) mesh.quaternion.set(...opts.quat);
    scene.add(mesh);

    this.objects.set(name, { body, mesh });
    return { name, mesh };
  }

  /**
   * Drag a dynamic object toward a world-space point (null releases it).
   * Applied as velocity control each substep: collisions keep resolving,
   * CCD keeps holding, and releasing mid-motion throws the object.
   */
  setDragTarget(name: string, point: [number, number, number] | null): void {
    if (!this.objects.has(name)) throw new Error(`No object named ${name}`);
    if (point) this.dragTargets.set(name, { x: point[0], y: point[1], z: point[2] });
    else this.dragTargets.delete(name);
  }

  /** Teleport a dynamic object (zeroes velocity; for paused-mode dragging). */
  setObjectPose(name: string, pos: [number, number, number], quat?: [number, number, number, number]): void {
    const entry = this.objects.get(name);
    if (!entry) throw new Error(`No object named ${name}`);
    entry.body.setTranslation({ x: pos[0], y: pos[1], z: pos[2] }, true);
    if (quat) entry.body.setRotation(quatXYZW(quat), true);
    entry.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    entry.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    entry.mesh.position.set(pos[0], pos[1], pos[2]);
    if (quat) entry.mesh.quaternion.set(...quat);
  }

  removeObject(name: string, scene: THREE.Scene): boolean {
    const entry = this.objects.get(name);
    if (!entry) return false;
    this.dragTargets.delete(name);
    for (let c = 0; c < entry.body.numColliders(); c++) {
      const handle = entry.body.collider(c).handle;
      this.purgeContactsFor(this.colliderNames.get(handle));
      this.colliderNames.delete(handle);
    }
    this.world.removeRigidBody(entry.body);
    scene.remove(entry.mesh);
    this.objects.delete(name);
    return true;
  }

  objectStates(): Record<string, ObjectState> {
    const out: Record<string, ObjectState> = {};
    for (const [name, { body }] of this.objects) {
      const t = body.translation();
      const r = body.rotation();
      out[name] = { pos: [t.x, t.y, t.z], quat: [r.x, r.y, r.z, r.w] };
    }
    return out;
  }

  /** Currently-touching collider pairs, as sorted name tuples. */
  contacts(): Array<[string, string]> {
    return [...this.activeContacts].map((k) => k.split(' ') as [string, string]);
  }

  /**
   * Link pairs of one robot currently touching (self-collision), as sorted
   * `[linkA, linkB]` tuples without the `link:robot/` prefix. Always empty
   * unless the world was built with `selfCollision: true`.
   */
  selfContacts(robotName: string): Array<[string, string]> {
    const prefix = `link:${robotName}/`;
    const out: Array<[string, string]> = [];
    for (const [a, b] of this.contacts()) {
      if (!a.startsWith(prefix) || !b.startsWith(prefix)) continue;
      out.push([a.slice(prefix.length), b.slice(prefix.length)]);
    }
    return out;
  }

  /**
   * Advance simulation. Accumulates into fixed-size substeps. Each substep
   * moves every joint toward its target by at most
   * min(URDF velocity limit × dt, maxJointDelta) — walls can never teleport
   * past objects regardless of how large a jump the caller requested.
   */
  step(dt: number): void {
    this.accumulator += dt;
    const nSub = Math.floor(this.accumulator / this.fixedDt + 1e-9);
    if (nSub <= 0) return;
    this.accumulator -= nSub * this.fixedDt;

    for (let k = 0; k < nSub; k++) {
      let anyJointMoved = false;
      for (const [robot, targetMap] of this.targets) {
        const appliedMap = this.applied.get(robot)!;
        const art = this.articulations.get(robot);
        let robotMoved = false;
        for (const [name, target] of targetMap) {
          const current = appliedMap.get(name) ?? 0;
          if (current === target) continue;
          const joint = robot.joints[name];
          const vel = Number(joint?.limit.velocity) || Infinity;
          const maxStep = Math.min(vel * this.fixedDt, this.maxJointDelta);
          const delta = Math.min(Math.max(target - current, -maxStep), maxStep);
          const next = Math.abs(target - current) <= maxStep ? target : current + delta;
          appliedMap.set(name, next);
          // Dynamic robots: the motor target is ramped the same way, so the
          // PD error (hence motor force) stays bounded — the stand-in for
          // PhysX's maxForce, which Rapier's JS API does not expose.
          if (art) art.setMotorTarget(name, next);
          else robot.setJointValue(name, next);
          robotMoved = true;
        }
        if (robotMoved) {
          if (!art) robot.updateMatrixWorld(true);
          anyJointMoved = true;
        }
      }
      this.driveLinkBodies();
      if (anyJointMoved) {
        // Sleep/wake around kinematic contacts has a history of edge cases.
        for (const { body } of this.objects.values()) body.wakeUp();
      }

      for (const [name, target] of this.dragTargets) {
        const entry = this.objects.get(name);
        if (!entry) {
          this.dragTargets.delete(name);
          continue;
        }
        const body = entry.body;
        body.wakeUp();
        const t = body.translation();
        const gain = 12; // 1/s toward the pointer
        const maxSpeed = 4; // m/s
        let vx = (target.x - t.x) * gain;
        let vy = (target.y - t.y) * gain;
        let vz = (target.z - t.z) * gain;
        const speed = Math.hypot(vx, vy, vz);
        if (speed > maxSpeed) {
          const k = maxSpeed / speed;
          vx *= k;
          vy *= k;
          vz *= k;
        }
        body.setLinvel({ x: vx, y: vy, z: vz }, true);
        const av = body.angvel();
        body.setAngvel({ x: av.x * 0.9, y: av.y * 0.9, z: av.z * 0.9 }, true);
      }

      this.world.step(this.eventQueue, this.hooks);

      this.eventQueue.drainCollisionEvents((h1, h2, started) => {
        const a = this.colliderNames.get(h1) ?? `?${h1}`;
        const b = this.colliderNames.get(h2) ?? `?${h2}`;
        const key = a < b ? `${a} ${b}` : `${b} ${a}`;
        if (started) this.activeContacts.add(key);
        else this.activeContacts.delete(key);
      });
    }

    // Dynamic articulations: pull joint values out of the bodies into the
    // scene graph (rendering/seg/getState read the kinematic tree).
    for (const art of this.articulations.values()) art.syncSceneGraph();

    // Sync dynamic bodies to their visual meshes.
    for (const { body, mesh } of this.objects.values()) {
      const t = body.translation();
      const r = body.rotation();
      mesh.position.set(t.x, t.y, t.z);
      mesh.quaternion.set(r.x, r.y, r.z, r.w);
    }
  }

  dispose(): void {
    this.world.free();
    this.eventQueue.free();
    this.linkBodies.length = 0;
    this.articulations.clear();
    this.objects.clear();
    this.colliderNames.clear();
    this.activeContacts.clear();
    this.adjacentBodies.clear();
  }
}

function pairKey(a: number, b: number): string {
  return a < b ? `${a}:${b}` : `${b}:${a}`;
}

/**
 * (parent, child) link pairs joined by a joint. urdf-loader nests each joint
 * under its parent link and the child link under the joint, so adjacency is
 * read straight off the scene graph — no URDF re-parse needed.
 */
function adjacentLinkPairs(robot: URDFRobot): Array<[URDFLink, URDFLink]> {
  type MaybeLink = THREE.Object3D & { isURDFLink?: boolean };
  const pairs: Array<[URDFLink, URDFLink]> = [];
  for (const joint of Object.values(robot.joints)) {
    const parent = joint.parent as MaybeLink | null;
    if (!parent?.isURDFLink) continue;
    for (const child of joint.children) {
      if ((child as MaybeLink).isURDFLink) {
        pairs.push([parent as unknown as URDFLink, child as unknown as URDFLink]);
      }
    }
  }
  return pairs;
}

function snapshotJointValues(robot: URDFRobot): Map<string, number> {
  const map = new Map<string, number>();
  for (const [name, joint] of Object.entries(robot.joints)) {
    if (joint.jointType === 'fixed') continue;
    map.set(name, Number(joint.jointValue[0] ?? 0));
  }
  return map;
}

function quatXYZW([x, y, z, w]: [number, number, number, number]): RAPIER_NS.Rotation {
  return { x, y, z, w };
}

/** Stable pleasant color from a name (for spawned object meshes). */
function colorFor(name: string): number {
  let h = 2166136261;
  for (let i = 0; i < name.length; i++) {
    h ^= name.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  const hue = (h >>> 0) % 360;
  return new THREE.Color().setHSL(hue / 360, 0.65, 0.55).getHex();
}
