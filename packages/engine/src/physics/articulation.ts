import * as THREE from 'three';
import type RAPIER_NS from '@dimforge/rapier3d-compat';
import { RAPIER } from './rapier.js';
import type { URDFRobot, URDFLink, URDFJoint } from 'urdf-loader';

type RigidBody = RAPIER_NS.RigidBody;
type UnitJoint = RAPIER_NS.UnitImpulseJoint;

export interface DriveOptions {
  /**
   * Motor stiffness (PD "P" gain). With the default acceleration-based motor
   * model this is in 1/s²: 400 ≈ a 20 rad/s spring. 0 makes the joint passive
   * (a door that swings when pushed). Default 400.
   */
  stiffness?: number;
  /** Motor damping ("D" gain), 1/s under the acceleration model. Default 40. */
  damping?: number;
}

export interface ArticulationOptions extends DriveOptions {
  /**
   * Density (kg/m³) used to estimate each link body's mass from the bounding
   * box of its colliders. URDF <inertial> blocks are ignored: datasets ship
   * them empty or inconsistent, and a hollow trimesh has no volume of its
   * own. Default 300 (particle board).
   */
  density?: number;
  /** 'force' uses MotorModel.ForceBased (gains in N·m, N). Default 'acceleration'. */
  motorModel?: 'acceleration' | 'force';
  /** Which URDF geometry becomes the colliders (see PhysicsOptions.collisionFrom). */
  collisionFrom: 'visual' | 'collision';
}

interface JointEntry {
  name: string;
  joint: UnitJoint;
  urdf: URDFJoint;
  parent: RigidBody;
  child: RigidBody;
  /** Joint axis in the parent body's local frame (constant). */
  axis: THREE.Vector3;
  kind: 'revolute' | 'prismatic';
  /** URDF joint value at setup — Rapier's joint coordinate is zero there. */
  q0: number;
  /** Prismatic: axis-projected child offset at setup. */
  d0: number;
  /** Last read value, for unwrapping continuous joints. */
  prev: number;
  stiffness: number;
  damping: number;
}

interface BodyEntry {
  body: RigidBody;
  /** Link whose frame the body is rigidly attached to. */
  link: URDFLink;
  /** body_world = link.matrixWorld × linkToBody (constant). */
  linkToBody: THREE.Matrix4;
}

const _pos = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _quat2 = new THREE.Quaternion();
const _mat = new THREE.Matrix4();
const _v = new THREE.Vector3();

/**
 * One URDF robot as a chain of DYNAMIC Rapier bodies joined by impulse
 * joints with position motors — the Isaac/PhysX articulation model, in
 * maximal coordinates. Contacts and joints are solved together, so a motored
 * drawer sliding into a passive door pushes it open, and a blocked joint
 * stalls instead of tunnelling.
 *
 * Body layout: links joined by fixed joints are merged into one body; the
 * root link is a fixed body (furniture stands still). Every body frame has
 * identity orientation at setup, so one world-space joint axis serves both
 * Rapier anchor frames and joint readback is a plain relative-pose problem.
 * Joint values are read back from body poses after each step and written
 * into the three.js scene graph, so rendering/seg/getState keep working on
 * the ideal kinematic tree.
 */
export class Articulation {
  readonly robot: URDFRobot;
  private readonly world: RAPIER_NS.World;
  private readonly bodies: BodyEntry[] = [];
  private readonly joints = new Map<string, JointEntry>();
  private readonly defaultStiffness: number;
  private readonly defaultDamping: number;
  private readonly motorModel: RAPIER_NS.MotorModel;
  /** Rapier body handle pairs ("lo:hi") joined by a joint, for contact filtering. */
  readonly adjacentPairs: string[] = [];
  readonly colliderHandles: number[] = [];

  constructor(
    world: RAPIER_NS.World,
    robot: URDFRobot,
    opts: ArticulationOptions,
    nameCollider: (handle: number, linkName: string) => void,
  ) {
    this.world = world;
    this.robot = robot;
    this.defaultStiffness = opts.stiffness ?? 400;
    this.defaultDamping = opts.damping ?? 40;
    this.motorModel =
      opts.motorModel === 'force' ? RAPIER.MotorModel.ForceBased : RAPIER.MotorModel.AccelerationBased;
    const density = opts.density ?? 300;
    robot.updateMatrixWorld(true);

    // ---- group links into bodies ------------------------------------------
    const groupRoot = new Map<URDFLink, URDFLink>();
    const rootOf = (link: URDFLink): URDFLink => {
      const cached = groupRoot.get(link);
      if (cached) return cached;
      const joint = link.parent as (THREE.Object3D & { isURDFJoint?: boolean }) | null;
      let root = link;
      if (joint?.isURDFJoint && !isMovable(joint as unknown as URDFJoint)) {
        const parentLink = joint.parent as (THREE.Object3D & { isURDFLink?: boolean }) | null;
        if (parentLink?.isURDFLink) root = rootOf(parentLink as unknown as URDFLink);
      }
      groupRoot.set(link, root);
      return root;
    };
    const groups = new Map<URDFLink, URDFLink[]>();
    for (const link of Object.values(robot.links)) {
      const root = rootOf(link);
      const list = groups.get(root) ?? [];
      list.push(link);
      groups.set(root, list);
    }

    const bodyOfRoot = new Map<URDFLink, RigidBody>();
    const flag = opts.collisionFrom === 'visual' ? 'isURDFVisual' : 'isURDFCollider';
    for (const [root, links] of groups) {
      const isBase = !(root.parent as { isURDFJoint?: boolean } | null)?.isURDFJoint;
      root.getWorldPosition(_pos);
      const desc = (isBase ? RAPIER.RigidBodyDesc.fixed() : RAPIER.RigidBodyDesc.dynamic())
        .setTranslation(_pos.x, _pos.y, _pos.z)
        .setLinearDamping(0.5)
        .setAngularDamping(0.5);
      const body = world.createRigidBody(desc);
      const bodyWorld = new THREE.Matrix4().makeTranslation(_pos.x, _pos.y, _pos.z);
      const bodyInverse = bodyWorld.clone().invert();
      const aabb = new THREE.Box3();
      let colliders = 0;
      for (const link of links) {
        const linkName = linkNameOf(robot, link);
        for (const child of link.children) {
          if (!(child as unknown as Record<string, unknown>)[flag]) continue;
          child.traverse((node) => {
            const mesh = node as THREE.Mesh;
            if (!mesh.isMesh) return;
            const { vertices, indices } = extractTrimesh(mesh, bodyInverse);
            if (indices.length === 0) return;
            for (let i = 0; i < vertices.length; i += 3) {
              aabb.expandByPoint(_v.set(vertices[i]!, vertices[i + 1]!, vertices[i + 2]!));
            }
            const colDesc = RAPIER.ColliderDesc.trimesh(vertices, indices, RAPIER.TriMeshFlags.FIX_INTERNAL_EDGES)
              .setDensity(0)
              .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS)
              .setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS);
            const collider = world.createCollider(colDesc, body);
            nameCollider(collider.handle, linkName);
            this.colliderHandles.push(collider.handle);
            colliders++;
          });
        }
      }
      if (!isBase) {
        // Box mass properties from the collider AABB (see ArticulationOptions.density).
        const size = colliders > 0 ? aabb.getSize(new THREE.Vector3()) : new THREE.Vector3(0.1, 0.1, 0.1);
        const center = colliders > 0 ? aabb.getCenter(new THREE.Vector3()) : new THREE.Vector3();
        const mass = Math.max(density * size.x * size.y * size.z, 1e-3);
        const k = mass / 12;
        body.setAdditionalMassProperties(
          mass,
          { x: center.x, y: center.y, z: center.z },
          {
            x: k * (size.y * size.y + size.z * size.z),
            y: k * (size.x * size.x + size.z * size.z),
            z: k * (size.x * size.x + size.y * size.y),
          },
          { x: 0, y: 0, z: 0, w: 1 },
          true,
        );
      }
      bodyOfRoot.set(root, body);
      this.bodies.push({
        body,
        link: root,
        linkToBody: _mat.copy(root.matrixWorld).invert().multiply(bodyWorld).clone(),
      });
    }

    // ---- joints ---------------------------------------------------------------
    for (const [name, urdf] of Object.entries(robot.joints)) {
      if (!isMovable(urdf)) continue;
      const parentLink = urdf.parent as (THREE.Object3D & { isURDFLink?: boolean }) | null;
      const childLink = urdf.children.find((c) => (c as { isURDFLink?: boolean }).isURDFLink);
      if (!parentLink?.isURDFLink || !childLink) continue;
      const parent = bodyOfRoot.get(rootOf(parentLink as unknown as URDFLink));
      const child = bodyOfRoot.get(rootOf(childLink as unknown as URDFLink));
      if (!parent || !child || parent === child) continue;

      urdf.getWorldPosition(_pos);
      urdf.getWorldQuaternion(_quat);
      const axis = urdf.axis.clone().normalize().applyQuaternion(_quat);
      const p = parent.translation();
      const c = child.translation();
      const anchor1 = { x: _pos.x - p.x, y: _pos.y - p.y, z: _pos.z - p.z };
      const anchor2 = { x: _pos.x - c.x, y: _pos.y - c.y, z: _pos.z - c.z };
      const kind = urdf.jointType === 'prismatic' ? 'prismatic' : 'revolute';
      const data =
        kind === 'prismatic'
          ? RAPIER.JointData.prismatic(anchor1, anchor2, axis)
          : RAPIER.JointData.revolute(anchor1, anchor2, axis);
      const q0 = Number(urdf.jointValue[0] ?? 0);
      if (urdf.jointType !== 'continuous' && !urdf.ignoreLimits) {
        data.limitsEnabled = true;
        data.limits = [Number(urdf.limit.lower) - q0, Number(urdf.limit.upper) - q0];
      }
      const joint = world.createImpulseJoint(data, parent, child, true) as UnitJoint;
      joint.configureMotorModel(this.motorModel);
      joint.configureMotorPosition(0, this.defaultStiffness, this.defaultDamping);
      const d0 = _v.set(c.x - p.x, c.y - p.y, c.z - p.z).dot(axis);
      this.joints.set(name, {
        name,
        joint,
        urdf,
        parent,
        child,
        axis,
        kind,
        q0,
        d0,
        prev: q0,
        stiffness: this.defaultStiffness,
        damping: this.defaultDamping,
      });
      const a = parent.handle;
      const b = child.handle;
      this.adjacentPairs.push(a < b ? `${a}:${b}` : `${b}:${a}`);
    }
  }

  hasJoint(name: string): boolean {
    return this.joints.has(name);
  }

  /** Command the motor toward a URDF joint value (caller clamps to limits). */
  setMotorTarget(name: string, value: number): void {
    const j = this.joints.get(name);
    if (!j) return;
    j.joint.configureMotorPosition(value - j.q0, j.stiffness, j.damping);
    j.parent.wakeUp();
    j.child.wakeUp();
  }

  /** Change PD gains of one joint; the current target is kept. */
  setDrive(name: string, opts: DriveOptions, currentTarget: number): void {
    const j = this.joints.get(name);
    if (!j) throw new Error(`No joint named ${name}`);
    if (opts.stiffness !== undefined) j.stiffness = Math.max(0, opts.stiffness);
    if (opts.damping !== undefined) j.damping = Math.max(0, opts.damping);
    j.joint.configureMotorPosition(currentTarget - j.q0, j.stiffness, j.damping);
    j.child.wakeUp();
  }

  /** Current URDF joint values derived from body poses. */
  readJointValues(): Map<string, number> {
    const out = new Map<string, number>();
    for (const j of this.joints.values()) {
      const pr = j.parent.rotation();
      const cr = j.child.rotation();
      _quat.set(pr.x, pr.y, pr.z, pr.w).invert();
      let value: number;
      if (j.kind === 'revolute') {
        _quat2.set(cr.x, cr.y, cr.z, cr.w);
        _quat2.premultiply(_quat); // parent⁻¹ · child
        // Signed angle about the (constrained) axis.
        const dot = _quat2.x * j.axis.x + _quat2.y * j.axis.y + _quat2.z * j.axis.z;
        let angle = 2 * Math.atan2(dot, _quat2.w);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        value = j.q0 + angle;
        if (j.urdf.jointType === 'continuous') {
          // unwrap toward the previous reading
          while (value - j.prev > Math.PI) value -= 2 * Math.PI;
          while (value - j.prev < -Math.PI) value += 2 * Math.PI;
        }
      } else {
        const p = j.parent.translation();
        const c = j.child.translation();
        _v.set(c.x - p.x, c.y - p.y, c.z - p.z).applyQuaternion(_quat);
        value = j.q0 + _v.dot(j.axis) - j.d0;
      }
      j.prev = value;
      out.set(j.name, value);
    }
    return out;
  }

  /** Write body-derived joint values into the scene graph. */
  syncSceneGraph(): void {
    for (const [name, value] of this.readJointValues()) this.robot.setJointValue(name, value);
    this.robot.updateMatrixWorld(true);
  }

  /** Snap every body to the scene graph's FK pose with zero velocity. */
  teleportToSceneGraph(): void {
    this.robot.updateMatrixWorld(true);
    for (const { body, link, linkToBody } of this.bodies) {
      _mat.multiplyMatrices(link.matrixWorld, linkToBody);
      _pos.setFromMatrixPosition(_mat);
      _quat.setFromRotationMatrix(_mat);
      body.setTranslation({ x: _pos.x, y: _pos.y, z: _pos.z }, false);
      body.setRotation({ x: _quat.x, y: _quat.y, z: _quat.z, w: _quat.w }, false);
      if (body.isDynamic()) {
        body.setLinvel({ x: 0, y: 0, z: 0 }, false);
        body.setAngvel({ x: 0, y: 0, z: 0 }, false);
      }
    }
    for (const j of this.joints.values()) j.prev = Number(j.urdf.jointValue[0] ?? j.q0);
  }

  /** True when every dynamic body is (nearly) motionless. */
  atRest(linTol = 5e-3, angTol = 1e-2): boolean {
    for (const { body } of this.bodies) {
      if (!body.isDynamic()) continue;
      if (body.isSleeping()) continue;
      const v = body.linvel();
      const w = body.angvel();
      if (Math.hypot(v.x, v.y, v.z) > linTol) return false;
      if (Math.hypot(w.x, w.y, w.z) > angTol) return false;
    }
    return true;
  }

  wakeUp(): void {
    for (const { body } of this.bodies) body.wakeUp();
  }

  dispose(): void {
    for (const j of this.joints.values()) this.world.removeImpulseJoint(j.joint, true);
    this.joints.clear();
    for (const { body } of this.bodies) this.world.removeRigidBody(body);
    this.bodies.length = 0;
  }
}

function isMovable(joint: URDFJoint): boolean {
  const t = joint.jointType;
  return t === 'revolute' || t === 'continuous' || t === 'prismatic';
}

function linkNameOf(robot: URDFRobot, link: URDFLink): string {
  for (const [name, l] of Object.entries(robot.links)) if (l === link) return name;
  return link.name;
}

/** Mesh vertices transformed into the given frame (frameInverse × mesh.matrixWorld). */
export function extractTrimesh(
  mesh: THREE.Mesh,
  frameInverse: THREE.Matrix4,
): { vertices: Float32Array; indices: Uint32Array } {
  const local = new THREE.Matrix4().multiplyMatrices(frameInverse, mesh.matrixWorld);
  const posAttr = mesh.geometry.getAttribute('position');
  if (!posAttr) return { vertices: new Float32Array(0), indices: new Uint32Array(0) };
  const vertices = new Float32Array(posAttr.count * 3);
  const v = new THREE.Vector3();
  for (let i = 0; i < posAttr.count; i++) {
    v.fromBufferAttribute(posAttr, i).applyMatrix4(local);
    vertices[i * 3] = v.x;
    vertices[i * 3 + 1] = v.y;
    vertices[i * 3 + 2] = v.z;
  }
  const index = mesh.geometry.getIndex();
  const indices = index ? new Uint32Array(index.array) : new Uint32Array([...Array(posAttr.count).keys()]);
  return { vertices, indices };
}
