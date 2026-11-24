import { describe, it, expect, beforeAll } from 'vitest';
import * as THREE from 'three';
import { initRapier, PhysicsWorld } from '@lightphyx/engine';
import type { URDFRobot } from 'urdf-loader';

beforeAll(async () => {
  await initRapier();
});

type Flags = { isURDFLink?: boolean; isURDFJoint?: boolean; isURDFVisual?: boolean; isURDFCollider?: boolean };

function link(name: string, box: [number, number, number], center: [number, number, number]) {
  const l = new THREE.Group() as THREE.Group & Flags;
  l.name = name;
  l.isURDFLink = true;
  const vis = new THREE.Group() as THREE.Group & Flags;
  vis.isURDFVisual = true;
  vis.isURDFCollider = true;
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...box));
  mesh.position.set(...center);
  vis.add(mesh);
  l.add(vis);
  return l;
}

function joint(
  name: string,
  type: 'revolute' | 'prismatic',
  origin: [number, number, number],
  axis: [number, number, number],
  limits: [number, number],
) {
  const j = new THREE.Group() as THREE.Group &
    Flags & {
      jointType: string;
      jointValue: number[];
      axis: THREE.Vector3;
      limit: { lower: number; upper: number; effort: number; velocity: number };
      ignoreLimits: boolean;
      origPos: THREE.Vector3;
    };
  j.name = name;
  j.isURDFJoint = true;
  j.jointType = type;
  j.jointValue = [0];
  j.axis = new THREE.Vector3(...axis);
  j.limit = { lower: limits[0], upper: limits[1], effort: 100, velocity: 1 };
  j.ignoreLimits = false;
  j.position.set(...origin);
  j.origPos = j.position.clone();
  return j;
}

/**
 * Fridge stand-in (URDF frame: Z up, front = −Y):
 *   base   fixed 0.5×0.4×0.7 box, front face at y = −0.2
 *   door   revolute about −Z at the front-left edge; a 0.5×0.02×0.7 leaf
 *          covering the front (y ≈ −0.21). value ≥ 0 swings it outward.
 *   drawer prismatic along +Y inside the base, 0.3×0.3×0.15; value −0.3
 *          slides it out through the door's plane.
 * Mirrors urdf-loader's scene-graph nesting: link > joint > link, with
 * setJointValue updating the joint transform like URDFJoint does.
 */
function makeFridge(): URDFRobot {
  const robot = new THREE.Group() as unknown as URDFRobot & THREE.Group;
  robot.name = 'fridge';
  const base = link('base', [0.5, 0.4, 0.7], [0, 0, 0.35]);
  const door = link('door', [0.5, 0.02, 0.7], [0.25, 0, 0]);
  const drawer = link('drawer', [0.3, 0.3, 0.15], [0, 0, 0]);
  const hinge = joint('hinge', 'revolute', [-0.25, -0.21, 0.35], [0, 0, -1], [0, 1.57]);
  const slide = joint('slide', 'prismatic', [0, 0, 0.35], [0, 1, 0], [-0.3, 0]);
  robot.add(base);
  base.add(hinge);
  hinge.add(door);
  base.add(slide);
  slide.add(drawer);
  const joints = { hinge, slide };
  (robot as { links?: unknown }).links = { base, door, drawer };
  (robot as { joints?: unknown }).joints = joints;
  (robot as { setJointValue?: unknown }).setJointValue = (name: string, v: number) => {
    const j = joints[name as keyof typeof joints];
    if (!j) return false;
    v = Math.min(Math.max(v, j.limit.lower), j.limit.upper);
    if (j.jointValue[0] === v) return false;
    j.jointValue = [v];
    if (j.jointType === 'revolute') j.quaternion.setFromAxisAngle(j.axis, v);
    else j.position.copy(j.origPos).addScaledVector(j.axis, v);
    return true;
  };
  robot.updateMatrixWorld(true);
  return robot;
}

const q = (robot: URDFRobot, name: string) => Number(robot.joints[name]!.jointValue[0]);

describe('dynamic articulation', () => {
  it('motored joints track set targets without obstacles (axis + sign conventions)', () => {
    const world = new PhysicsWorld();
    const robot = makeFridge();
    world.addRobot(robot, { dynamic: true });
    expect(world.isDynamic(robot)).toBe(true);
    world.setJointTarget(robot, 'hinge', 1.0); // open the door first
    world.step(3);
    expect(q(robot, 'hinge')).toBeCloseTo(1.0, 1);
    world.setJointTarget(robot, 'slide', -0.3);
    world.step(3);
    expect(q(robot, 'slide')).toBeCloseTo(-0.3, 1);
    expect(world.settled()).toBe(true);
    world.dispose();
  });

  it('a motored drawer pushes a passive door open (contacts propagate through joints)', () => {
    const world = new PhysicsWorld();
    const robot = makeFridge();
    world.addRobot(robot, { dynamic: true });
    world.setJointDrive(robot, 'hinge', { stiffness: 0, damping: 2 }); // door swings freely
    world.step(0.5);
    expect(q(robot, 'hinge')).toBeCloseTo(0, 2); // closed, nothing touching it yet

    world.setJointTarget(robot, 'slide', -0.3);
    let touched = false;
    let maxHinge = 0;
    for (let i = 0; i < 180; i++) {
      world.step(1 / 60);
      if (world.selfContacts('fridge').some(([a, b]) => a === 'door' && b === 'drawer')) touched = true;
      maxHinge = Math.max(maxHinge, q(robot, 'hinge'));
    }
    expect(touched).toBe(true);
    expect(q(robot, 'slide')).toBeLessThan(-0.2); // drawer got most of the way out
    expect(maxHinge).toBeGreaterThan(0.3); // door was shoved open
    world.dispose();
  });

  it('a drawer driven into a held door stalls instead of tunnelling', () => {
    const world = new PhysicsWorld();
    const robot = makeFridge();
    world.addRobot(robot, { dynamic: true });
    // Hold the door shut with a stiff spring (acceleration model: 1/s²). The
    // drawer's default drive (400) pushes with ~k·|lag|·m and loses.
    world.setJointDrive(robot, 'hinge', { stiffness: 1e5, damping: 1e3 });
    world.setJointTarget(robot, 'slide', -0.3);
    world.step(4);
    expect(q(robot, 'hinge')).toBeLessThan(0.15); // door essentially closed
    expect(q(robot, 'slide')).toBeGreaterThan(-0.2); // drawer stopped at the door
    expect(q(robot, 'slide')).toBeLessThan(-0.02); // ...but did move until it hit it
    world.dispose();
  });

  it('immediate joint writes teleport the bodies and read back exactly', () => {
    const world = new PhysicsWorld();
    const robot = makeFridge();
    world.addRobot(robot, { dynamic: true });
    world.setJointImmediate(robot, 'hinge', 0.8);
    world.setJointImmediate(robot, 'slide', -0.1);
    world.step(1 / 60);
    expect(q(robot, 'hinge')).toBeCloseTo(0.8, 2);
    expect(q(robot, 'slide')).toBeCloseTo(-0.1, 2);
    world.dispose();
  });

  it('a spawned box rides along inside a dynamic drawer', () => {
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    const robot = makeFridge();
    world.addRobot(robot, { dynamic: true });
    world.setJointImmediate(robot, 'hinge', 1.5); // door out of the way
    world.spawn({ shape: 'box', size: [0.06, 0.06, 0.06], pos: [0, 0, 0.5], name: 'cube' }, scene);
    world.step(1.5); // lands on the drawer top (z = 0.425)
    expect(world.objectStates()['cube']!.pos[2]).toBeGreaterThan(0.44);
    world.setJointTarget(robot, 'slide', -0.3);
    world.step(3);
    expect(world.objectStates()['cube']!.pos[1]).toBeLessThan(-0.15); // carried out with the drawer
    world.dispose();
  });

  it('removeRobot drops bodies, joints and link contacts', () => {
    const world = new PhysicsWorld();
    const robot = makeFridge();
    world.addRobot(robot, { dynamic: true });
    world.setJointDrive(robot, 'hinge', { stiffness: 0 });
    world.setJointTarget(robot, 'slide', -0.3);
    world.step(1.5);
    world.removeRobot(robot);
    world.step(0.1);
    expect(world.contacts().some(([a, b]) => a.startsWith('link:') || b.startsWith('link:'))).toBe(false);
    expect(world.isDynamic(robot)).toBe(false);
    world.dispose();
  });

  it('is deterministic', () => {
    const run = () => {
      const world = new PhysicsWorld();
      const robot = makeFridge();
      world.addRobot(robot, { dynamic: true });
      world.setJointDrive(robot, 'hinge', { stiffness: 0, damping: 2 });
      world.setJointTarget(robot, 'slide', -0.3);
      world.step(2);
      const out = [q(robot, 'hinge'), q(robot, 'slide')];
      world.dispose();
      return out;
    };
    expect(run()).toEqual(run());
  });
});
