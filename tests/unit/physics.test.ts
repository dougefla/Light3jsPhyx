import { describe, it, expect, beforeAll } from 'vitest';
import * as THREE from 'three';
import { initRapier, PhysicsWorld } from '@lightphyx/engine';
import type { URDFRobot } from 'urdf-loader';

beforeAll(async () => {
  await initRapier();
});

/**
 * Minimal stand-in for a URDFRobot: one prismatic "tray" link (a flat box)
 * sliding along +Y. Exercises the exact surface PhysicsWorld consumes.
 */
function makeTrayBot(z = 0.5): URDFRobot {
  const robot = new THREE.Group() as unknown as URDFRobot & THREE.Group;
  robot.name = 'bot';
  const link = new THREE.Group();
  link.position.z = z;
  robot.add(link);

  const colliderGroup = new THREE.Group() as THREE.Group & {
    isURDFCollider: boolean;
    isURDFVisual: boolean;
  };
  colliderGroup.isURDFCollider = true;
  colliderGroup.isURDFVisual = true; // serves both collisionFrom modes
  colliderGroup.add(new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.4, 0.02)));
  link.add(colliderGroup);

  const joint = {
    jointType: 'prismatic' as const,
    jointValue: [0] as number[],
    limit: { lower: -1, upper: 1, effort: 100, velocity: 1 },
  };
  (robot as { links?: unknown }).links = { tray: link };
  (robot as { joints?: unknown }).joints = { slide: joint };
  (robot as { setJointValue?: unknown }).setJointValue = (_name: string, v: number) => {
    if (joint.jointValue[0] === v) return false;
    joint.jointValue = [v];
    link.position.y = v;
    return true;
  };
  robot.updateMatrixWorld(true);
  return robot;
}

/**
 * Three-link chain laid out the way urdf-loader builds a scene graph
 * (link > joint > link), so adjacency can be read off parent/child:
 *   base ─j1─ arm ─j2─ tip
 * `arm` sits inside `base` (adjacent pair — must be filtered); `tip` straddles
 * base's +X face at j2 = 0 (non-adjacent pair — a genuine self-collision) and
 * slides out along +X as j2 grows.
 */
function makeChainBot(): URDFRobot {
  const robot = new THREE.Group() as unknown as URDFRobot & THREE.Group;
  robot.name = 'chain';
  const mkLink = (name: string, size: number) => {
    const link = new THREE.Group() as THREE.Group & { isURDFLink: boolean };
    link.name = name;
    link.isURDFLink = true;
    const vis = new THREE.Group() as THREE.Group & { isURDFVisual: boolean; isURDFCollider: boolean };
    vis.isURDFVisual = true;
    vis.isURDFCollider = true;
    vis.add(new THREE.Mesh(new THREE.BoxGeometry(size, size, size)));
    link.add(vis);
    return link;
  };
  const mkJoint = (name: string) => {
    const j = new THREE.Group() as THREE.Group & {
      isURDFJoint: boolean;
      jointType: 'prismatic';
      jointValue: number[];
      limit: { lower: number; upper: number; effort: number; velocity: number };
    };
    j.name = name;
    j.isURDFJoint = true;
    j.jointType = 'prismatic';
    j.jointValue = [0];
    j.limit = { lower: 0, upper: 2, effort: 100, velocity: 10 };
    return j;
  };
  const base = mkLink('base', 0.4);
  const arm = mkLink('arm', 0.1);
  const tip = mkLink('tip', 0.1);
  const j1 = mkJoint('j1');
  const j2 = mkJoint('j2');
  robot.add(base);
  base.add(j1);
  j1.add(arm); // arm centred inside base: overlapping but adjacent
  arm.add(j2);
  j2.position.x = 0.2; // tip centre on base's +X face: surfaces cross
  j2.add(tip);
  (robot as { links?: unknown }).links = { base, arm, tip };
  (robot as { joints?: unknown }).joints = { j1, j2 };
  (robot as { setJointValue?: unknown }).setJointValue = (name: string, v: number) => {
    const j = name === 'j1' ? j1 : j2;
    if (j.jointValue[0] === v) return false;
    j.jointValue = [v];
    j.position.x = (name === 'j2' ? 0.2 : 0) + v;
    return true;
  };
  robot.updateMatrixWorld(true);
  return robot;
}

function settleScenario(): { world: PhysicsWorld; scene: THREE.Scene; robot: URDFRobot } {
  const scene = new THREE.Scene();
  const world = new PhysicsWorld();
  const robot = makeTrayBot();
  world.addRobot(robot);
  world.addGround(0);
  world.spawn({ shape: 'sphere', size: [0.03], pos: [0, 0, 0.6], name: 'ball' }, scene);
  return { world, scene, robot };
}

describe('PhysicsWorld', () => {
  it('drops a ball to rest on the ground under gravity', () => {
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    world.addGround(0);
    world.spawn({ shape: 'sphere', size: [0.05], pos: [0.5, 0.5, 0.5], name: 'ball' }, scene);
    world.step(3);
    const z = world.objectStates()['ball']!.pos[2];
    expect(z).toBeGreaterThan(0.04);
    expect(z).toBeLessThan(0.06);
    expect(world.contacts()).toContainEqual(['ground', 'obj:ball']);
    world.dispose();
  });

  it('reports contact begin and end', () => {
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    world.addGround(0);
    world.spawn({ shape: 'box', size: [0.1, 0.1, 0.1], pos: [0, 0, 0.4], name: 'crate' }, scene);
    world.step(0.05);
    expect(world.contacts()).toHaveLength(0); // still falling
    world.step(2);
    expect(world.contacts()).toContainEqual(['ground', 'obj:crate']);
    world.dispose();
  });

  it('kinematic tray carries a resting crate when the joint moves (drawer semantics)', () => {
    // A crate (not a ball: balls physically roll and lag on flat trays —
    // real drawers carry balls via their walls, covered by the browser smoke).
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    const robot = makeTrayBot();
    world.addRobot(robot);
    world.addGround(0);
    world.spawn({ shape: 'box', size: [0.08, 0.08, 0.08], pos: [0, 0, 0.6], name: 'crate' }, scene);
    world.step(2); // settle onto the tray
    const settled = world.objectStates()['crate']!;
    expect(settled.pos[2]).toBeGreaterThan(0.5); // resting on tray, not ground

    world.setJointTarget(robot, 'slide', 0.25);
    world.step(1); // joint chases target under the velocity limit
    world.step(1); // settle

    const carried = world.objectStates()['crate']!;
    expect(carried.pos[1]).toBeGreaterThan(0.2); // static friction dragged it along
    expect(carried.pos[2]).toBeGreaterThan(0.5); // still on the tray
    world.dispose();
  });

  it('removing a resting object clears its contact (no phantom, issue #8)', () => {
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    world.addGround(0);
    world.spawn({ shape: 'sphere', size: [0.05], pos: [0, 0, 0.5], name: 'ball' }, scene);
    world.step(2);
    expect(world.contacts()).toContainEqual(['ground', 'obj:ball']);
    world.removeObject('ball', scene);
    world.step(0.1);
    expect(world.contacts().some((p) => p.includes('obj:ball'))).toBe(false);
    world.dispose();
  });

  it('immediate joint reset teleports and does not fling a resting object (issue #7)', () => {
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    const robot = makeTrayBot();
    world.addRobot(robot);
    world.spawn({ shape: 'box', size: [0.08, 0.08, 0.08], pos: [0, 0, 0.6], name: 'crate' }, scene);
    world.step(2); // crate rests on the tray at slide=0
    world.setJointTarget(robot, 'slide', 0.4);
    world.step(2); // tray (and crate) move out
    // Teleport the tray back: the crate must NOT be launched by a swept wall.
    world.setJointImmediate(robot, 'slide', 0.0);
    const before = world.objectStates()['crate']!.pos;
    world.step(0.2);
    const after = world.objectStates()['crate']!.pos;
    const speed = Math.hypot(after[0] - before[0], after[1] - before[1], after[2] - before[2]) / 0.2;
    expect(speed).toBeLessThan(2.0); // not flung; gravity-scale motion only
    world.dispose();
  });

  it('is deterministic: identical runs produce bit-identical states', () => {
    const run = () => {
      const { world, robot } = settleScenario();
      world.step(1.5);
      world.setJointTarget(robot, 'slide', -0.2);
      world.step(1.5);
      const state = world.objectStates()['ball']!;
      world.dispose();
      return state;
    };
    const a = run();
    const b = run();
    expect(a.pos).toEqual(b.pos);
    expect(a.quat).toEqual(b.quat);
    expect(Object.is(a.pos[0], b.pos[0])).toBe(true);
    expect(Object.is(a.pos[1], b.pos[1])).toBe(true);
    expect(Object.is(a.pos[2], b.pos[2])).toBe(true);
  });

  it('pointer-spring drag pulls an object to the target, gravity resumes on release', () => {
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    world.addGround(0);
    world.spawn({ shape: 'sphere', size: [0.05], pos: [0, 0, 0.05], name: 'ball' }, scene);
    world.step(0.5); // settle
    world.setDragTarget('ball', [0.3, 0.2, 0.5]);
    world.step(1.5);
    const held = world.objectStates()['ball']!.pos;
    expect(Math.hypot(held[0] - 0.3, held[1] - 0.2, held[2] - 0.5)).toBeLessThan(0.05);
    world.setDragTarget('ball', null);
    world.step(2);
    const dropped = world.objectStates()['ball']!.pos;
    expect(dropped[2]).toBeLessThan(0.08); // fell back to the ground
    world.dispose();
  });

  it('interpolates large joint jumps instead of teleporting through objects', () => {
    // The tray slab sweeps through a grounded ball at slab height: with
    // interpolation + CCD the ball must be pushed ahead, not tunneled through.
    const scene = new THREE.Scene();
    const world = new PhysicsWorld();
    const robot = makeTrayBot(0.05); // slab centered at the resting ball's center height
    world.addRobot(robot);
    world.addGround(0);
    world.spawn({ shape: 'sphere', size: [0.05], pos: [0, 0.3, 0.05], name: 'ball' }, scene);
    world.step(0.5);
    world.setJointTarget(robot, 'slide', 0.6);
    world.step(1);
    world.step(1);
    const ball = world.objectStates()['ball']!;
    // swept from y=0.3 outward (tray half-extent 0.2 + ball r 0.05 ≈ y>0.6)
    expect(ball.pos[1]).toBeGreaterThan(0.5);
    world.dispose();
  });
});

describe('PhysicsWorld self-collision', () => {
  const linkPairs = (world: PhysicsWorld) =>
    world.contacts().filter(([a, b]) => a.startsWith('link:') && b.startsWith('link:'));

  it('is off by default: overlapping links never appear in contacts()', () => {
    const world = new PhysicsWorld();
    world.addRobot(makeChainBot());
    world.step(0.5);
    expect(linkPairs(world)).toEqual([]);
    expect(world.selfContacts('chain')).toEqual([]);
    world.dispose();
  });

  it('reports non-adjacent link pairs and filters joint-adjacent ones', () => {
    const world = new PhysicsWorld({ selfCollision: true });
    const robot = makeChainBot();
    world.addRobot(robot);
    world.step(0.5);
    expect(world.contacts()).toContainEqual(['link:chain/base', 'link:chain/tip']);
    expect(world.selfContacts('chain')).toEqual([['base', 'tip']]);
    // arm is inside base but joined to it by j1 → never reported
    expect(linkPairs(world).some((p) => p.includes('link:chain/arm'))).toBe(false);
    world.dispose();
  });

  it('contact ends once the joint moves the link clear, and returns when it comes back', () => {
    const world = new PhysicsWorld({ selfCollision: true });
    const robot = makeChainBot();
    world.addRobot(robot);
    world.step(0.5);
    expect(world.selfContacts('chain')).toEqual([['base', 'tip']]);
    world.setJointTarget(robot, 'j2', 1.0); // tip slides 1 m out along +X
    world.step(1);
    world.step(0.5);
    expect(world.selfContacts('chain')).toEqual([]);
    world.setJointTarget(robot, 'j2', 0.0);
    world.step(1);
    world.step(0.5);
    expect(world.selfContacts('chain')).toEqual([['base', 'tip']]);
    world.dispose();
  });

  it('immediate (teleport) joint writes are reflected after the next step', () => {
    const world = new PhysicsWorld({ selfCollision: true });
    const robot = makeChainBot();
    world.addRobot(robot);
    world.step(0.5);
    world.setJointImmediate(robot, 'j2', 1.0);
    world.step(1 / 60);
    expect(world.selfContacts('chain')).toEqual([]);
    world.dispose();
  });

  it('removing the robot clears its self-contacts', () => {
    const world = new PhysicsWorld({ selfCollision: true });
    const robot = makeChainBot();
    world.addRobot(robot);
    world.step(0.5);
    expect(world.selfContacts('chain')).toHaveLength(1);
    world.removeRobot(robot);
    world.step(0.1);
    expect(world.selfContacts('chain')).toEqual([]);
    expect(linkPairs(world)).toEqual([]);
    world.dispose();
  });

  it('reports links of two different robots touching', () => {
    const world = new PhysicsWorld({ selfCollision: true });
    const a = makeTrayBot(0.5);
    const b = makeTrayBot(0.51); // 2 cm slab at z=0.51 crosses the slab at z=0.5
    b.name = 'bot2';
    world.addRobot(a);
    world.addRobot(b);
    world.step(0.5);
    expect(world.contacts()).toContainEqual(['link:bot/tray', 'link:bot2/tray']);
    expect(world.selfContacts('bot')).toEqual([]); // cross-robot ≠ self
    world.dispose();
  });

  it('dynamic objects still rest on kinematic links with self-collision on (hooks path)', () => {
    const scene = new THREE.Scene();
    const world = new PhysicsWorld({ selfCollision: true });
    const robot = makeTrayBot();
    world.addRobot(robot);
    world.addGround(0);
    world.spawn({ shape: 'box', size: [0.08, 0.08, 0.08], pos: [0, 0, 0.6], name: 'crate' }, scene);
    world.step(2);
    expect(world.objectStates()['crate']!.pos[2]).toBeGreaterThan(0.5);
    expect(world.contacts()).toContainEqual(['link:bot/tray', 'obj:crate']);
    world.dispose();
  });
});
