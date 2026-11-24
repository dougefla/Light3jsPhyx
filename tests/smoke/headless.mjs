/**
 * Headless end-to-end smoke: server + pool + capture pipeline.
 * Asserts: seg ids == links(+objects), depth range sane, seg/depth pixel
 * alignment, mp4 produced, cross-env isolation.
 *
 * Usage: node tests/smoke/headless.mjs  (URDF_PATH env var to override)
 */
import { SimServer } from '../../packages/server/dist/server.js';
import { statSync, readFileSync } from 'node:fs';
import path from 'node:path';
import os from 'node:os';

const urdf = process.env.URDF_PATH ?? new URL('../assets/mini_cabinet.urdf', import.meta.url).pathname;
const outDir = path.join(os.tmpdir(), `l3p-smoke-${process.pid}`);

const server = new SimServer({ port: 0, pool: { browsers: 1, pagesPerBrowser: 1, envsPerPage: 2 }, outDir });
await server.start();

const failures = [];
const check = (cond, msg) => {
  console.log(`${cond ? 'ok  ' : 'FAIL'} ${msg}`);
  if (!cond) failures.push(msg);
};

try {
  const { envs } = await server.call('create_envs', { count: 2 });
  check(envs.length === 2, 'created 2 envs on one page');
  const [a, b] = envs;

  const loaded = await server.call('load_urdf', { env: a, path: urdf });
  await server.call('load_urdf', { env: b, path: urdf });
  check(loaded.joints.length >= 1, `env ${a}: ${loaded.joints.length} movable joints`);
  const J = loaded.joints[0].name;
  const jlim = loaded.joints[0];
  const far = Math.abs(jlim.lower) > Math.abs(jlim.upper) ? jlim.lower : jlim.upper;

  await server.call('add_ground', { env: [a, b], visual: false });
  await server.call('set_camera', { env: [a, b], width: 640, height: 480 });
  await server.call('frame_camera', { env: [a, b] });

  // Cross-env isolation: open drawers only in env a.
  await server.call('set_joint', { env: a, name: J, value: far });
  await server.call('step', { env: 'all', dt: 1 / 30, n: 60 });
  const stateA = await server.call('get_state', { env: a });
  const stateB = await server.call('get_state', { env: b });
  check(Math.abs(stateA.joints[J] - far) < 1e-6, 'env a drawer open');
  check(stateB.joints[J] === 0, 'env b untouched (isolation)');

  // Spawn an object only in env b; seg must include it only there.
  await server.call('spawn', { env: b, shape: 'sphere', size: [0.04], pos: [0, 0, 1.5], name: 'marble' });
  await server.call('step', { env: b, dt: 1 / 30, n: 30 });

  const passesA = await collectRender(a, { save: { dir: outDir, prefix: 'a' } });
  const passesB = await collectRender(b, {});

  const idsA = uniqueIds(passesA.seg, 640, 480);
  const idsB = uniqueIds(passesB.seg, 640, 480);
  const nLinks = Object.values(passesA.labels).filter((l) => !l.startsWith('obj:') && l !== 'ground').length;
  check(idsA.size >= Math.min(nLinks, 2) && !labelHas(passesA.labels, 'obj:marble'), `env a: ${idsA.size} ids (${nLinks} links), no marble`);
  check(labelHas(passesB.labels, 'obj:marble'), 'env b labels include marble');
  check([...idsB].some((id) => passesB.labels[id] === 'obj:marble'), 'env b seg pixels include marble');

  // Depth: nonzero foreground within sane metric range, aligned with seg.
  // Payload sits at byte offset 5 of the ws frame — copy to align for Float32.
  const depthBuf = passesA.depth;
  const depth = new Float32Array(
    depthBuf.buffer.slice(depthBuf.byteOffset, depthBuf.byteOffset + 640 * 480 * 4),
  );
  const segIds = decodeIds(passesA.seg, 640, 480);
  let misaligned = 0;
  let fg = 0;
  for (let i = 0; i < depth.length; i++) {
    const hasDepth = depth[i] > 0;
    const hasId = segIds[i] !== 0;
    if (hasDepth !== hasId) misaligned++;
    if (hasId) fg++;
  }
  check(fg > 1000, `env a: ${fg} foreground pixels`);
  check(misaligned === 0, `seg/depth pixel alignment (${misaligned} mismatches)`);

  // Recording: short clip while closing the drawer.
  const recPath = path.join(outDir, 'smoke.mp4');
  await server.call('start_record', { env: a, fps: 30, path: recPath });
  await server.call('set_joint', { env: a, name: J, value: 0 });
  await server.call('step', { env: a, dt: 1 / 30, n: 30 });
  const rec = await server.call('stop_record', { env: a });
  check(rec.frames >= 30, `recorded ${rec.frames} frames`);
  check(rec.path.endsWith('.mp4') && statSync(rec.path).size > 2_000, `mp4 exists (${statSync(rec.path).size} bytes)`);

  // Determinism across identical envs: fresh env pair, same commands, same state.
  const r1 = await deterministicRun();
  const r2 = await deterministicRun();
  check(JSON.stringify(r1) === JSON.stringify(r2), 'bit-identical repeated run');

  // Issue #8: destroying envs must release page capacity for reuse.
  const statsBefore = await server.call('pool_stats');
  await server.call('destroy_envs', { env: 'all' });
  await server.call('create_envs', { count: 2 });
  const statsAfter = await server.call('pool_stats');
  check(
    statsAfter.pages.length <= statsBefore.pages.length,
    `pool reuses pages after destroy (${statsBefore.pages.length} -> ${statsAfter.pages.length})`,
  );
  check(statsAfter.envs === 2, `env registry consistent (${statsAfter.envs} envs)`);

  // Outline pass: rgb changes, but seg is BIT-IDENTICAL and depth unchanged
  // (outlines must never contaminate the label / depth channels).
  const { envs: oe } = await server.call('create_envs', { count: 1 });
  const oenv = oe[0];
  await server.call('load_urdf', { env: oenv, path: urdf });
  await server.call('add_ground', { env: oenv, visual: false });
  await server.call('frame_camera', { env: oenv });
  const plain = await collectRenderOn(oenv, false);
  const outlined = await collectRenderOn(oenv, true);
  check(!bufEqual(plain.rgb, outlined.rgb), 'outline changes the rgb pass');
  check(bufEqual(plain.seg, outlined.seg), 'seg is bit-identical with outline on/off');
  check(bufEqual(plain.depth, outlined.depth), 'depth is bit-identical with outline on/off');
} finally {
  await server.dispose();
}

console.log(failures.length ? `\n${failures.length} FAILURES` : '\nHEADLESS SMOKE PASS');
process.exit(failures.length ? 1 : 0);

async function collectRender(env, extra) {
  const bins = {};
  const result = await server.call(
    'render',
    { env, passes: ['rgb', 'seg', 'depth'], ...extra },
    (channel, payload) => {
      bins[channel] = payload;
    },
  );
  return { rgb: bins[1], seg: bins[2], depth: bins[3], labels: result.seg_labels ?? {} };
}

async function collectRenderOn(env, outline) {
  const bins = {};
  await server.call('render', { env, passes: ['rgb', 'seg', 'depth'], outline }, (ch, p) => {
    // copy: the same buffer slot is reused across the two calls otherwise
    bins[ch] = Buffer.from(p);
  });
  return { rgb: bins[1], seg: bins[2], depth: bins[3] };
}

function bufEqual(a, b) {
  if (!a || !b || a.length !== b.length) return false;
  return Buffer.compare(Buffer.from(a), Buffer.from(b)) === 0;
}

function decodeIds(rgba, w, h) {
  const out = new Int32Array(w * h);
  for (let i = 0; i < w * h; i++) {
    out[i] = (rgba[i * 4] << 16) | (rgba[i * 4 + 1] << 8) | rgba[i * 4 + 2];
  }
  return out;
}

function uniqueIds(rgba, w, h) {
  const set = new Set(decodeIds(rgba, w, h));
  set.delete(0);
  return set;
}

function labelHas(labels, name) {
  return Object.values(labels).includes(name);
}

async function deterministicRun() {
  const { envs } = await server.call('create_envs', { count: 1 });
  const e = envs[0];
  await server.call('load_urdf', { env: e, path: urdf });
  await server.call('add_ground', { env: e, visual: false });
  await server.call('spawn', { env: e, shape: 'sphere', size: [0.03], pos: [0.15, -0.25, 1.4], name: 'd' });
  const js = await server.call('list_joints', { env: e });
  await server.call('set_joint', { env: e, name: js[0].name, value: js[0].lower * 0.8 });
  await server.call('step', { env: e, dt: 1 / 60, n: 120 });
  const state = await server.call('get_state', { env: e });
  await server.call('destroy_envs', { env: e });
  return state.objects;
}
