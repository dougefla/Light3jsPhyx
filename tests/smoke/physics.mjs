import { chromium } from 'playwright';

const urdfPath = process.env.URDF_PATH ?? new URL('../assets/mini_cabinet.urdf', import.meta.url).pathname;
const base = process.env.BASE_URL ?? 'http://localhost:5173/';
const outDir = process.argv[2] ?? '.';

const browser = await chromium.launch({
  channel: 'chromium',
  headless: true,
  args: ['--enable-gpu', '--force-color-profile=srgb', '--force-device-scale-factor=1'],
});
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
page.on('pageerror', (e) => console.log('[pageerror]', e.message));

await page.goto(base + '?play=0&urdf=' + encodeURIComponent('/@fs' + urdfPath));
await page.waitForFunction(() => window.__lp?.loaded, undefined, { timeout: 30000 });

const result = await page.evaluate(async () => {
  const { env, THREE } = window.__lp;

  // Open the top drawer, drop a ball into the middle of its visual basin.
  env.setJoint('joint_0', -0.28);
  env.step(1 / 60, 30);
  const drawer = env.robot.links['link_0'];
  const visual = drawer.children.find((c) => c.isURDFVisual);
  const box = new THREE.Box3().setFromObject(visual);
  const c = box.getCenter(new THREE.Vector3());

  env.spawn({ shape: 'sphere', size: [0.03], pos: [c.x, c.y, box.max.z + 0.05], name: 'ball' });
  env.step(1 / 60, 120); // ~2s settle
  const before = env.getState().objects['ball'];

  // Close the drawer; the back wall should push/carry the ball with it.
  env.setJoint('joint_0', 0);
  env.step(1 / 60, 150); // ~2.5s (interpolated + settle)
  const after = env.getState().objects['ball'];

  return {
    before,
    after,
    deltaY: after.pos[1] - before.pos[1],
    contacts: env.contacts(),
    jointAfter: env.getJoint('joint_0'),
  };
});
console.log(JSON.stringify(result, null, 2));

await page.screenshot({ path: `${outDir}/m2_closed_with_ball.png` });

const inDrawer = result.contacts.some(
  ([a, b]) => (a.includes('link_0') || b.includes('link_0')) && (a === 'obj:ball' || b === 'obj:ball'),
);
const ok =
  inDrawer && // ball ends resting against/inside the closed drawer
  result.deltaY > 0.2 && // it was pushed along (rolling may exceed drawer travel)
  result.after.pos[2] > 0.9; // still at drawer height, not fallen to the ground
console.log(ok ? 'PHYSICS SMOKE PASS' : 'PHYSICS SMOKE FAIL');
await browser.close();
process.exit(ok ? 0 : 1);
