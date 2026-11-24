import { chromium } from 'playwright';

const urdfPath = process.env.URDF_PATH ?? new URL('../assets/mini_cabinet.urdf', import.meta.url).pathname;
const urdf = '/@fs' + urdfPath;
const base = process.env.BASE_URL ?? 'http://localhost:5173/';
const out = process.argv[2] ?? '/tmp/shot.png';
const jointArg = process.argv[3]; // e.g. "joint_0=-0.25,joint_6=-0.2"

const browser = await chromium.launch({
  channel: 'chromium',
  headless: true,
  args: ['--enable-gpu', '--force-color-profile=srgb', '--force-device-scale-factor=1'],
});
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
page.on('console', (m) => console.log('[console]', m.type(), m.text()));
page.on('pageerror', (e) => console.log('[pageerror]', e.message));

await page.goto(base + '?urdf=' + encodeURIComponent(urdf));
await page.waitForFunction(() => window.__lp?.loaded, undefined, { timeout: 30000 });

const info = await page.evaluate(() => {
  const env = window.__lp.env;
  let meshes = 0;
  let textured = 0;
  env.scene.traverse((o) => {
    if (o.isMesh && o.visible) {
      meshes++;
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      if (mats.some((m) => m.map)) textured++;
    }
  });
  return {
    meshes,
    textured,
    joints: env.listJoints(),
    webgl: window.__lp.renderer.getContext().getParameter(
      window.__lp.renderer.getContext().VERSION,
    ),
  };
});
console.log(JSON.stringify(info, null, 2));

if (jointArg) {
  await page.evaluate((spec) => {
    for (const kv of spec.split(',')) {
      const [name, v] = kv.split('=');
      window.__lp.env.setJoint(name, Number(v));
    }
  }, jointArg);
}
await page.waitForTimeout(300);
await page.screenshot({ path: out });
console.log('screenshot:', out);
await browser.close();
