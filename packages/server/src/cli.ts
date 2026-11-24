#!/usr/bin/env node
import path from 'node:path';
import { SimServer, type ServerOptions } from './server.js';

interface Flags {
  [k: string]: string | boolean;
}

function parseArgs(argv: string[]): { cmd: string; flags: Flags } {
  const cmd = argv[0] && !argv[0].startsWith('-') ? argv[0] : 'serve';
  const flags: Flags = {};
  for (let i = cmd === argv[0] ? 1 : 0; i < argv.length; i++) {
    const a = argv[i]!;
    if (!a.startsWith('--')) continue;
    const key = a.slice(2);
    const next = argv[i + 1];
    if (next && !next.startsWith('--')) {
      flags[key] = next;
      i++;
    } else flags[key] = true;
  }
  return { cmd, flags };
}

function positiveInt(value: string, what: string): number {
  const n = Number(value);
  if (!Number.isInteger(n) || n < 1) throw new Error(`${what} must be a positive integer, got '${value}'`);
  return n;
}

/**
 * Browser count: --browsers N, else LIGHTPHYX_BROWSERS, else the pool
 * default (cores/2, 2..8). Each browser is one GPU process (~0.75 GB):
 * fewer saves memory, more renders in parallel (DESIGN §5.5).
 */
function browsersFlag(flags: Flags): number | undefined {
  if (flags.browsers !== undefined) return positiveInt(String(flags.browsers), '--browsers');
  const env = process.env.LIGHTPHYX_BROWSERS;
  return env ? positiveInt(env, 'LIGHTPHYX_BROWSERS') : undefined;
}

function poolFlag(flags: Flags, expectedEnvs: number | undefined): ServerOptions['pool'] {
  const browsers = browsersFlag(flags);
  if (typeof flags.pool === 'string') {
    const [b = 1, p = 1, e = 1] = flags.pool.split(',').map((x) => Number(x) || 1);
    if (browsers === undefined || browsers === b) return { browsers: b, pagesPerBrowser: p, envsPerPage: e };
    // Browser count overridden: keep the pool's total capacity B×P×E so a
    // caller that sized it for its clients doesn't start stacking pages.
    return { browsers, pagesPerBrowser: p, envsPerPage: Math.ceil((b * e) / browsers) };
  }
  if (browsers === undefined) return undefined;
  // Only the browser count was chosen: spread the expected envs over them
  // (one page each) instead of stacking extra pages at the default E.
  return expectedEnvs ? { browsers, envsPerPage: Math.ceil(expectedEnvs / browsers) } : { browsers };
}

/** expectedEnvs sizes E for --browsers without pre-launching (bench). */
function serverOptions(flags: Flags, expectedEnvs?: number): ServerOptions {
  // --warm-envs N: start launching browsers for N envs immediately, so the
  // first create_envs (or a client still connecting) finds them ready.
  const warmEnvs = typeof flags['warm-envs'] === 'string' ? Number(flags['warm-envs']) : undefined;
  const pool = poolFlag(flags, warmEnvs ?? expectedEnvs);
  const headed = flags.headed === true;
  return {
    port: flags.port ? Number(flags.port) : undefined,
    pool: headed ? { ...(pool ?? {}), headless: false } : pool,
    outDir: typeof flags.out === 'string' ? flags.out : undefined,
    warmEnvs,
  };
}

async function cmdServe(flags: Flags): Promise<void> {
  const server = new SimServer(serverOptions(flags));
  await server.start();
  console.log(`light-3js-phyx server listening on ${server.baseURL}`);
  console.log(`  WebUI:      ${server.baseURL}/`);
  console.log(`  SDK socket: ws://127.0.0.1:${server.port}/ws`);

  let shuttingDown = false;
  const shutdown = (why: string) => {
    if (shuttingDown) return;
    shuttingDown = true;
    console.log(`shutting down (${why})`);
    void server.dispose().finally(() => process.exit(0));
  };
  process.on('SIGTERM', () => shutdown('SIGTERM'));
  process.on('SIGINT', () => shutdown('SIGINT'));
  // Parent-watch (issue #13): with --exit-on-stdin-close the server holds a
  // stdin pipe from its parent; if the parent dies for ANY reason — even
  // SIGKILL — the OS closes the pipe and we shut down (browsers included).
  // Opt-in by flag, NOT by isTTY: manual `serve` under nohup/& gets /dev/null
  // stdin whose instant EOF would otherwise kill a legitimate daemon.
  if (flags['exit-on-stdin-close'] === true) {
    process.stdin.resume();
    process.stdin.on('end', () => shutdown('stdin closed'));
    process.stdin.on('close', () => shutdown('stdin closed'));
    process.stdin.on('error', () => shutdown('stdin error'));
  }
}

async function cmdCapture(flags: Flags): Promise<void> {
  const urdf = String(flags.urdf ?? '');
  if (!urdf) throw new Error('capture requires --urdf <path>');
  const [w, h] = String(flags.size ?? '1280x720').split('x').map(Number);
  const outDir = path.resolve(String(flags.out ?? 'out'));
  const seconds = Number(flags.seconds ?? 3);
  const fps = Number(flags.fps ?? 30);

  const server = new SimServer({ port: 0, pool: { browsers: 1, pagesPerBrowser: 1, envsPerPage: 1 }, outDir });
  await server.start();
  try {
    const { envs } = (await server.call('create_envs', { count: 1 })) as { envs: number[] };
    const env = envs[0]!;
    const loaded = (await server.call('load_urdf', { env, path: urdf })) as {
      joints: Array<{ name: string; lower: number; upper: number }>;
    };
    await server.call('add_ground', { env, visual: false });
    await server.call('set_camera', { env, width: w, height: h });
    await server.call('frame_camera', { env });

    const render = (await server.call('render', {
      env,
      passes: ['rgb', 'seg', 'depth'],
      save: { dir: outDir, prefix: 'capture' },
    })) as { files: string[] };
    console.log('images:');
    for (const f of render.files) console.log('  ' + f);

    if (flags.video !== false && flags['no-video'] !== true) {
      const videoPath = path.join(outDir, 'capture.mp4');
      await server.call('start_record', { env, fps, path: videoPath });
      // Sweep every movable joint from rest to its far limit and back.
      const steps = Math.round(seconds * fps);
      for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const phase = t < 0.5 ? t * 2 : (1 - t) * 2;
        const values: Record<string, number> = {};
        for (const j of loaded.joints) {
          const far = Math.abs(j.lower) > Math.abs(j.upper) ? j.lower : j.upper;
          values[j.name] = far * phase;
        }
        await server.call('set_joints', { env, values });
        await server.call('step', { env, dt: 1 / fps });
      }
      const rec = (await server.call('stop_record', { env })) as { path: string; frames: number };
      console.log(`video:\n  ${rec.path} (${rec.frames} frames)`);
    }
  } catch (err) {
    console.error(err);
    await server.dispose();
    process.exit(1);
  }
  await server.dispose();
  process.exit(0);
}

async function cmdBench(flags: Flags): Promise<void> {
  const urdf = String(flags.urdf ?? '');
  if (!urdf) throw new Error('bench requires --urdf <path>');
  const nEnvs = Number(flags.envs ?? 50);
  const steps = Number(flags.steps ?? 300);
  const renders = Number(flags.renders ?? 10);
  const [w, h] = String(flags.size ?? '640x480').split('x').map(Number);

  const server = new SimServer(serverOptions(flags, nEnvs));
  await server.start();
  try {
    let t = Date.now();
    const { envs } = (await server.call('create_envs', { count: nEnvs })) as { envs: number[] };
    console.log(`create ${envs.length} envs: ${Date.now() - t}ms`);

    t = Date.now();
    await server.call('load_urdf', { env: envs, path: urdf });
    await server.call('add_ground', { env: envs, visual: false });
    await server.call('set_camera', { env: envs, width: w, height: h });
    await server.call('frame_camera', { env: envs });
    console.log(`load urdf ×${envs.length}: ${Date.now() - t}ms`);

    // Give every env some dynamics so physics isn't trivially asleep.
    await server.call('spawn', { env: envs, shape: 'sphere', size: [0.03], pos: [0.1, 0.1, 1.3] });

    t = Date.now();
    await server.call('step', { env: 'all', dt: 1 / 60, n: steps });
    const stepMs = Date.now() - t;
    const envSteps = nEnvs * steps;
    console.log(
      `step ×${steps} on ${nEnvs} envs: ${stepMs}ms → ${Math.round(envSteps / (stepMs / 1000))} env-steps/s`,
    );

    t = Date.now();
    for (let i = 0; i < renders; i++) {
      await Promise.all(
        envs.map((env) => server.call('render', { env, passes: ['rgb', 'seg', 'depth'] })),
      );
    }
    const renderMs = Date.now() - t;
    const frames = nEnvs * renders * 3;
    console.log(
      `render ×${renders} (rgb+seg+depth @${w}x${h}) on ${nEnvs} envs: ${renderMs}ms → ${Math.round(
        frames / (renderMs / 1000),
      )} pass-frames/s`,
    );
    console.log((await server.call('pool_stats')) as object);
  } catch (err) {
    console.error(err);
    await server.dispose();
    process.exit(1);
  }
  await server.dispose();
  process.exit(0);
}

const { cmd, flags } = parseArgs(process.argv.slice(2));
const commands: Record<string, (f: Flags) => Promise<void>> = {
  serve: cmdServe,
  capture: cmdCapture,
  bench: cmdBench,
};
const run = commands[cmd];
if (!run) {
  console.error(`Unknown command: ${cmd}\nUsage: light-3js-phyx <serve|capture|bench> [--flags]`);
  process.exit(2);
}
run(flags).catch((err) => {
  console.error(err);
  process.exit(1);
});
