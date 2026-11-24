import { spawn, type ChildProcess } from 'node:child_process';
import { mkdtempSync, rmSync } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { chromium } from 'playwright';

/**
 * Minimal Chromium launcher + CDP client over --remote-debugging-pipe.
 *
 * Why not Playwright's launcher: Playwright enables the CDP `Network` domain
 * on every page it creates, and Chrome then streams every WebSocket frame the
 * page sends back to the driver as a base64 `Network.webSocketFrameSent`
 * event. For a pool page pushing 1.2 MB passes that meant ~1.6 MB of base64
 * per pass encoded in Chrome, piped to Node, `toString`ed and `JSON.parse`d —
 * roughly 1 ms of hub CPU per pass-frame and the single biggest hub cost at
 * scale. Driving the pool pages through this client (no `Network`) removes
 * that tax on both sides. Playwright is still used for `npx playwright
 * install chromium` and for locating that executable.
 */

type Handler = (params: Record<string, unknown>, sessionId?: string) => void;

interface Pending {
  resolve: (v: Record<string, unknown>) => void;
  reject: (e: Error) => void;
}

export interface LaunchOptions {
  headless: boolean;
  /** Extra Chromium switches (GPU flags etc.). */
  args: string[];
  executablePath?: string;
  /** Initial window size; pool pages only use it for the preview canvas. */
  windowSize?: { width: number; height: number };
}

/** Chromium switches Playwright applies (subset that matters headless/local). */
const BASE_SWITCHES = [
  '--disable-field-trial-config',
  '--disable-background-networking',
  '--disable-background-timer-throttling',
  '--disable-backgrounding-occluded-windows',
  '--disable-back-forward-cache',
  '--disable-breakpad',
  '--disable-client-side-phishing-detection',
  '--disable-component-extensions-with-background-pages',
  '--disable-component-update',
  '--no-default-browser-check',
  '--disable-default-apps',
  '--disable-extensions',
  // Memory (measured, 1 browser / 7 envs after rendering: 940 → 760 MB):
  // SpareRendererForSitePerProcess keeps a warm spare renderer (~100 MB)
  // per browser that a single-page pool never uses; NetworkServiceInProcess2
  // folds the network utility process (~100 MB) into the browser process.
  '--disable-features=Translate,PaintHolding,MediaRouter,DialMediaRouteProvider,GlobalMediaControls,OptimizationHints,HttpsUpgrades,DestroyProfileOnBrowserClose,SpareRendererForSitePerProcess',
  '--enable-features=NetworkServiceInProcess2',
  '--allow-pre-commit-input',
  '--disable-hang-monitor',
  '--disable-ipc-flooding-protection',
  '--disable-popup-blocking',
  '--disable-prompt-on-repost',
  '--disable-renderer-backgrounding',
  '--disable-search-engine-choice-screen',
  '--disable-sync',
  '--metrics-recording-only',
  '--no-first-run',
  '--no-service-autorun',
  '--password-store=basic',
  '--use-mock-keychain',
  '--no-sandbox', // Playwright's default on every platform; Linux args add it too
];

/**
 * Chromium honours only the LAST --enable-features / --disable-features
 * switch, so a caller-supplied one (launchArgs, LIGHTPHYX_CHROMIUM_ARGS) would
 * silently drop the built-in list. Merge them into one switch each.
 */
function mergeFeatureSwitches(args: string[]): string[] {
  const enable: string[] = [];
  const disable: string[] = [];
  const rest: string[] = [];
  for (const a of args) {
    if (a.startsWith('--enable-features=')) enable.push(...a.slice(18).split(','));
    else if (a.startsWith('--disable-features=')) disable.push(...a.slice(19).split(','));
    else rest.push(a);
  }
  const uniq = (xs: string[]) => [...new Set(xs.filter(Boolean))];
  if (enable.length) rest.push(`--enable-features=${uniq(enable).join(',')}`);
  if (disable.length) rest.push(`--disable-features=${uniq(disable).join(',')}`);
  return rest;
}

export class Chromium {
  private nextId = 1;
  private readonly pending = new Map<number, Pending>();
  private readonly handlers = new Map<string, Set<Handler>>();
  private chunks: Buffer[] = [];
  private stderrTail = '';
  /** True once the process is gone (exit or kill). */
  exited = false;
  readonly exitPromise: Promise<void>;
  private resolveExit!: () => void;

  private constructor(
    private readonly proc: ChildProcess,
    private readonly userDataDir: string,
  ) {
    this.exitPromise = new Promise((r) => (this.resolveExit = r));
    const toChrome = proc.stdio[3] as NodeJS.WritableStream;
    const fromChrome = proc.stdio[4] as NodeJS.ReadableStream;
    fromChrome.on('data', (chunk: Buffer) => this.onData(chunk));
    (proc.stderr as NodeJS.ReadableStream | null)?.on('data', (d: Buffer) => {
      this.stderrTail = (this.stderrTail + d.toString()).slice(-4000);
    });
    toChrome.on('error', () => undefined);
    proc.on('exit', () => {
      this.exited = true;
      const err = new Error('browser process exited');
      for (const p of this.pending.values()) p.reject(err);
      this.pending.clear();
      this.emit('Chromium.exit', {});
      rmSync(this.userDataDir, { recursive: true, force: true });
      this.resolveExit();
    });
  }

  static async launch(opts: LaunchOptions): Promise<Chromium> {
    const exe = opts.executablePath ?? chromium.executablePath();
    const userDataDir = mkdtempSync(path.join(os.tmpdir(), 'lightphyx-chromium-'));
    const size = opts.windowSize ?? { width: 640, height: 480 };
    const args = mergeFeatureSwitches([
      ...BASE_SWITCHES,
      ...(opts.headless ? ['--headless', '--hide-scrollbars', '--mute-audio'] : []),
      `--window-size=${size.width},${size.height}`,
      ...opts.args,
      `--user-data-dir=${userDataDir}`,
      '--remote-debugging-pipe',
      '--no-startup-window',
    ]);
    let proc: ChildProcess;
    try {
      proc = spawn(exe, args, { stdio: ['ignore', 'ignore', 'pipe', 'pipe', 'pipe'] });
    } catch (err) {
      rmSync(userDataDir, { recursive: true, force: true });
      throw err;
    }
    const browser = new Chromium(proc, userDataDir);
    try {
      // First round-trip doubles as the "pipe is up" handshake; it also turns
      // on the target lifecycle events the pool needs for crash detection.
      await browser.send('Target.setDiscoverTargets', { discover: true }, undefined, 30_000);
    } catch (err) {
      await browser.close();
      throw new Error(`Chromium failed to start: ${(err as Error).message}\n${browser.stderrTail}`);
    }
    return browser;
  }

  /** Send a CDP command (optionally to an attached session) and await its result. */
  send(
    method: string,
    params: Record<string, unknown> = {},
    sessionId?: string,
    timeoutMs = 30_000,
  ): Promise<Record<string, unknown>> {
    if (this.exited) return Promise.reject(new Error('browser process exited'));
    const id = this.nextId++;
    const msg = JSON.stringify(sessionId ? { id, method, params, sessionId } : { id, method, params });
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        if (this.pending.delete(id)) reject(new Error(`CDP ${method} timed out`));
      }, timeoutMs);
      this.pending.set(id, {
        resolve: (v) => {
          clearTimeout(timer);
          resolve(v);
        },
        reject: (e) => {
          clearTimeout(timer);
          reject(e);
        },
      });
      (this.proc.stdio[3] as NodeJS.WritableStream).write(msg + '\0');
    });
  }

  on(method: string, handler: Handler): () => void {
    let set = this.handlers.get(method);
    if (!set) this.handlers.set(method, (set = new Set()));
    set.add(handler);
    return () => set!.delete(handler);
  }

  private emit(method: string, params: Record<string, unknown>, sessionId?: string): void {
    const set = this.handlers.get(method);
    if (!set) return;
    for (const h of set) {
      try {
        h(params, sessionId);
      } catch (err) {
        console.error(`[cdp] handler for ${method} threw:`, (err as Error).message);
      }
    }
  }

  private onData(chunk: Buffer): void {
    // Messages are JSON documents terminated by NUL.
    let start = 0;
    let end = chunk.indexOf(0);
    while (end !== -1) {
      const part = chunk.subarray(start, end);
      const text = this.chunks.length ? Buffer.concat([...this.chunks, part]).toString() : part.toString();
      this.chunks = [];
      this.dispatch(text);
      start = end + 1;
      end = chunk.indexOf(0, start);
    }
    if (start < chunk.length) this.chunks.push(chunk.subarray(start));
  }

  private dispatch(text: string): void {
    let msg: {
      id?: number;
      result?: Record<string, unknown>;
      error?: { message: string };
      method?: string;
      params?: Record<string, unknown>;
      sessionId?: string;
    };
    try {
      msg = JSON.parse(text);
    } catch {
      return;
    }
    if (msg.id !== undefined) {
      const p = this.pending.get(msg.id);
      if (!p) return;
      this.pending.delete(msg.id);
      if (msg.error) p.reject(new Error(msg.error.message));
      else p.resolve(msg.result ?? {});
    } else if (msg.method) {
      this.emit(msg.method, msg.params ?? {}, msg.sessionId);
    }
  }

  /** Graceful Browser.close with a kill fallback; resolves once the process is gone. */
  async close(): Promise<void> {
    if (this.exited) return;
    const timer = setTimeout(() => this.proc.kill('SIGKILL'), 3000);
    try {
      await this.send('Browser.close', {}, undefined, 2500).catch(() => this.proc.kill('SIGTERM'));
      await this.exitPromise;
    } finally {
      clearTimeout(timer);
    }
  }
}
