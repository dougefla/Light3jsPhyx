import os from 'node:os';
import { Chromium } from './browser.js';

export interface PoolConfig {
  /** Parallel unit: each browser instance owns one GPU process. */
  browsers: number;
  pagesPerBrowser: number;
  envsPerPage: number;
  headless: boolean;
}

export const DEFAULT_POOL: PoolConfig = {
  browsers: Math.max(2, Math.min(8, Math.floor(os.cpus().length / 2))),
  pagesPerBrowser: 1,
  envsPerPage: 7,
  headless: true,
};

export function launchArgs(): string[] {
  // LIGHTPHYX_CHROMIUM_ARGS: extra switches appended verbatim (whitespace
  // separated) — for per-machine GPU/memory tuning without a code change.
  const extra = (process.env.LIGHTPHYX_CHROMIUM_ARGS ?? '').split(/\s+/).filter(Boolean);
  const common = [
    ...extra,
    '--force-color-profile=srgb',
    '--force-device-scale-factor=1',
    '--hide-scrollbars',
    '--disable-background-timer-throttling',
    '--disable-renderer-backgrounding',
  ];
  if (process.platform === 'darwin') return [...common, '--enable-gpu'];
  if (process.platform === 'linux') {
    if (process.env.LIGHTPHYX_GPU === 'vulkan') {
      // NVIDIA server path (see DESIGN §5.3): ANGLE over Vulkan, no surface.
      return [
        ...common,
        '--no-sandbox',
        '--use-angle=vulkan',
        '--enable-features=Vulkan',
        '--disable-vulkan-surface',
        '--disable-dev-shm-usage',
      ];
    }
    return [
      ...common,
      '--no-sandbox',
      '--disable-gpu',
      '--enable-unsafe-swiftshader',
      '--disable-dev-shm-usage', // containers often mount tiny /dev/shm
    ];
  }
  return common;
}

export interface PoolPage {
  pageId: string;
  /** CDP target id of the page inside its browser. */
  targetId: string;
  envCount: number;
}

interface Launch {
  promise: Promise<PoolPage>;
  /** Env slots on this page already promised to waiting reserve() calls. */
  claimed: number;
}

/**
 * One browser instance. `pages` is claimed synchronously (before the launch
 * even finishes) so concurrent page launches can never over-subscribe a slot
 * or over-launch browsers; `ready` resolves once the process is up.
 */
interface BrowserSlot {
  pages: number;
  ready: Promise<Chromium>;
  browser: Chromium | null;
}

/**
 * B×P×E pool of headless pages (see DESIGN.md §5.5). Browsers are the
 * parallel unit (one GPU process each); pages within a browser add JS
 * threads; envs within a page share loaded assets.
 *
 * Launches are parallel: reserving 50 envs on an empty pool starts all 8
 * browsers at once instead of one after another (which used to cost
 * ~400 ms × B on the critical path of every create_envs).
 */
export class PagePool {
  private browsers: BrowserSlot[] = [];
  /** Owning browser slot per page, so removePage frees the right slot. */
  private readonly pageSlots = new Map<string, BrowserSlot>();
  readonly pages: PoolPage[] = [];
  /**
   * Pages whose launch is in flight (warm-up or a reserve), with the env
   * slots already promised to waiting reservations. Concurrent reserve()
   * calls share these slots instead of each launching a page of its own —
   * otherwise N clients creating 1 env at once start N pages (N renderer
   * processes + WebGL contexts) and the browsers fall over around N≈60.
   */
  private readonly launching = new Set<Launch>();
  private nextPageId = 0;
  /** Callback fired when a page's renderer dies (crash or close). */
  onPageDown?: (pageId: string) => void;

  constructor(
    private readonly baseURL: string,
    readonly config: PoolConfig,
    /** Resolves when the page's WebSocket registers with the hub. */
    private readonly waitForPageConn: (pageId: string, timeoutMs: number) => Promise<void>,
  ) {}

  /**
   * Atomically reserve `count` env slots across the pool, returning
   * per-page allocations. All booking is synchronous (no await between a
   * capacity check and its increment) so concurrent callers can't
   * double-book: first rooms on live pages, then free rooms on pages still
   * launching, then new pages — all started right now, concurrently.
   * Roll back with release() if the downstream create RPC fails.
   */
  async reserve(count: number, retries = 1): Promise<Array<{ page: PoolPage; room: number }>> {
    const out: Array<{ page: PoolPage; room: number }> = [];
    const E = this.config.envsPerPage;
    let need = count;
    for (const p of this.pages) {
      if (need <= 0) break;
      const room = Math.min(E - p.envCount, need);
      if (room > 0) {
        p.envCount += room;
        out.push({ page: p, room });
        need -= room;
      }
    }
    const waits: Array<{ launch: Launch; room: number }> = [];
    for (const l of this.launching) {
      if (need <= 0) break;
      const room = Math.min(E - l.claimed, need);
      if (room > 0) {
        l.claimed += room;
        waits.push({ launch: l, room });
        need -= room;
      }
    }
    while (need > 0) {
      const l = this.startLaunch();
      const room = Math.min(E, need);
      l.claimed = room;
      waits.push({ launch: l, room });
      need -= room;
    }
    // A launched page enters `pages` with envCount = its claims already
    // applied (see launchPage), so there is nothing left to book here.
    const settled = await Promise.allSettled(waits.map((w) => w.launch.promise));
    let failed = 0;
    let firstErr: unknown;
    settled.forEach((s, i) => {
      if (s.status === 'fulfilled') out.push({ page: s.value, room: waits[i]!.room });
      else {
        failed += waits[i]!.room;
        firstErr ??= s.reason;
      }
    });
    if (failed > 0) {
      // A failed launch (e.g. a page that never connected) gets one more try
      // on fresh capacity before the whole reservation is abandoned.
      const more = retries > 0 ? await this.reserve(failed, retries - 1).catch(() => null) : null;
      if (!more) {
        // Undo what we booked so the caller sees a clean failure.
        for (const { page, room } of out) this.release(page.pageId, room);
        throw firstErr ?? new Error('pool could not satisfy reservation');
      }
      out.push(...more);
    }
    return out;
  }

  /**
   * Pre-launch enough pages for `envs` envs in the background. reserve() picks
   * the in-flight launches up, so a client connecting while browsers are still
   * starting never waits longer than it would have anyway.
   */
  warm(envs: number): void {
    const have = this.pages.length + this.launching.size;
    const want = Math.ceil(envs / this.config.envsPerPage);
    for (let i = have; i < want; i++) {
      // A failed warm launch is logged, not fatal: reserve() will retry.
      this.startLaunch().promise.catch((err: Error) => console.error('[pool] warm launch failed:', err.message));
    }
  }

  /** Return reserved-but-unused capacity (create RPC failed, or env destroyed). */
  release(pageId: string, count: number): void {
    const p = this.pages.find((x) => x.pageId === pageId);
    if (p) p.envCount = Math.max(0, p.envCount - count);
  }

  /** Drop a dead page: remove it from rotation and free its browser slot. */
  removePage(pageId: string): void {
    const i = this.pages.findIndex((p) => p.pageId === pageId);
    if (i >= 0) this.pages.splice(i, 1);
    const slot = this.pageSlots.get(pageId);
    this.pageSlots.delete(pageId);
    if (!slot) return;
    slot.pages = Math.max(0, slot.pages - 1);
    // A browser whose process died can never host a page again; forget it
    // so launchPage() starts a fresh one instead of failing on newPage().
    if (slot.browser?.exited) this.forgetSlot(slot);
  }

  private forgetSlot(slot: BrowserSlot): void {
    const j = this.browsers.indexOf(slot);
    if (j >= 0) this.browsers.splice(j, 1);
  }

  /** Synchronously pick (or start) the browser that will host the next page. */
  private claimSlot(): BrowserSlot {
    let slot = this.browsers.find((b) => b.pages < this.config.pagesPerBrowser);
    if (!slot) {
      if (this.browsers.length >= this.config.browsers) {
        // Pool at configured capacity: keep stacking pages round-robin.
        slot = this.browsers.reduce((a, b) => (a.pages <= b.pages ? a : b));
      } else {
        const created: BrowserSlot = {
          pages: 0,
          browser: null,
          ready: Chromium.launch({ headless: this.config.headless, args: launchArgs() }).then((browser) => {
            created.browser = browser;
            // A browser whose process died can never host a page again.
            browser.on('Chromium.exit', () => this.forgetSlot(created));
            return browser;
          }),
        };
        // A browser that never came up must not occupy a pool slot forever.
        created.ready.catch(() => this.forgetSlot(created));
        this.browsers.push(created);
        slot = created;
      }
    }
    slot.pages++;
    return slot;
  }

  private startLaunch(): Launch {
    const launch: Launch = { claimed: 0, promise: null! };
    this.launching.add(launch);
    launch.promise = this.launchPage(launch);
    return launch;
  }

  private async launchPage(launch: Launch): Promise<PoolPage> {
    try {
      const entry = await this.openPage();
      // Same tick: leave `launching` and enter `pages` with the claims made
      // while it launched, so no reserve() can see its capacity twice.
      this.launching.delete(launch);
      entry.envCount = launch.claimed;
      this.pages.push(entry);
      return entry;
    } catch (err) {
      this.launching.delete(launch);
      throw err;
    }
  }

  private async openPage(): Promise<PoolPage> {
    const slot = this.claimSlot();
    const pageId = `page_${this.nextPageId++}`;
    this.pageSlots.set(pageId, slot);
    let browser: Chromium;
    let targetId: string;
    try {
      browser = await slot.ready;
      ({ targetId } = (await browser.send('Target.createTarget', {
        url: `${this.baseURL}/?page=${pageId}`,
      })) as { targetId: string });
    } catch (err) {
      this.pageSlots.delete(pageId);
      slot.pages = Math.max(0, slot.pages - 1);
      throw err;
    }
    // Renderer death (GPU/OOM crash, or the page closing) must invalidate the
    // page so the hub stops routing envs to a corpse. Fires onPageDown once.
    let down = false;
    const offs: Array<() => void> = [];
    const markDown = () => {
      if (down) return;
      down = true;
      for (const off of offs) off();
      this.removePage(pageId);
      this.onPageDown?.(pageId);
    };
    const forThisTarget = (fn: () => void) => (p: Record<string, unknown>) => {
      if (p.targetId === targetId) fn();
    };
    offs.push(
      browser.on('Target.targetCrashed', forThisTarget(markDown)),
      browser.on('Target.targetDestroyed', forThisTarget(markDown)),
      browser.on('Chromium.exit', markDown),
    );
    // Surface page errors the way Playwright's pageerror/console did. Runtime
    // is cheap to enable; it is the Network domain we must never turn on.
    try {
      const { sessionId } = (await browser.send('Target.attachToTarget', { targetId, flatten: true })) as {
        sessionId: string;
      };
      await browser.send('Runtime.enable', {}, sessionId);
      offs.push(
        browser.on('Runtime.exceptionThrown', (p, sid) => {
          if (sid !== sessionId) return;
          const d = p.exceptionDetails as { text?: string; exception?: { description?: string } } | undefined;
          console.error(`[${pageId}]`, d?.exception?.description ?? d?.text ?? 'uncaught exception');
        }),
        browser.on('Runtime.consoleAPICalled', (p, sid) => {
          if (sid !== sessionId || p.type !== 'error') return;
          const args = (p.args as Array<{ value?: unknown; description?: string }> | undefined) ?? [];
          console.error(`[${pageId}]`, args.map((a) => a.value ?? a.description ?? '').join(' '));
        }),
      );
    } catch (err) {
      console.error(`[${pageId}] could not attach console logging:`, (err as Error).message);
    }
    try {
      await this.waitForPageConn(pageId, 30_000);
    } catch (err) {
      await browser.send('Target.closeTarget', { targetId }).catch(() => undefined);
      markDown();
      throw err;
    }
    if (down) throw new Error(`page ${pageId} died while starting`);
    return { pageId, targetId, envCount: 0 };
  }

  async dispose(): Promise<void> {
    // Let in-flight launches settle so no browser process outlives us.
    await Promise.allSettled([...this.launching].map((l) => l.promise));
    this.launching.clear();
    await Promise.allSettled(
      this.browsers.map(async (b) => {
        const browser = await b.ready;
        await browser.close();
      }),
    );
    this.browsers.length = 0;
    this.pages.length = 0;
  }
}
