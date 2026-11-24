import { describe, it, expect } from 'vitest';
import { PagePool } from '../../packages/server/src/pool.js';

// N clients each creating 1 env at the same moment used to launch N pages
// (one per reserve call: in-flight launches were invisible to each other),
// i.e. N renderer processes + WebGL contexts — the server fell over near 60.
// Browser launch is stubbed out; only the booking logic is under test.
function stubPool(envsPerPage: number, opts: { failFirst?: number } = {}) {
  const pool = new PagePool('http://stub', { browsers: 8, pagesPerBrowser: 1, envsPerPage, headless: true }, async () => {});
  let opened = 0;
  let failures = opts.failFirst ?? 0;
  (pool as unknown as { openPage: () => Promise<unknown> }).openPage = async () => {
    await new Promise((r) => setTimeout(r, 20));
    if (failures-- > 0) throw new Error('launch failed');
    return { pageId: `page_${opened++}`, targetId: 't', envCount: 0 };
  };
  return { pool, opened: () => opened };
}

describe('PagePool.reserve', () => {
  it('concurrent single-env reservations share in-flight pages', async () => {
    const { pool, opened } = stubPool(7);
    const res = await Promise.all(Array.from({ length: 60 }, () => pool.reserve(1)));
    expect(opened()).toBe(9); // ⌈60/7⌉, not 60
    expect(res.flat().reduce((n, r) => n + r.room, 0)).toBe(60);
    expect(pool.pages.map((p) => p.envCount).sort((a, b) => a - b)).toEqual([4, 7, 7, 7, 7, 7, 7, 7, 7]);
  });

  it('warm-up launches are filled by later reservations, never double-booked', async () => {
    const { pool, opened } = stubPool(4);
    pool.warm(8);
    const res = await Promise.all(Array.from({ length: 10 }, () => pool.reserve(1)));
    expect(opened()).toBe(3);
    expect(res.flat().length).toBe(10);
    expect(pool.pages.every((p) => p.envCount <= 4)).toBe(true);
    expect(pool.pages.reduce((n, p) => n + p.envCount, 0)).toBe(10);
  });

  it('retries a failed launch once, and rolls back cleanly when that fails too', async () => {
    const ok = stubPool(4, { failFirst: 1 });
    const res = await ok.pool.reserve(6);
    expect(res.reduce((n, r) => n + r.room, 0)).toBe(6);

    const bad = stubPool(4, { failFirst: 99 });
    await expect(bad.pool.reserve(6)).rejects.toThrow(/launch failed/);
    expect(bad.pool.pages.reduce((n, p) => n + p.envCount, 0)).toBe(0);
  });
});
