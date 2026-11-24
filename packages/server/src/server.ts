import http from 'node:http';
import path from 'node:path';
import { mkdirSync, writeFileSync } from 'node:fs';
import { WebSocketServer, WebSocket, type RawData } from 'ws';
import { createStaticHandler, webuiDistDir } from './static.js';
import { PagePool, DEFAULT_POOL, type PoolConfig } from './pool.js';
import { Mp4Muxer } from './mux.js';
import { writeRGBAPng, writeDepthPng16, writeNpyFloat32 } from './images.js';
import {
  CHANNEL,
  BINARY_HEADER_BYTES,
  encodeBinaryFrame,
  decodeBinaryFrame,
  type RpcRequest,
  type RpcResponse,
} from './protocol.js';

/**
 * Receives one binary frame from a page. `payload` is a zero-copy view of the
 * received frame (header stripped); `frame` is the whole received frame,
 * owned by the hub, so a forwarding sink may rewrite the 4-byte request id in
 * place and send it on without a 1–5 MB copy per pass.
 */
type BinarySink = (channel: number, payload: Buffer, frame: Buffer) => void;

/** Server-side handle for one connected pool page. */
class PageConn {
  private nextId = 1;
  private closed = false;
  private readonly pending = new Map<
    number,
    { resolve: (v: unknown) => void; reject: (e: Error) => void; onBinary?: BinarySink }
  >();
  private readonly streams = new Map<number, BinarySink>();
  /** Backstop so a call whose page silently died still settles. */
  private readonly callTimeoutMs = 120_000;

  constructor(readonly ws: WebSocket) {
    ws.on('message', (data: RawData, isBinary: boolean) => {
      // A malformed frame from a page must never crash the hub — drop it.
      try {
        if (isBinary) {
          const buf = data as Buffer;
          if (buf.length < BINARY_HEADER_BYTES) return;
          const { requestId, channel, payload } = decodeBinaryFrame(buf);
          (this.pending.get(requestId)?.onBinary ?? this.streams.get(requestId))?.(channel, payload, buf);
          return;
        }
        const msg = JSON.parse(String(data)) as RpcResponse & { hello?: string };
        if (msg.hello) return;
        const entry = this.pending.get(msg.id);
        if (!entry) return;
        this.pending.delete(msg.id);
        if (msg.error) entry.reject(new Error(msg.error.message));
        else entry.resolve(msg.result);
      } catch (err) {
        console.error('[page] dropped malformed frame:', (err as Error).message);
      }
    });
    ws.on('close', () => this.fail(new Error('page disconnected')));
    ws.on('error', () => this.fail(new Error('page socket error')));
  }

  /** Reject everything in flight and refuse further calls (page is gone). */
  fail(err: Error): void {
    this.closed = true;
    for (const { reject } of this.pending.values()) reject(err);
    this.pending.clear();
    this.streams.clear();
  }

  private send(id: number, method: string, params: Record<string, unknown>): void {
    this.ws.send(JSON.stringify({ id, method, params } satisfies RpcRequest));
  }

  call(method: string, params: Record<string, unknown>, onBinary?: BinarySink): Promise<unknown> {
    if (this.closed) return Promise.reject(new Error('page disconnected'));
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        if (this.pending.delete(id)) reject(new Error(`page RPC '${method}' timed out`));
      }, this.callTimeoutMs);
      this.pending.set(id, {
        resolve: (v) => {
          clearTimeout(timer);
          resolve(v);
        },
        reject: (e) => {
          clearTimeout(timer);
          reject(e);
        },
        onBinary,
      });
      this.send(id, method, params);
    });
  }

  /** Like call(), but binary frames keep routing to sink after resolution. */
  async callStream(
    method: string,
    params: Record<string, unknown>,
    sink: BinarySink,
  ): Promise<{ result: unknown; streamId: number }> {
    if (this.closed) throw new Error('page disconnected');
    const id = this.nextId++;
    this.streams.set(id, sink);
    try {
      const result = await new Promise((resolve, reject) => {
        this.pending.set(id, { resolve, reject });
        this.send(id, method, params);
      });
      return { result, streamId: id };
    } catch (err) {
      this.streams.delete(id);
      throw err;
    }
  }

  closeStream(streamId: number): void {
    this.streams.delete(streamId);
  }
}

interface EnvEntry {
  conn: PageConn;
  pageId: string;
  local: number;
}

interface Recording {
  muxer: Mp4Muxer;
  conn: PageConn;
  streamId: number;
  writes: Promise<void>;
}

export interface ServerOptions {
  port?: number;
  host?: string;
  pool?: Partial<PoolConfig>;
  outDir?: string;
  distDir?: string;
  /** Pre-launch enough pool pages for this many envs as soon as start() returns. */
  warmEnvs?: number;
}

export class SimServer {
  private readonly httpServer: http.Server;
  private readonly wss = new WebSocketServer({ noServer: true });
  private readonly pageConns = new Map<string, PageConn>();
  private readonly pageWaiters = new Map<string, () => void>();
  private readonly envs = new Map<number, EnvEntry>();
  private readonly recordings = new Map<number, Recording>();
  private nextGlobalEnv = 0;
  private pool: PagePool | null = null;
  readonly outDir: string;
  port = 0;

  constructor(private readonly opts: ServerOptions = {}) {
    this.outDir = path.resolve(opts.outDir ?? 'out');
    const statics = createStaticHandler(opts.distDir ?? webuiDistDir());
    this.httpServer = http.createServer(statics);
    this.httpServer.on('upgrade', (req, socket, head) => {
      const url = new URL(req.url ?? '/', 'http://localhost');
      if (url.pathname !== '/ws' && url.pathname !== '/page') {
        socket.destroy();
        return;
      }
      // Cross-site WebSocket hijacking guard: binding to 127.0.0.1 does NOT
      // stop a page the user is browsing from opening ws://127.0.0.1:8323/ws
      // and driving the engine / overwriting output files. Reject any request
      // that carries a cross-origin Origin header. Non-browser clients (the
      // Python SDK, Playwright pages) send no Origin and are allowed.
      if (!this.isAllowedOrigin(req.headers.origin)) {
        socket.write('HTTP/1.1 403 Forbidden\r\n\r\n');
        socket.destroy();
        return;
      }
      this.wss.handleUpgrade(req, socket, head, (ws) => {
        if (url.pathname === '/page') this.registerPage(url.searchParams.get('id') ?? '', ws);
        else this.serveSdkClient(ws);
      });
    });
  }

  async start(): Promise<void> {
    const host = this.opts.host ?? '127.0.0.1';
    await new Promise<void>((resolve) =>
      this.httpServer.listen(this.opts.port ?? 8323, host, resolve),
    );
    const addr = this.httpServer.address();
    this.port = typeof addr === 'object' && addr ? addr.port : 0;
    this.pool = new PagePool(
      `http://127.0.0.1:${this.port}`,
      { ...DEFAULT_POOL, ...this.opts.pool },
      (pageId, timeoutMs) => this.waitForPage(pageId, timeoutMs),
    );
    this.pool.onPageDown = (pageId) => this.invalidatePage(pageId);
    mkdirSync(this.outDir, { recursive: true });
    if (this.opts.warmEnvs && this.opts.warmEnvs > 0) this.pool.warm(this.opts.warmEnvs);
  }

  /** A page died: reject its in-flight calls and evict its envs/recordings. */
  private invalidatePage(pageId: string): void {
    const conn = this.pageConns.get(pageId);
    this.pageConns.delete(pageId);
    conn?.fail(new Error(`page ${pageId} crashed`));
    for (const [gid, entry] of [...this.envs]) {
      if (entry.pageId !== pageId) continue;
      this.envs.delete(gid);
      const rec = this.recordings.get(gid);
      if (rec) {
        this.recordings.delete(gid);
        void rec.muxer.finish().catch(() => undefined); // salvage what was written
      }
    }
  }

  get baseURL(): string {
    return `http://127.0.0.1:${this.port}`;
  }

  /** Allow same-origin browser pages and non-browser clients (no Origin). */
  private isAllowedOrigin(origin: string | undefined): boolean {
    if (!origin) return true; // SDK / Playwright: no Origin header
    try {
      const host = new URL(origin).hostname;
      return host === '127.0.0.1' || host === 'localhost' || host === '[::1]';
    } catch {
      return false;
    }
  }

  private registerPage(pageId: string, ws: WebSocket): void {
    this.pageConns.set(pageId, new PageConn(ws));
    this.pageWaiters.get(pageId)?.();
    this.pageWaiters.delete(pageId);
  }

  private waitForPage(pageId: string, timeoutMs: number): Promise<void> {
    if (this.pageConns.has(pageId)) return Promise.resolve();
    return new Promise((resolve, reject) => {
      const timer = setTimeout(
        () => reject(new Error(`page ${pageId} did not connect within ${timeoutMs}ms`)),
        timeoutMs,
      );
      this.pageWaiters.set(pageId, () => {
        clearTimeout(timer);
        resolve();
      });
    });
  }

  private env(id: unknown): EnvEntry & { gid: number } {
    const gid = Number(id);
    const entry = this.envs.get(gid);
    if (!entry) throw new Error(`No env ${String(id)}`);
    return { ...entry, gid };
  }

  private envIds(param: unknown): number[] {
    if (param === 'all' || param === undefined) return [...this.envs.keys()];
    if (Array.isArray(param)) return param.map(Number);
    return [Number(param)];
  }

  private toURL(p: string): string {
    if (/^https?:\/\//.test(p)) return p;
    // Encode each path segment so filenames containing '#', '?', spaces, etc.
    // survive as path data instead of being parsed as URL fragment/query.
    const encoded = path
      .resolve(p)
      .split('/')
      .map((seg) => encodeURIComponent(seg))
      .join('/');
    return `${this.baseURL}/files${encoded}`;
  }

  /** Core router — used by SDK sockets and by the CLI in-process. */
  async call(
    method: string,
    params: Record<string, unknown> = {},
    sink?: BinarySink,
  ): Promise<unknown> {
    switch (method) {
      case 'create_envs': {
        if (!this.pool) throw new Error('Server not started');
        const count = Number(params.count ?? 1);
        // Reserve capacity atomically up front (race-safe); each page's slots
        // are booked synchronously before any create RPC is sent. The create
        // RPCs then go out to every page at once (each page builds its envs
        // in its own process — serializing them here just added latency).
        const reservations = await this.pool.reserve(count);
        const results = await Promise.allSettled(
          reservations.map(async ({ page, room }) => {
            const conn = this.pageConns.get(page.pageId);
            if (!conn) throw new Error(`page ${page.pageId} died between reserve and use`);
            const res = (await conn.call('create_envs', {
              count: room,
              physics: params.physics,
              self_collision: params.self_collision,
            })) as { envs: number[] };
            return { conn, page, room, locals: res.envs };
          }),
        );
        const failed = results.filter((r): r is PromiseRejectedResult => r.status === 'rejected');
        if (failed.length > 0) {
          // All-or-nothing: tear down what did get created and free every
          // reservation, so the caller never ends up with envs it can't name.
          await Promise.allSettled(
            results.map(async (r, i) => {
              const { page, room } = reservations[i]!;
              if (r.status === 'fulfilled' && r.value.locals.length > 0) {
                await r.value.conn.call('destroy_envs', { env: r.value.locals }).catch(() => undefined);
              }
              this.pool!.release(page.pageId, room);
            }),
          );
          throw failed[0]!.reason;
        }
        // Global ids are handed out in reservation order — deterministic
        // regardless of which page answered first.
        const created: number[] = [];
        for (const r of results) {
          if (r.status !== 'fulfilled') continue;
          const { conn, page, room, locals } = r.value;
          for (const local of locals) {
            const gid = this.nextGlobalEnv++;
            this.envs.set(gid, { conn, pageId: page.pageId, local });
            created.push(gid);
          }
          // Reconcile if the page created fewer than reserved.
          if (locals.length < room) this.pool.release(page.pageId, room - locals.length);
        }
        return { envs: created };
      }

      case 'destroy_envs': {
        const byConn = this.groupByConn(this.envIds(params.env));
        for (const [conn, ids] of byConn) {
          await conn.call('destroy_envs', { env: ids.map((g) => this.envs.get(g)!.local) });
          for (const g of ids) {
            const entry = this.envs.get(g)!;
            // Stop any recording on a destroyed env, or its muxer + stream
            // leak (env is gone, stop_record can never reach it).
            const rec = this.recordings.get(g);
            if (rec) {
              this.recordings.delete(g);
              rec.conn.closeStream(rec.streamId);
              void rec.writes.then(() => rec.muxer.finish()).catch(() => undefined);
            }
            // Release the page's capacity quota (issue #8).
            this.pool?.release(entry.pageId, 1);
            this.envs.delete(g);
          }
        }
        return { ok: true };
      }

      case 'step': {
        const byConn = this.groupByConn(this.envIds(params.env));
        await Promise.all(
          [...byConn].map(([conn, ids]) =>
            conn.call('step', {
              env: ids.map((g) => this.envs.get(g)!.local),
              dt: params.dt,
              n: params.n,
            }),
          ),
        );
        return { ok: true };
      }

      case 'load_urdf': {
        return this.fanout(params, (entry, p) =>
          entry.conn.call('load_urdf', { ...p, env: entry.local, url: this.toURL(String(p.path ?? p.url)) }),
        );
      }

      case 'render': {
        const entry = this.env(params.env);
        const save = params.save as { dir?: string; prefix?: string } | undefined;
        const collected: Array<{ channel: number; payload: Buffer }> = [];
        const result = (await entry.conn.call(
          'render',
          { ...params, env: entry.local, save: undefined },
          (channel, payload, frame) => {
            sink?.(channel, payload, frame);
            if (save) collected.push({ channel, payload });
          },
        )) as { width: number; height: number; seg_labels?: Record<number, string> };

        if (save) {
          const dir = path.resolve(save.dir ?? this.outDir);
          mkdirSync(dir, { recursive: true });
          const prefix = save.prefix ?? `env${entry.gid}`;
          const files: string[] = [];
          for (const { channel, payload } of collected) {
            const { width, height } = result;
            if (channel === CHANNEL.rgb) {
              const f = path.join(dir, `${prefix}_rgb.png`);
              await writeRGBAPng(payload, width, height, f);
              files.push(f);
            } else if (channel === CHANNEL.seg) {
              const f = path.join(dir, `${prefix}_seg.png`);
              await writeRGBAPng(payload, width, height, f);
              files.push(f);
              const labels = path.join(dir, `${prefix}_seg_labels.json`);
              writeFileSync(labels, JSON.stringify(result.seg_labels ?? {}, null, 2));
              files.push(labels);
            } else if (channel === CHANNEL.depth) {
              const depth = new Float32Array(payload.buffer.slice(payload.byteOffset, payload.byteOffset + payload.byteLength));
              const f16 = path.join(dir, `${prefix}_depth.png`);
              await writeDepthPng16(depth, width, height, f16);
              files.push(f16);
              const npy = path.join(dir, `${prefix}_depth.npy`);
              await writeNpyFloat32(depth, height, width, npy);
              files.push(npy);
            }
          }
          return { ...result, files };
        }
        return result;
      }

      case 'start_record': {
        const entry = this.env(params.env);
        if (this.recordings.has(entry.gid)) throw new Error(`env ${entry.gid} already recording`);
        const fps = Number(params.fps ?? 30);
        const outPath = path.resolve(
          String(params.path ?? path.join(this.outDir, `env${entry.gid}_${Date.now()}.mp4`)),
        );
        mkdirSync(path.dirname(outPath), { recursive: true });
        const muxer = new Mp4Muxer(outPath, fps);
        await muxer.start();
        const rec: Recording = { muxer, conn: entry.conn, streamId: 0, writes: Promise.resolve() };
        let result: unknown, streamId: number;
        try {
          ({ result, streamId } = await entry.conn.callStream(
            'start_record',
            { ...params, env: entry.local, path: undefined },
            (channel, payload) => {
              if (channel === CHANNEL.video) rec.writes = rec.writes.then(() => muxer.write(payload));
            },
          ));
        } catch (err) {
          // Encoder failed to start (e.g. no H.264): don't leak the muxer/ffmpeg.
          await muxer.finish().catch(() => undefined);
          throw err;
        }
        rec.streamId = streamId;
        this.recordings.set(entry.gid, rec);
        return { ...(result as object), path: outPath };
      }

      case 'stop_record': {
        const entry = this.env(params.env);
        const rec = this.recordings.get(entry.gid);
        if (!rec) throw new Error(`env ${entry.gid} not recording`);
        const result = (await entry.conn.call('stop_record', { env: entry.local })) as object;
        await rec.writes;
        rec.conn.closeStream(rec.streamId);
        const finalPath = await rec.muxer.finish();
        this.recordings.delete(entry.gid);
        return { ...result, path: finalPath };
      }

      case 'pool_stats': {
        return {
          pages: this.pool?.pages.map((p) => ({ id: p.pageId, envs: p.envCount })) ?? [],
          envs: this.envs.size,
        };
      }

      // Plain env-addressed forwards (single env, list, or 'all').
      case 'list_joints':
      case 'set_joint':
      case 'set_joints':
      case 'set_joint_drive':
      case 'get_state':
      case 'spawn':
      case 'remove_object':
      case 'remove_robot':
      case 'add_ground':
      case 'contacts':
      case 'self_contacts':
      case 'reset':
      case 'set_camera':
      case 'frame_camera':
      case 'seg_labels':
        return this.fanout(params, (entry, p) => entry.conn.call(method, { ...p, env: entry.local }));

      default:
        throw new Error(`Unknown method: ${method}`);
    }
  }

  /** Single env → single result; list/'all' → {results: {gid: result}}. */
  private async fanout(
    params: Record<string, unknown>,
    fn: (entry: EnvEntry & { gid: number }, params: Record<string, unknown>) => Promise<unknown>,
  ): Promise<unknown> {
    const single = typeof params.env === 'number' || typeof params.env === 'string' && params.env !== 'all';
    const ids = this.envIds(params.env);
    const results = await Promise.all(ids.map((gid) => fn(this.env(gid), params)));
    if (single && ids.length === 1) return results[0];
    return { results: Object.fromEntries(ids.map((gid, i) => [gid, results[i]])) };
  }

  private groupByConn(ids: number[]): Map<PageConn, number[]> {
    const map = new Map<PageConn, number[]>();
    for (const gid of ids) {
      const entry = this.env(gid);
      const list = map.get(entry.conn) ?? [];
      list.push(gid);
      map.set(entry.conn, list);
    }
    return map;
  }

  private serveSdkClient(ws: WebSocket): void {
    // Envs this socket created. A client that dies without close() (SIGKILL,
    // timeout kill) would otherwise hold its pool slots until the server
    // exits; under many short-lived workers that shrinks capacity until every
    // new client forces an extra page launch and the browsers fall over.
    const owned = new Set<number>();
    let gone = false;
    ws.on('close', () => {
      gone = true;
      const ids = [...owned].filter((g) => this.envs.has(g));
      owned.clear();
      if (ids.length === 0) return;
      this.call('destroy_envs', { env: ids }).catch((err: Error) =>
        console.error(`[hub] reclaiming ${ids.length} env(s) of a closed client failed:`, err.message),
      );
    });
    ws.on('message', (data: RawData, isBinary: boolean) => {
      if (isBinary) return;
      let req: RpcRequest;
      try {
        req = JSON.parse(String(data)) as RpcRequest;
      } catch {
        ws.send(JSON.stringify({ id: null, error: { message: 'invalid JSON' } }));
        return;
      }
      // Forward page frames zero-copy: the hub owns the received buffer, so
      // stamping the SDK's request id over the page's id is enough. Frames
      // synthesized without an owning buffer fall back to a fresh encode.
      const sink: BinarySink = (channel, payload, frame) => {
        if (frame && frame.length === BINARY_HEADER_BYTES + payload.length) {
          frame.writeUInt32LE(req.id >>> 0, 0);
          frame.writeUInt8(channel, 4);
          ws.send(frame);
        } else {
          ws.send(encodeBinaryFrame(req.id, channel, payload));
        }
      };
      this.call(req.method, req.params ?? {}, sink)
        .then((result) => {
          if (req.method === 'create_envs') {
            const ids = (result as { envs: number[] }).envs;
            // Created after the client vanished: nobody can name these envs.
            if (gone) void this.call('destroy_envs', { env: ids }).catch(() => undefined);
            else for (const g of ids) owned.add(g);
          } else if (req.method === 'destroy_envs') {
            for (const g of this.envIds(req.params?.env)) owned.delete(g);
          }
          ws.send(JSON.stringify({ id: req.id, result: result ?? null }));
        })
        .catch((err: Error) => ws.send(JSON.stringify({ id: req.id, error: { message: err.message } })));
    });
  }

  async dispose(): Promise<void> {
    // Flush any in-flight recordings so their mp4/h264 files are finalized
    // (and ffmpeg children reaped) rather than orphaned on shutdown.
    await Promise.allSettled(
      [...this.recordings.values()].map((rec) => rec.writes.then(() => rec.muxer.finish())),
    );
    this.recordings.clear();
    await this.pool?.dispose();
    this.wss.close();
    await new Promise<void>((resolve) => this.httpServer.close(() => resolve()));
  }
}
