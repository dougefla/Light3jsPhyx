import { spawn, type ChildProcess } from 'node:child_process';
import { createWriteStream, type WriteStream } from 'node:fs';
import { once } from 'node:events';

/**
 * Muxes an H.264 Annex B elementary stream (WebCodecs output) into an mp4
 * with zero re-encode. Falls back to streaming a bare .h264 file when ffmpeg
 * is unavailable — the fallback writes to disk incrementally rather than
 * buffering the whole video in RAM (a long recording would otherwise OOM).
 */
export class Mp4Muxer {
  private proc: ChildProcess | null = null;
  private fallback: WriteStream | null = null;
  private finished = false;
  private finalPath: string;
  readonly outPath: string;

  constructor(outPath: string, private readonly fps: number) {
    this.outPath = outPath;
    this.finalPath = outPath;
  }

  async start(): Promise<void> {
    try {
      this.proc = spawn(
        'ffmpeg',
        [
          '-hide_banner',
          '-loglevel', 'error',
          '-y',
          '-f', 'h264',
          '-r', String(this.fps),
          '-i', 'pipe:0',
          '-c', 'copy',
          '-movflags', '+faststart',
          this.outPath,
        ],
        { stdio: ['pipe', 'ignore', 'inherit'] },
      );
      await new Promise<void>((resolve, reject) => {
        this.proc!.once('spawn', resolve);
        this.proc!.once('error', reject);
      });
    } catch {
      // ffmpeg missing: stream straight to a raw .h264 file (no RAM buffer).
      this.proc = null;
      this.finalPath = this.outPath.replace(/\.mp4$/, '') + '.h264';
      this.fallback = createWriteStream(this.finalPath);
    }
  }

  async write(chunk: Buffer): Promise<void> {
    if (this.finished) return;
    const sink = this.proc?.stdin ?? this.fallback;
    if (!sink) return;
    if (!sink.write(chunk)) await once(sink, 'drain');
  }

  /** Returns the final output path (.mp4, or .h264 when ffmpeg is missing). Idempotent. */
  async finish(): Promise<string> {
    if (this.finished) return this.finalPath;
    this.finished = true;
    if (this.proc?.stdin) {
      this.proc.stdin.end();
      const [code] = (await once(this.proc, 'close')) as [number];
      if (code !== 0) throw new Error(`ffmpeg exited with code ${code}`);
      return this.finalPath;
    }
    if (this.fallback) {
      this.fallback.end();
      await once(this.fallback, 'finish');
    }
    return this.finalPath;
  }
}
