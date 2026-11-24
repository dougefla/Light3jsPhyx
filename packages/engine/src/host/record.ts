/**
 * In-page H.264 recording via WebCodecs. Annex B chunks stream out through
 * onChunk; the server muxes them into mp4 with zero re-encode. Timestamps
 * are SIM time — wall clock is meaningless in stepped simulation.
 */
export class EnvRecorder {
  private encoder: VideoEncoder | null = null;
  private frames = 0;
  private error: Error | null = null;

  constructor(private readonly onChunk: (bytes: Uint8Array) => void) {}

  static async pickCodec(width: number, height: number, fps: number): Promise<string | null> {
    for (const codec of ['avc1.4d0028', 'avc1.42e028', 'avc1.640028']) {
      const { supported } = await VideoEncoder.isConfigSupported({
        codec,
        width,
        height,
        framerate: fps,
        avc: { format: 'annexb' },
      });
      if (supported) return codec;
    }
    return null;
  }

  async start(width: number, height: number, fps: number, bitrate = 6_000_000): Promise<void> {
    const codec = await EnvRecorder.pickCodec(width, height, fps);
    if (!codec) {
      throw new Error(
        'No H.264 encoder available in this browser build ' +
          '(open-source Chromium lacks proprietary codecs — use Chrome for Testing)',
      );
    }
    this.encoder = new VideoEncoder({
      output: (chunk) => {
        const bytes = new Uint8Array(chunk.byteLength);
        chunk.copyTo(bytes);
        this.onChunk(bytes);
      },
      error: (e) => {
        this.error = e as Error;
      },
    });
    this.encoder.configure({
      codec,
      width,
      height,
      framerate: fps,
      bitrate,
      avc: { format: 'annexb' },
      latencyMode: 'quality',
    });
  }

  /** rgba is top-down; timestamp in microseconds of sim time. */
  async captureFrame(rgba: Uint8Array, width: number, height: number, timestampUs: number): Promise<void> {
    if (!this.encoder) throw new Error('Recorder not started');
    if (this.error) throw this.error;
    while (this.encoder.encodeQueueSize > 8) {
      await new Promise<void>((resolve) =>
        this.encoder!.addEventListener('dequeue', () => resolve(), { once: true }),
      );
    }
    const frame = new VideoFrame(rgba as unknown as BufferSource, {
      format: 'RGBA',
      codedWidth: width,
      codedHeight: height,
      timestamp: timestampUs,
    });
    // Periodic keyframes keep the stream seekable after muxing.
    this.encoder.encode(frame, { keyFrame: this.frames % 60 === 0 });
    frame.close();
    this.frames++;
  }

  get frameCount(): number {
    return this.frames;
  }

  async stop(): Promise<number> {
    if (!this.encoder) return 0;
    await this.encoder.flush();
    this.encoder.close();
    this.encoder = null;
    if (this.error) throw this.error;
    return this.frames;
  }
}
