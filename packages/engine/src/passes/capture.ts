import * as THREE from 'three';
import type { SegRegistry } from './seg.js';

export interface CaptureSize {
  width: number;
  height: number;
}

export type PassName = 'rgb' | 'seg' | 'depth';

const DEPTH_VERT = /* glsl */ `
varying float vViewZ;
void main() {
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  vViewZ = -mv.z;
  gl_Position = projectionMatrix * mv;
}`;

/**
 * Metric view-space depth, bit-packed: the IEEE-754 float32 bits of the depth
 * are written little-endian into an RGBA8 target (R = low byte … A = high
 * byte). The readback is therefore a raw Float32 LE array once rows are
 * flipped — no float render target, no RGBA32F readback (4× the bytes) and no
 * per-pixel extraction loop in JS. k/255 → unorm8 conversion is exact, so the
 * floats round-trip bit-for-bit (the same contract the seg ID pass relies on).
 * Background clears to 0 → float 0.0 = "no hit".
 */
const DEPTH_FRAG = /* glsl */ `
precision highp float;
precision highp int;
varying float vViewZ;
layout(location = 0) out highp vec4 outColor;
void main() {
  uint bits = floatBitsToUint(vViewZ);
  outColor = vec4(
    float(bits & 255u),
    float((bits >> 8u) & 255u),
    float((bits >> 16u) & 255u),
    float(bits >> 24u)) / 255.0;
}`;

/** One issued readPixels-into-PBO whose bytes arrive after the next fence. */
export interface PendingRead {
  pbo: WebGLBuffer;
  /** Destination view; receives the raw bottom-up pixels, then gets flipped. */
  dst: Uint8Array;
  width: number;
  height: number;
}

/**
 * Async pixel readback with ONE GPU fence per batch.
 *
 * three's readRenderTargetPixelsAsync allocates a PBO, creates a fence and
 * polls it with a fixed 4 ms first sleep, per call. With three passes per env
 * and several envs per page, that per-call idle time — not the GPU — dominated
 * the capture path. Here every pass in a request is issued back-to-back into
 * pooled PBOs, one fence covers them all, and the poll starts unclamped.
 */
export class PixelReader {
  private readonly gl: WebGL2RenderingContext;
  private readonly pool: WebGLBuffer[] = [];
  private static readonly MAX_POOLED = 32;

  constructor(private readonly renderer: THREE.WebGLRenderer) {
    this.gl = renderer.getContext() as WebGL2RenderingContext;
  }

  /**
   * Enqueue a readback of rt (RGBA8, bottom-up) into a PBO. Synchronous: the
   * GL commands are ordered after whatever was rendered into rt, so the
   * target may be reused for the next env immediately.
   */
  issue(rt: THREE.WebGLRenderTarget, dst: Uint8Array): PendingRead {
    const gl = this.gl;
    const { width, height } = rt;
    if (dst.byteLength !== width * height * 4) {
      throw new Error(`readback buffer is ${dst.byteLength} bytes, target needs ${width * height * 4}`);
    }
    // For a multisampled target three resolves into __webglFramebuffer at the
    // end of render(); reading from it is exactly what three's own readback does.
    const fb = (this.renderer.properties.get(rt) as { __webglFramebuffer: WebGLFramebuffer }).__webglFramebuffer;
    // Bind through three's state cache so its notion of the bound framebuffer
    // stays coherent with what we did.
    this.renderer.state.bindFramebuffer(gl.FRAMEBUFFER, fb);
    const pbo = this.pool.pop() ?? gl.createBuffer();
    if (!pbo) throw new Error('createBuffer failed (context lost?)');
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, pbo);
    gl.bufferData(gl.PIXEL_PACK_BUFFER, dst.byteLength, gl.STREAM_READ);
    gl.readPixels(0, 0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, 0);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    return { pbo, dst, width, height };
  }

  /**
   * Fence the queue once, wait for it, then copy every pending read out of its
   * PBO (flipping rows to top-down). Resolves in issue order; returns the
   * timestamp (performance.now()) at which the fence was observed signaled.
   */
  async resolve(reads: PendingRead[]): Promise<number> {
    if (reads.length === 0) return performance.now();
    const gl = this.gl;
    const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0);
    if (!sync) throw new Error('fenceSync failed (context lost?)');
    gl.flush();
    try {
      await waitSync(gl, sync);
    } finally {
      gl.deleteSync(sync);
    }
    const signaled = performance.now();
    for (const r of reads) {
      gl.bindBuffer(gl.PIXEL_PACK_BUFFER, r.pbo);
      gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, r.dst);
      if (this.pool.length < PixelReader.MAX_POOLED) this.pool.push(r.pbo);
      else gl.deleteBuffer(r.pbo);
      flipRowsInPlace(r.dst, r.width, r.height);
    }
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    return signaled;
  }

  dispose(): void {
    for (const pbo of this.pool) this.gl.deleteBuffer(pbo);
    this.pool.length = 0;
  }
}

/**
 * Poll a fence. WebGL2 never reports a sync as signaled inside the task that
 * created it, so the first probe is after a yield. Chromium runs setTimeout(0)
 * unclamped for the first few nesting levels (then 4 ms), which gives a natural
 * fast-then-backoff curve for GPU work that typically completes within 1–3 ms.
 */
function waitSync(gl: WebGL2RenderingContext, sync: WebGLSync): Promise<void> {
  return new Promise((resolve, reject) => {
    let probes = 0;
    const probe = () => {
      const r = gl.clientWaitSync(sync, 0, 0);
      if (r === gl.ALREADY_SIGNALED || r === gl.CONDITION_SATISFIED) resolve();
      else if (r === gl.WAIT_FAILED) reject(new Error('clientWaitSync failed'));
      else setTimeout(probe, probes++ < 4 ? 0 : 2);
    };
    setTimeout(probe, 0);
  });
}

/**
 * Renders the three capture passes into render targets and reads pixels
 * back (render targets bypass tone mapping and output color-space transforms
 * in three.js, which is what makes the ID pass bit-exact). All outputs are
 * returned top-to-bottom (the GL bottom-up readback is flipped here).
 *
 * Every pass is split into a synchronous `issue*` (render + enqueue readback;
 * all scene state it touched is restored before it returns) and a shared
 * `resolve` that waits on one fence for all of them. Callers may issue the
 * passes of several requests before resolving — the GPU queue keeps them in
 * order — which is what lets a page pipeline env renders instead of idling on
 * every readback.
 */
export class CaptureRenderer {
  private readonly targets = new Map<string, THREE.WebGLRenderTarget>();
  readonly reader: PixelReader;
  private readonly depthMaterial = new THREE.ShaderMaterial({
    vertexShader: DEPTH_VERT,
    fragmentShader: DEPTH_FRAG,
    glslVersion: THREE.GLSL3,
  });

  constructor(readonly renderer: THREE.WebGLRenderer) {
    this.reader = new PixelReader(renderer);
  }

  /** Cap on distinct render targets kept alive (3 passes × a few sizes). */
  private static readonly MAX_TARGETS = 12;

  private target(kind: PassName, { width, height }: CaptureSize): THREE.WebGLRenderTarget {
    const key = `${kind}:${width}x${height}`;
    const existing = this.targets.get(key);
    if (existing) {
      // LRU touch: re-insert so recently-used targets survive eviction.
      this.targets.delete(key);
      this.targets.set(key, existing);
      return existing;
    }
    // Evict the least-recently-used target before allocating a new one, so a
    // job that varies resolution across episodes can't grow GPU memory without
    // bound (targets otherwise lived until the whole page was disposed).
    while (this.targets.size >= CaptureRenderer.MAX_TARGETS) {
      const oldestKey = this.targets.keys().next().value as string | undefined;
      if (oldestKey === undefined) break;
      this.targets.get(oldestKey)!.dispose();
      this.targets.delete(oldestKey);
    }
    const rt = new THREE.WebGLRenderTarget(width, height, {
      format: THREE.RGBAFormat,
      type: THREE.UnsignedByteType,
      minFilter: THREE.NearestFilter,
      magFilter: THREE.NearestFilter,
      generateMipmaps: false,
      depthBuffer: true,
      // MSAA on the beauty pass only: without it the offscreen RGB target
      // has hard, unantialiased edges that crawl frame-to-frame as the
      // camera moves — the "flicker" seen in recordings (the interactive
      // canvas is antialiased, so it never showed there). The seg and depth
      // passes MUST stay at samples:0 — multisampling would blend per-link
      // ID colors and packed depth bits, breaking bit-exactness.
      samples: kind === 'rgb' ? 4 : 0,
    });
    // Beauty readbacks should be sRGB-encoded like a normal canvas.
    if (kind === 'rgb') rt.texture.colorSpace = THREE.SRGBColorSpace;
    this.targets.set(key, rt);
    return rt;
  }

  private withTarget<T>(rt: THREE.WebGLRenderTarget, fn: () => T): T {
    const prev = this.renderer.getRenderTarget();
    this.renderer.setRenderTarget(rt);
    try {
      return fn();
    } finally {
      this.renderer.setRenderTarget(prev);
    }
  }

  /** Bytes one pass of this size produces (all passes are RGBA8 on the wire). */
  static byteLength({ width, height }: CaptureSize): number {
    return width * height * 4;
  }

  /** Beauty pass: render + enqueue readback into dst (width*height*4 bytes). */
  issueRGB(scene: THREE.Scene, camera: THREE.Camera, size: CaptureSize, dst: Uint8Array): PendingRead {
    const rt = this.target('rgb', size);
    this.withTarget(rt, () => this.renderer.render(scene, camera));
    return this.reader.issue(rt, dst);
  }

  /** ID pass: per-mesh flat-color materials; restored before returning. */
  issueSeg(
    scene: THREE.Scene,
    camera: THREE.Camera,
    size: CaptureSize,
    seg: SegRegistry,
    dst: Uint8Array,
  ): PendingRead {
    const rt = this.target('seg', size);
    const restore: Array<() => void> = [];
    scene.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh) return;
      const prev = mesh.material;
      mesh.material = seg.idMaterialFor(mesh);
      restore.push(() => (mesh.material = prev));
    });
    const prevBackground = scene.background;
    scene.background = null;
    const prevClear = this.renderer.getClearColor(new THREE.Color());
    const prevAlpha = this.renderer.getClearAlpha();
    this.renderer.setClearColor(0x000000, 1); // id 0 = background
    try {
      this.withTarget(rt, () => this.renderer.render(scene, camera));
      return this.reader.issue(rt, dst);
    } finally {
      this.renderer.setClearColor(prevClear, prevAlpha);
      scene.background = prevBackground;
      for (const fn of restore) fn();
    }
  }

  /** Depth pass: bit-packed float32 metres (see DEPTH_FRAG). */
  issueDepth(scene: THREE.Scene, camera: THREE.Camera, size: CaptureSize, dst: Uint8Array): PendingRead {
    const rt = this.target('depth', size);
    const prevOverride = scene.overrideMaterial;
    const prevBackground = scene.background;
    const prevClear = this.renderer.getClearColor(new THREE.Color());
    const prevAlpha = this.renderer.getClearAlpha();
    scene.overrideMaterial = this.depthMaterial;
    scene.background = null;
    this.renderer.setClearColor(0x000000, 0); // all-zero bytes = float 0 = no hit
    try {
      this.withTarget(rt, () => this.renderer.render(scene, camera));
      return this.reader.issue(rt, dst);
    } finally {
      scene.overrideMaterial = prevOverride;
      scene.background = prevBackground;
      this.renderer.setClearColor(prevClear, prevAlpha);
    }
  }

  /** Wait for every issued read; buffers are filled (top-down) on return. */
  resolve(reads: PendingRead[]): Promise<number> {
    return this.reader.resolve(reads);
  }

  // ---- one-shot conveniences (issue + resolve) -----------------------------

  async renderRGB(scene: THREE.Scene, camera: THREE.Camera, size: CaptureSize): Promise<Uint8Array> {
    const dst = new Uint8Array(CaptureRenderer.byteLength(size));
    await this.resolve([this.issueRGB(scene, camera, size, dst)]);
    return dst;
  }

  async renderSeg(
    scene: THREE.Scene,
    camera: THREE.Camera,
    size: CaptureSize,
    seg: SegRegistry,
  ): Promise<Uint8Array> {
    const dst = new Uint8Array(CaptureRenderer.byteLength(size));
    await this.resolve([this.issueSeg(scene, camera, size, seg, dst)]);
    return dst;
  }

  async renderDepth(scene: THREE.Scene, camera: THREE.Camera, size: CaptureSize): Promise<Float32Array> {
    const dst = new Uint8Array(CaptureRenderer.byteLength(size));
    await this.resolve([this.issueDepth(scene, camera, size, dst)]);
    return new Float32Array(dst.buffer, dst.byteOffset, size.width * size.height);
  }

  dispose(): void {
    for (const rt of this.targets.values()) rt.dispose();
    this.targets.clear();
    this.depthMaterial.dispose();
    this.reader.dispose();
  }
}

export function flipRowsInPlace(buf: Uint8Array, width: number, height: number): Uint8Array {
  const stride = width * 4;
  const tmp = new Uint8Array(stride);
  for (let y = 0; y < height >> 1; y++) {
    const a = y * stride;
    const b = (height - 1 - y) * stride;
    tmp.set(buf.subarray(a, a + stride));
    buf.copyWithin(a, b, b + stride);
    buf.set(tmp, b);
  }
  return buf;
}
