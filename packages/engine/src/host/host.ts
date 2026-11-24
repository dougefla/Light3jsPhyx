import * as THREE from 'three';
import { Env } from '../env.js';
import { createCamera, frameObject } from '../camera.js';
import { CaptureRenderer, type CaptureSize, type PendingRead } from '../passes/capture.js';
import { Outliner } from '../passes/outline.js';
import { EnvRecorder } from './record.js';
import { CHANNEL, allocBinaryFrame, encodeBinaryFrame, type RpcRequest } from '../protocol/shared.js';
import type { SpawnOptions } from '../physics/world.js';

export type BinarySender = (requestId: number, channel: number, payload: Uint8Array) => void;
/** Sends an already-framed binary message (header + payload, see protocol). */
export type FrameSender = (frame: Uint8Array<ArrayBuffer>) => void;

interface EnvSlot {
  env: Env;
  camera: THREE.PerspectiveCamera;
  size: CaptureSize;
  recorder: EnvRecorder | null;
  recordFps: number;
  recordOutline: boolean;
  simTime: number;
  nextFrameTime: number;
  outliner: Outliner | null;
}

interface CameraParams {
  pos?: [number, number, number];
  lookat?: [number, number, number];
  fov?: number;
  width?: number;
  height?: number;
  near?: number;
  far?: number;
}

/**
 * Hosts E isolated envs inside one page/renderer and executes protocol
 * requests against them. Headless-only concurrency lives here; the
 * interactive WebUI never creates more than one env.
 */
export class Host {
  private readonly slots = new Map<number, EnvSlot>();
  private nextEnvId = 0;
  readonly capture: CaptureRenderer;

  constructor(readonly renderer: THREE.WebGLRenderer) {
    this.capture = new CaptureRenderer(renderer);
  }

  private slot(envId: unknown): EnvSlot {
    const slot = this.slots.get(Number(envId));
    if (!slot) throw new Error(`No env ${String(envId)} on this page`);
    return slot;
  }

  private envIds(param: unknown): number[] {
    if (param === 'all' || param === undefined) return [...this.slots.keys()];
    if (Array.isArray(param)) return param.map(Number);
    return [Number(param)];
  }

  /**
   * Execute one request. `release` (optional) is invoked as soon as the
   * request has finished touching shared state and is only waiting on the
   * GPU — the caller may then start the next request while this one's
   * readback is still in flight. Requests that never call it are released
   * by the caller on completion.
   */
  async handle(req: RpcRequest, sendFrame: FrameSender, release?: () => void): Promise<unknown> {
    const p = req.params ?? {};
    const sendBinary: BinarySender = (id, channel, payload) =>
      sendFrame(encodeBinaryFrame(id, channel, payload));
    switch (req.method) {
      case 'ping':
        return { ok: true, envs: this.slots.size };

      case 'create_envs': {
        const created: number[] = [];
        for (let i = 0; i < Number(p.count ?? 1); i++) {
          const env = new Env();
          env.scene.background = new THREE.Color(0xffffff);
          // physics=false: pure kinematic/render env — set_joint applies
          // instantly, no Rapier colliders are built per load_urdf, and
          // step()/contacts() are no-ops. Default stays physics on.
          // self_collision=true additionally reports link–link contacts
          // (see PhysicsOptions.selfCollision); off by default for speed.
          if (p.physics !== false) {
            await env.enablePhysics({ selfCollision: p.self_collision === true });
          }
          const camera = createCamera(50, 4 / 3);
          camera.position.set(2, -2, 1.5);
          camera.lookAt(0, 0, 0.5);
          const id = this.nextEnvId++;
          this.slots.set(id, {
            env,
            camera,
            size: { width: 640, height: 480 },
            recorder: null,
            recordFps: 30,
            recordOutline: false,
            simTime: 0,
            nextFrameTime: 0,
            outliner: null,
          });
          created.push(id);
        }
        return { envs: created };
      }

      case 'destroy_envs': {
        for (const id of this.envIds(p.env)) {
          const slot = this.slots.get(id);
          if (!slot) continue;
          slot.outliner?.dispose();
          slot.env.dispose();
          this.slots.delete(id);
        }
        return { ok: true };
      }

      case 'load_urdf': {
        const slot = this.slot(p.env);
        const num = (v: unknown) => (v === undefined || v === null ? undefined : Number(v));
        const robot = await slot.env.loadURDF(String(p.url), {
          name: p.name ? String(p.name) : undefined,
          pos: p.pos as [number, number, number] | undefined,
          yaw: num(p.yaw),
          physics: {
            dynamic: p.dynamic === true,
            stiffness: num(p.stiffness),
            damping: num(p.damping),
            density: num(p.density),
            motorModel: p.motor_model === 'force' ? 'force' : undefined,
          },
        });
        return {
          name: robot.name,
          joints: slot.env.listJoints(robot.name),
          dynamic: slot.env.isDynamic(robot.name),
        };
      }

      case 'set_joint_drive': {
        const num = (v: unknown) => (v === undefined || v === null ? undefined : Number(v));
        this.slot(p.env).env.setJointDrive(
          String(p.name),
          { stiffness: num(p.stiffness), damping: num(p.damping) },
          p.robot ? String(p.robot) : undefined,
        );
        return { ok: true };
      }

      case 'list_joints':
        return this.slot(p.env).env.listJoints(p.robot ? String(p.robot) : undefined);

      case 'set_joint': {
        const slot = this.slot(p.env);
        slot.env.setJoint(String(p.name), Number(p.value), {
          immediate: Boolean(p.immediate),
          robotName: p.robot ? String(p.robot) : undefined,
        });
        return { ok: true };
      }

      case 'set_joints': {
        const slot = this.slot(p.env);
        slot.env.setJoints(p.values as Record<string, number>, {
          immediate: Boolean(p.immediate),
          robotName: p.robot ? String(p.robot) : undefined,
        });
        return { ok: true };
      }

      case 'get_state': {
        const slot = this.slot(p.env);
        return { ...slot.env.getState(), settled: slot.env.jointsSettled(), sim_time: slot.simTime };
      }

      case 'spawn':
        return { name: this.slot(p.env).env.spawn(p as unknown as SpawnOptions) };

      case 'remove_object':
        return { removed: this.slot(p.env).env.removeObject(String(p.name)) };

      case 'remove_robot':
        return { removed: this.slot(p.env).env.removeRobot(String(p.name)) };

      case 'add_ground': {
        this.slot(p.env).env.addGround(Number(p.z ?? 0), {
          visual: p.visual === undefined ? true : Boolean(p.visual),
        });
        return { ok: true };
      }

      case 'contacts':
        return { contacts: this.slot(p.env).env.contacts() };

      case 'self_contacts':
        return {
          contacts: this.slot(p.env).env.selfContacts(p.robot ? String(p.robot) : undefined),
        };

      case 'reset': {
        const slot = this.slot(p.env);
        for (const name of Object.keys(slot.env.getState().objects)) slot.env.removeObject(name);
        for (const robot of slot.env.robots.values()) slot.env.resetJoints(robot.name);
        slot.simTime = 0;
        slot.nextFrameTime = 0;
        return { ok: true };
      }

      case 'set_camera': {
        const slot = this.slot(p.env);
        this.applyCamera(slot, p as CameraParams);
        return { ok: true };
      }

      case 'frame_camera': {
        const slot = this.slot(p.env);
        const robot = slot.env.robot;
        if (!robot) throw new Error('No robot loaded to frame');
        const center = frameObject(slot.camera, robot, Number(p.fill ?? 0.7));
        slot.camera.updateMatrixWorld(true);
        return {
          pos: slot.camera.position.toArray(),
          lookat: center.toArray(),
          fov: slot.camera.fov,
        };
      }

      case 'seg_labels':
        return { labels: this.slot(p.env).env.seg.labelMap() };

      case 'render': {
        const slot = this.slot(p.env);
        const passes = (p.passes as string[] | undefined) ?? ['rgb'];
        if (p.camera) this.applyCamera(slot, p.camera as CameraParams);
        slot.env.scene.updateMatrixWorld(true);
        const outline = Boolean(p.outline);
        const { camera, size } = slot;
        const bytes = CaptureRenderer.byteLength(size);
        // Issue every pass back-to-back (synchronous: render + enqueue
        // readback, scene state restored), then wait on ONE fence. Frames are
        // allocated with their wire header so the readback lands in place.
        const frames: Uint8Array<ArrayBuffer>[] = [];
        const reads: PendingRead[] = [];
        const t0 = performance.now();
        for (const pass of passes) {
          if (pass === 'rgb') {
            const { frame, payload } = allocBinaryFrame(req.id, CHANNEL.rgb, bytes);
            reads.push(this.issueRGB(slot, outline, payload));
            frames.push(frame);
          } else if (pass === 'seg') {
            const { frame, payload } = allocBinaryFrame(req.id, CHANNEL.seg, bytes);
            reads.push(this.capture.issueSeg(slot.env.scene, camera, size, slot.env.seg, payload));
            frames.push(frame);
          } else if (pass === 'depth') {
            const { frame, payload } = allocBinaryFrame(req.id, CHANNEL.depth, bytes);
            reads.push(this.capture.issueDepth(slot.env.scene, camera, size, payload));
            frames.push(frame);
          } else {
            throw new Error(`Unknown pass: ${pass}`);
          }
        }
        // Everything below only waits on the GPU: let the next request in.
        const labels = passes.includes('seg') ? slot.env.seg.labelMap() : undefined;
        const t1 = performance.now();
        release?.();
        const t2 = await this.capture.resolve(reads);
        const t3 = performance.now();
        for (const frame of frames) sendFrame(frame);
        return {
          passes,
          width: size.width,
          height: size.height,
          seg_labels: labels,
          // Diagnostic (params.timing): ms spent issuing GL work, waiting on
          // the fence, and copying the pixels out.
          timing: p.timing
            ? { issue: t1 - t0, wait: t2 - t1, copy: t3 - t2, send: performance.now() - t3 }
            : undefined,
        };
      }

      case 'start_record': {
        const slot = this.slot(p.env);
        if (slot.recorder) throw new Error('Already recording');
        if (p.width || p.height) this.applyCamera(slot, p as CameraParams);
        const fps = Number(p.fps ?? 30);
        const recorder = new EnvRecorder((bytes) => sendBinary(req.id, CHANNEL.video, bytes));
        await recorder.start(slot.size.width, slot.size.height, fps);
        slot.recorder = recorder;
        slot.recordFps = fps;
        slot.recordOutline = Boolean(p.outline);
        slot.nextFrameTime = slot.simTime; // first frame captured immediately
        await this.captureFrameAt(slot, slot.simTime);
        slot.nextFrameTime = slot.simTime + 1 / fps;
        return { ok: true, fps, width: slot.size.width, height: slot.size.height };
      }

      case 'stop_record': {
        const slot = this.slot(p.env);
        if (!slot.recorder) throw new Error('Not recording');
        const frames = await slot.recorder.stop();
        slot.recorder = null;
        return { frames };
      }

      case 'step': {
        const ids = this.envIds(p.env);
        const dt = Number(p.dt ?? 1 / 60);
        const n = Number(p.n ?? 1);
        const slots = ids.map((id) => this.slot(id));
        for (let i = 0; i < n; i++) {
          for (const slot of slots) {
            // Synchronous fast path: only a recording env needs the async
            // frame-capture interleave (an await per env-step otherwise
            // costs a microtask turn ×E×n for nothing).
            if (slot.recorder) await this.stepEnv(slot, dt);
            else {
              slot.env.step(dt);
              slot.simTime += dt;
            }
          }
        }
        return { ok: true, steps: n };
      }

      default:
        throw new Error(`Unknown method: ${req.method}`);
    }
  }

  private applyCamera(slot: EnvSlot, cam: CameraParams): void {
    if (cam.width || cam.height) {
      slot.size = {
        width: Number(cam.width ?? slot.size.width),
        height: Number(cam.height ?? slot.size.height),
      };
    }
    if (cam.fov) slot.camera.fov = Number(cam.fov);
    slot.camera.aspect = slot.size.width / slot.size.height;
    if (cam.pos) slot.camera.position.set(...cam.pos);
    if (cam.lookat) slot.camera.lookAt(...cam.lookat);
    slot.camera.updateMatrixWorld(true);
    // Depth clip planes: the default 0.01/100 gives a 10^4 near:far ratio that
    // destroys z-buffer precision for scenes tens of metres from the camera,
    // causing z-fighting flicker (esp. object bases coplanar with the ground).
    // Honour explicit values; otherwise fit the planes tightly to the scene.
    if (cam.near !== undefined || cam.far !== undefined) {
      if (cam.near !== undefined) slot.camera.near = Math.max(1e-4, Number(cam.near));
      if (cam.far !== undefined) slot.camera.far = Number(cam.far);
    } else {
      this.autoFitClipPlanes(slot);
    }
    slot.camera.updateProjectionMatrix();
  }

  /** Fit near/far to the scene's bounding sphere so depth precision stays high. */
  private autoFitClipPlanes(slot: EnvSlot): void {
    const box = new THREE.Box3().setFromObject(slot.env.scene);
    if (box.isEmpty()) return; // nothing loaded yet — keep current planes
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const dist = slot.camera.position.distanceTo(sphere.center);
    const margin = sphere.radius * 0.05 + 0.01;
    const near = Math.max(1e-3, dist - sphere.radius - margin);
    const far = dist + sphere.radius + margin;
    slot.camera.near = near;
    slot.camera.far = Math.max(far, near + 1e-2);
  }

  /**
   * Advance one env by dt. When recording, physics is sub-divided at the frame
   * cadence so each captured frame reflects the true intermediate pose (not the
   * final pose replayed N times) and carries its own timestamp — otherwise a
   * step(dt=0.1) at 30fps would encode ~3 identical frames (issue #6).
   */
  private async stepEnv(slot: EnvSlot, dt: number): Promise<void> {
    if (!slot.recorder) {
      slot.env.step(dt);
      slot.simTime += dt;
      return;
    }
    const end = slot.simTime + dt;
    while (slot.nextFrameTime <= end + 1e-9) {
      const advance = slot.nextFrameTime - slot.simTime;
      if (advance > 1e-9) {
        slot.env.step(advance);
        slot.simTime = slot.nextFrameTime;
      }
      await this.captureFrameAt(slot, slot.nextFrameTime);
      slot.nextFrameTime += 1 / slot.recordFps;
    }
    if (end - slot.simTime > 1e-9) {
      slot.env.step(end - slot.simTime);
      slot.simTime = end;
    }
  }

  private async captureFrameAt(slot: EnvSlot, simTime: number): Promise<void> {
    if (!slot.recorder) return;
    slot.env.scene.updateMatrixWorld(true);
    const rgba = await this.renderRGB(slot, slot.recordOutline);
    await slot.recorder.captureFrame(rgba, slot.size.width, slot.size.height, Math.round(simTime * 1e6));
  }

  /**
   * RGB (beauty) pass, optionally with CAD outlines. Outlines are made
   * visible ONLY for this pass and hidden again immediately (before the
   * readback is even awaited), so the seg and depth passes never see them —
   * segmentation IDs and depth stay bit-exact regardless of the outline flag.
   */
  private issueRGB(slot: EnvSlot, outline: boolean, dst: Uint8Array): PendingRead {
    if (!outline) return this.capture.issueRGB(slot.env.scene, slot.camera, slot.size, dst);
    if (!slot.outliner) slot.outliner = new Outliner();
    slot.outliner.sync(slot.env.scene);
    slot.outliner.setVisible(true);
    try {
      return this.capture.issueRGB(slot.env.scene, slot.camera, slot.size, dst);
    } finally {
      slot.outliner.setVisible(false);
    }
  }

  private async renderRGB(slot: EnvSlot, outline: boolean): Promise<Uint8Array> {
    const dst = new Uint8Array(CaptureRenderer.byteLength(slot.size));
    await this.capture.resolve([this.issueRGB(slot, outline, dst)]);
    return dst;
  }

  /** Draw the first env to the canvas (watch-along preview for headed runs). */
  renderPreview(): void {
    const slot = this.slots.values().next().value as EnvSlot | undefined;
    if (!slot) return;
    this.renderer.render(slot.env.scene, slot.camera);
  }

  dispose(): void {
    for (const slot of this.slots.values()) {
      slot.outliner?.dispose();
      slot.env.dispose();
    }
    this.slots.clear();
    this.capture.dispose();
  }
}
