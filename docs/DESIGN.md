# light-3js-phyx Implementation Plan

> An extremely lightweight rendering and physics simulation engine for articulated objects, built on three.js.
> Status: design phase (v1 draft). This document is the single source of truth for the implementation, and any deviation during implementation must be written back into it.

## 1. Goals and non-goals

**Goals**

- Import URDF articulated objects (link/joint tree, OBJ meshes + MTL materials + diffuse textures), with normalized ArtVIP data as the reference use case (`scene.urdf` + `assets/part_*.obj/.mtl` + `textures/*.png`, prismatic/revolute/fixed joints).
- Rendering: global uniform lighting by default (pure AmbientLight), with textures and material colors displayed correctly.
- Physics: gravity + collision detection. Articulated links are kinematically driven (the SDK sets joint values and the engine does FK), and free rigid bodies in the scene are dynamic bodies (subject to gravity and colliding with the articulated object).
- WebUI: interactive viewing in the browser (orbit camera, joint sliders, object spawning, play/pause/single step).
- Headless: runs without a window, with deterministic stepping, and exports RGB images / per-link segmentation maps / depth maps / mp4 videos.
- Agent-friendly Python SDK: synchronous-style API, numpy images, usable with a single in-process `launch()`.
- **Concurrency (headless only)**: 50 concurrent instances on one machine is a first-class requirement, aimed at data production. Simulation stepping, video recording and rgb/segmentation/depth capture are all offline and step-driven, with **no real-time rendering requirement**. The SDK provides a vectorized multi-environment API. Browser access to the WebUI needs a single instance only and carries no concurrency complexity.
- Extremely lightweight: the core engine uses no framework (vanilla TS), the runtime dependencies are only essentials at the level of three, rapier and ws, and the WebUI does not use React.

**Non-goals (v1)**

- Fully dynamic joints (force-driven joints, gravity pulling a drawer open), motor control, fine-tuned friction modeling.
- Shadows, a full PBR lighting stack, ray-tracing-grade image quality.
- Multi-robot scene orchestration, sensor simulation (lidar and so on).
- Full URDF specification support (only the subset described below is supported).

## 2. Overall architecture

A single TypeScript engine codebase runs in two kinds of hosts, and there is only one control protocol:

```
                      ┌────────────────────────────────────────────────┐
                      │  Node server (packages/server)                 │
  Python SDK ────WS──▶│  · WebSocket control protocol (JSON + binary)  │
  (sdk/python)        │  · static hosting of the WebUI                 │
                      │  · headless: Playwright launches Chromium      │
  Browser ──HTTP/WS──▶│  · ffmpeg encodes mp4                          │
                      └───────────────┬────────────────────────────────┘
                                      │ forwarded over the same protocol
                      ┌───────────────▼────────────────────────────────┐
                      │  Engine (packages/engine, runs in the page)    │
                      │  · three.js scene / camera / render passes     │
                      │  · URDF parsing + FK joint tree                │
                      │  · Rapier physics (kinematic + dynamic)        │
                      │  · three passes: beauty / seg-id / depth       │
                      │  · frame capture                               │
                      └────────────────────────────────────────────────┘
```

Key property: **WebUI mode and headless mode are the same page and the same rendering path**. The only difference is whether Chromium has a window. An agent can therefore attach to a session that is open in a browser and drive it live (watching while it runs), or produce data in batches fully headless, and both modes produce identical pixels.

Concurrency model (**headless only**, see §5.5): **one page can host several independent environments (envs)** (N Scenes + N Rapier Worlds, stepped round-robin, sharing loaded assets), and the server manages a three-level pool of `B browser instances × P pages per instance × E envs per page`. Every protocol method carries `env` addressing and supports batch operations. 50 instances = for example 8×1×6+2. The topology is configurable and is tuned per machine by the benchmark script. **Interactive sessions are fixed to a single instance** (one page, one env). They do not take part in the pool, are not bound by throughput constraints, and keep real-time playback and nicely anti-aliased images.

## 3. Module design

### 3.1 packages/engine (browser-side core)

- **Scene**: `THREE.Scene`, with world coordinates following the **Z-up convention of URDF** (`camera.up = (0,0,1)`, gravity `(0,0,-9.81)`), which avoids the coordinate-conversion confusion of a -90° root rotation on the assets. The ground is an optional plane (z=0 by default, can be turned off).
- **Lighting**: by default only `AmbientLight(0xffffff, 1.0)` + `NoToneMapping`, that is, "global uniform lighting", where the texture color is the color you see. Hemisphere/directional switches are reserved (off by default).
- **URDF loading** (see §5.1): `urdf-loader` (NASA JPL, version pinned) + a custom `loadMeshCb` that loads OBJ/MTL/textures (three's `OBJLoader + MTLLoader`). Paths are resolved relative to the URDF file, and loaded meshes are cached by path. `parseCollision: true` also extracts the collision subtree to feed physics.
- **Joint tree / FK**: uses the `URDFRobot` of urdf-loader directly. `robot.setJointValue(name, v)` comes with the revolute/prismatic transforms and limit clamping, and `updateMatrixWorld` of the three scene graph completes FK, so no joint math has to be written.
- **Physics** (see §5.2): Rapier. Each link → a `kinematicPositionBased` rigid body + a trimesh collider (collision mesh, `FIX_INTERNAL_EDGES`). Before every physics step the FK pose is written with `setNextKinematicTranslation/Rotation` (never with `setTranslation`, because only the former lets Rapier derive a virtual velocity, which is what makes pushing and carrying dynamic bodies work correctly). Free objects: a `dynamic` rigid body + a primitive shape (box/sphere/cylinder) or a convex hull (**never** a trimesh on a dynamic body), with CCD enabled for small objects. The fixed timestep is `1/60` and `step(dt)` accumulates internally at the fixed timestep. Large joint commands are automatically interpolated into several small steps to prevent tunneling.
- **Render passes** (see §5.4):
  - *beauty*: regular materials, outputs RGB.
  - *seg*: each mesh is swapped to a flat color material, link i → a 24-bit ID color, with AA/toneMapping/color management off, rendered to a `WebGLRenderTarget` and read back exactly with `readRenderTargetPixels`.
  - *depth*: outputs metric view-space depth (float render target, WebGL2).
- **Frame capture**: always goes through render-to-target + readPixels (no dependence on `preserveDrawingBuffer`), is assembled into a `Uint8Array` inside the page, and is sent out over WS binary or a Playwright binding. Batch scenarios use `readRenderTargetPixelsAsync` (fence/PBO) pipelined across envs, so that right after the readback of env i is issued, env i+1 is stepped/rendered.
- **Multi-environment hosting** (used by headless pages only, while the WebUI shell always creates 1 env): one engine instance can create E mutually isolated envs (each env has one Scene + one Rapier World + its own camera/recording state), stepped round-robin. Assets are cached by URL. BufferGeometry/Texture are shared across envs, while Mesh nodes and any Material that needs per-env changes must be cloned (the single-parent rule of Object3D, and changes to a shared material would leak across scenes). The renderer state (clearColor/viewport/renderTarget) is reset before rendering every time the env switches. The lighting configuration is kept identical across envs to avoid recompiling shader program variants. When an env is destroyed, `world.free()` is called explicitly and the world is pooled for reuse (wasm linear memory only grows and never shrinks).
- **Step driving**: always driven by WS commands, never by `requestAnimationFrame` (background or occluded pages get throttled). Real-time playback in the WebUI also goes through "a timer sending step commands".

### 3.2 packages/server (Node)

- Static hosting of the WebUI build output, and the control protocol implemented on `/ws`.
- Session management: an `interactive` session = a browser page connecting in on its own initiative, and a `headless` session = the server launching Chromium with Playwright to load the same page. From the SDK's point of view the two are indistinguishable.
- **Instance pool** (see §5.5): headless mode is orchestrated as a three-level `B×P×E` pool (B browser instances = unit of parallelism / own GPU process / own crash domain, P pages per instance, E envs per page). Envs are addressed globally by `env_id` and routed to the corresponding page. Long-running jobs recycle pages/browser instances periodically per episode (memory creep in headless Chromium is an operational reality, ~200MB can grow to >1GB). The unified launch flags add `--disable-background-timer-throttling --disable-renderer-backgrounding`.
- Frames/video: the **default video path** is in-page encoding with the WebCodecs `VideoEncoder` (see §5.5), where the server only receives the EncodedVideoChunk stream and muxes it into mp4 (`ffmpeg -c copy` or an mp4 muxer, zero transcoding). The **raw pixel path** (per-frame PNG/npy stills, lossless evaluation data) is kept and is subject to backpressure and a global concurrency quota. Image export encodes PNG on the server side (sharp). When ffmpeg is absent, video degrades to a PNG sequence.
- CLI: `light-3js-phyx serve [--port 8323] [--headless] [--pool B,P,E]`.

### 3.3 packages/webui

- vanilla TS + Vite. `lil-gui` provides the joint sliders/object spawning/record button, `OrbitControls` the orbit camera, and `Stats` the frame rate. No React and no UI framework.
- Opening the page → automatically connects to the server WS and registers as an interactive session. Dragging a URDF folder in for purely local offline viewing (without the server) is also supported.
- **Fixed single instance**: one page, one env, with no env switching or pool management UI. The canvas may use `antialias:true` (interactive image quality comes first, and the constraint of pixel-aligned three channels belongs only to headless capture, §5.4).

### 3.4 sdk/python (the `lightphyx` package)

What agent-friendly means: synchronous blocking calls, complete types, images as numpy, errors as exceptions, and a task running end to end in 10 lines.

```python
import lightphyx as lp

sim = lp.launch(headless=True)              # starts server+Chromium automatically, or use lp.connect("ws://...")
obj = sim.load_urdf("/path/to/scene.urdf")
print(obj.joints)                            # {name: Joint(type, lower, upper, value)}
sim.add_ground(z=0.0)
ball = sim.spawn_sphere(radius=0.03, pos=(0.19, -0.1, 1.1), mass=0.05)

cam = sim.camera(pos=(1.5, -1.5, 1.5), lookat=(0, 0, 0.6), fov=50, size=(1280, 720))
with sim.record("open_drawer.mp4", fps=30):
    for q in lp.linspace(0, -0.3, steps=90):
        obj.set_joint("joint_0", float(q))
        sim.step(1 / 30)

rgb   = cam.rgb()                            # np.uint8  (H, W, 3)
seg   = cam.seg()                            # np.int32  (H, W)  value = link id, cam.seg_labels() → {id: link_name}
depth = cam.depth()                          # np.float32(H, W)  meters
hits  = sim.contacts()                       # current contact pairs [(body_a, body_b), ...]
sim.reset()                                  # save_state/restore_state deferred to v1.1 (see the note in §3.5)
```

**Vectorized multi-environment API** (the first-class interface for 50-way concurrency, in the style of gym.vector):

```python
import lightphyx as lp

vec = lp.launch_vec(num_envs=50, headless=True, pool=(8, 1, 7))   # B,P,E may be omitted to use the defaults
vec.load_urdf("/path/to/scene.urdf")                               # broadcast to all envs (assets load once per page)
vec.set_joints({"joint_0": qs})                                    # qs: shape (50,) per-env values
vec.step(1 / 30)                                                   # all envs step in parallel (truly parallel across pages)
rgbs = vec.render("rgb", size=(640, 480))                          # np.uint8 (50, H, W, 3)
segs = vec.render("seg")                                           # np.int32 (50, H, W)
env7 = vec[7]                                                      # single-env handle, same interface as sim (fine-grained per-env control)
vec.reset(indices=[3, 12])                                         # partial reset
```

Implementation: `websockets` (or `websocket-client`) + `numpy` + `pillow`, with no other heavy dependency. `launch()` is responsible for finding node/npx, starting the server and waiting until it is ready. Vector calls fan out to the pages on the server side and are aggregated on return, so the SDK side still sees a single round trip.

### 3.5 Control protocol (WS)

- Control frames: JSON-RPC style `{id, method, params}` → `{id, result | error}`. **Every method that acts on a scene takes an `env` parameter** (a single value, a list or `"all"`), which defaults to env 0 when omitted. Single-instance use carries zero burden, and 50 instances use the same protocol.
- Binary frames: `[u32 request id][u16 env id][u8 channel: rgb|seg|depth|video][payload raw]`, which avoids the base64 overhead.
- Method set (implemented in v1): environment management `create_envs(n), destroy_envs, pool_stats`. Scene `load_urdf, list_joints, set_joint(s), get_state, spawn(shape…), remove_object, add_ground, reset`. Simulation `step(dt, n)` (the server fans out in parallel to envs across pages and aggregates behind a barrier). Rendering `set_camera, frame_camera, render(passes[, save]), seg_labels`. Recording `start_record, stop_record`. Queries `contacts, self_contacts` (the latter requires `create_envs(self_collision=true)`, where the physics layer enables the Rapier KINEMATIC_KINEMATIC collision type and uses hooks to filter links adjacent through a joint). Dynamic links `load_urdf(dynamic=true)` + `set_joint_drive` (links are dynamic rigid bodies + impulse joints with PD motors, fixed joints are merged, the root link is fixed, and joint values are solved back from the rigid-body poses into the scene graph, see `physics/articulation.ts`). Batch semantics: `env` may be a single value, a list or `"all"`. **Deferred to v1.1**: `save_state/restore_state` (handle remapping for Rapier snapshots is highly complex, and v1 covers the need with `reset` + deterministic replay) and `set_pose`.
- The protocol is the stable layer. The WebUI, the Python SDK and any future agent tool (including an MCP wrapper) depend only on it.

## 4. Directory structure

```
light-3js-phyx/
├── package.json              # npm workspaces
├── packages/
│   ├── engine/               # three+rapier core (pure browser library, no UI)
│   │   └── src/{urdf,scene,physics,passes,capture,protocol}/
│   ├── server/               # ws service, playwright headless, ffmpeg, CLI
│   └── webui/                # vite app (thin shell: engine + lil-gui)
├── sdk/python/lightphyx/     # Python SDK (pyproject, pip install -e works)
├── examples/                 # 2-3 end-to-end examples each for ts and python
├── tests/                    # integration test assets (a small ArtVIP sample checked in)
├── docs/DESIGN.md            # this document
└── .github/workflows/ci.yml
```

## 5. Key technical decisions

> The conclusions in this section were verified by a dedicated survey of each upstream library (npm registry / source code / official documentation) in 2026-08, and are not from memory.

### 5.1 URDF parsing: use `urdf-loader` with a custom OBJ/MTL loading callback

**Decision**: use `urdf-loader` (NASA JPL / gkjohnson, v0.13.1, released 2026-07-08, peer dependency three ≥0.152, zero runtime dependencies, 305KB). The "in-house parser of ~300 lines" from the design draft was rejected by the survey. An in-house parser would have to rewrite rpy→quaternion, the per-joint-type transforms, limit clamping and other code that urdf-loader has been fixing for 8 years, while the OBJ/MTL loading callback has to be written in both options, so the net saving is ≈0 and the risk is higher.

Key points:

- The default mesh loader only understands STL/DAE, so a `loadMeshCb` must be provided. The correct approach (the community pattern of "guessing the .mtl path" is known to be fragile): fetch the OBJ text → read its `mtllib` line → load with `MTLLoader` and point `setResourcePath()` at the OBJ directory (which guarantees that relative `map_Kd` paths resolve) → `objLoader.setMaterials(mtl).parse(text)`. Cache by path.
- MTLLoader silently ignores the PBR extension fields (`Pm/Pr/Pc/Pcr`) (they fall through the switch default with no warning), so the ArtVIP MTL files work as they are. It produces `MeshPhongMaterial`, which behaves correctly under pure AmbientLight.
- Textures need `texture.colorSpace = SRGBColorSpace` confirmed or set, otherwise colors look washed out.
- The URDF `<material>` only serves as a fallback for meshes without a material and does not override MTL textures.
- `parseCollision: true` yields the `URDFCollider` subtree. The geometry extraction → Rapier collider still has to be written.
- 0.x semantic versioning, so **pin the version number**. The combination of `<mesh scale>` + texture paths is the most likely place for integration bugs and is covered by unit tests.

### 5.2 Rapier physics: kinematicPositionBased + trimesh (links) / primitive shapes (dynamic)

**Decision**: `@dimforge/rapier3d-compat` 0.19.3 (single-file ESM, wasm embedded as base64, zero Vite configuration, at the cost of only ~200KB more transfer than the non-compat build, and it needs `await RAPIER.init()`). The official documentation confirms that a kinematic body driven through `setNextKinematic*` derives its velocity from the pose difference and correctly pushes dynamic bodies and carries them by friction (the drawer-carries-ball scenario works out of the box, since `KINEMATIC_DYNAMIC` is in the default active collision types).

Key points:

- Link colliders: `ColliderDesc.trimesh(v, i, TriMeshFlags.FIX_INTERNAL_EDGES)`. Concave furniture must use a trimesh (a convex hull would fill the drawer cavity solid), and `FIX_INTERNAL_EDGES` suppresses ghost collisions when a ball rolls over the internal edges of the drawer bottom.
- **Collision geometry is taken from `<visual>` by default** (established during the M2 implementation, configurable with `collisionFrom: 'visual' | 'collision'`). Measurements show that every `<collision>` origin in ArtVIP is `0 0 0` while the `<visual>` origin carries the real offset (same OBJ, different placement), which means the collision poses in the source data are broken. Building from visual also naturally guarantees "physics = rendering" consistency. Datasets with proper simplified collision meshes can switch back to `'collision'`.
- **Joints have target-tracking semantics** (established during the M2 implementation, replacing the original "interpolation within a single step" scheme). `setJoint` sets a target, physics approaches it on every substep by `min(URDF velocity limit × dt, maxJointDelta)`, and the rendered pose and the physics pose always agree. `immediate: true` is an explicit teleport (for resets). The original scheme did not work for the common call pattern `setJoint(large jump); step(small dt)` (too few substeps within a single call, the back wall was measured to jump past the ball in one step, and CCD could not save it either). Rate-limit semantics eliminate tunneling by construction and directly use the velocity field that URDF already provides.
- Dynamic objects: primitive shapes first, and `convexHull` for arbitrary meshes. **The official advice is against trimeshes on dynamic bodies** (zero thickness with no interior, so they get stuck). Small objects use `setCcdEnabled(true)` + optionally `setSoftCcdPrediction`.
- Collision events: `collider.setActiveEvents(ActiveEvents.COLLISION_EVENTS)` + `EventQueue(true)` passed to `world.step(eventQueue)`, then `drainCollisionEvents((h1,h2,started)=>…)`. Contact details come from `world.contactPair()`.
- Known limits: the force exerted by a kinematic body is unbounded, so a dynamic object squeezed between a kinematic link and static geometry is ejected violently. Engine-level countermeasures: limit the per-step joint increment (≤ half the thinnest wall thickness), interpolate large commands automatically, and optionally stop the joint when an obstruction is detected. The sleep wake-up on kinematic contact had a bug fixed only in 0.19.2, so a manual `wakeUp()` fallback + a regression test (ball asleep in the drawer → pull it open) are kept.
- Determinism: same version + same initial conditions + fixed timestep → bit-identical on a single platform (cross-platform results are affected by JS `Math.sin/cos`, which is acceptable).

### 5.3 Headless capture pipeline: Playwright new headless + in-page readPixels + binary WS → ffmpeg

**Decision**: Playwright (1.62.x, version pinned) with `channel: 'chromium'` (the new headless, the same rendering stack as headed mode). `page.screenshot`/CDP screencast/`toDataURL` are **not used** (all of them go through JSON/base64 and are slow and non-deterministic). Frames are read inside the page via render-to-target + `readRenderTargetPixels`, and the raw RGBA is sent to the server over a **binary WebSocket** (hundreds of MB/s on localhost, so the bottleneck is rendering and not transport), one frame per physics step, which gives naturally deterministic video.

**2026-09 update: pool pages are no longer launched or driven by Playwright**. Playwright is still responsible for `npx playwright install chromium` and for locating the executable (`chromium.executablePath()`, Chrome for Testing, with H.264), but the pool browsers are now spawned directly by `packages/server/src/browser.ts` (`--remote-debugging-pipe`, a minimal CDP client of about 200 lines: `Target.createTarget / attachToTarget / Runtime.enable`, with crashes and closures reported through `Target.targetCrashed / targetDestroyed`). The reason (located by measurement with `--cpu-prof`) is that Playwright calls `Network.enable` on every page it creates, and for a debugging session with the Network domain enabled Chrome **mirrors every WebSocket frame sent by the page back to the debugger as base64** (`Network.webSocketFrameSent.payloadData`). One 640×480 pass of 1.2MB → 1.6MB of base64 encoded on the Chrome network service side and sent back to Node over the CDP pipe, where the hub then runs `toString + JSON.parse`. That is about 1ms of hub CPU per pass-frame (the largest single cost of the hub), and `ws.send` on the page side also rose from 0.25ms to 3.5ms per request. This item alone took a single page with 7 envs and three channels from 129 → 738 pass-frames/s. The screenshot/physics smoke scripts of the WebUI (`tests/smoke/shot.mjs` and others) still use Playwright and are unaffected.

Key points:

- GPU flag matrix: macOS development machines use `--enable-gpu` (ANGLE Metal. SwiftShader is unreliable on Apple Silicon, so a real GPU is required). GPU-less Linux CI uses `--disable-gpu --enable-unsafe-swiftshader` (the automatic SwiftShader fallback was removed in 2026, and without this flag WebGL creation fails outright). NVIDIA Linux uses `--no-sandbox --use-angle=vulkan --enable-features=Vulkan --disable-vulkan-surface`. All of them add `--force-color-profile=srgb --force-device-scale-factor=1 --hide-scrollbars`.
- ffmpeg: `-f rawvideo -pix_fmt rgba -s WxH -r fps -i - -vf vflip -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p -movflags +faststart out.mp4` (readPixels rows run bottom-up, so ffmpeg does the `vflip`, and `yuv420p` is the baseline for mp4 compatibility). stdin backpressure (`drain`) is respected.
- PNG stills are encoded on the Node side (sharp/pngjs), not inside the page.
- Frames carry a step-number tag, and WS backpressure is controlled through `bufferedAmount`.
- Chromium download: a shared cache through `PLAYWRIGHT_BROWSERS_PATH`. `npx playwright install chromium --no-shell` downloads only the full Chromium.
- Risk tracking: `--enable-unsafe-swiftshader` is a deprecation-transition switch and upstream is heading towards removing it entirely. The alternatives are Mesa llvmpipe or a CI runner with a GPU. Pixels differ between mac (Metal) and CI (SwiftShader), so golden tests use per-backend reference images or a tolerance. Segmentation maps are flat color + Nearest + no AA and are therefore bit-identical across backends.

### 5.4 Segmentation/depth rendering: three-pass design (beauty / ID / depth)

**Decision**: a single `WebGLRenderer` (WebGL2, **no migration to WebGPU**, whose readback path has known bugs) with three passes:

- **beauty**: regular materials rendered to the canvas (or an RT, texture.colorSpace=SRGB).
- **ID pass**: per-mesh material replacement (a shared flat-color `MeshBasicMaterial` is cached for each link at load time, and one `scene.traverse` switches them). `scene.overrideMaterial` is a single instance and cannot vary a uniform per object, so it is unusable. The `onBeforeRender` trick depends on undocumented internals and was dropped. Rendering goes to a non-multisampled RGBA8 RT (`NearestFilter, samples: 0`), is read back with `readRenderTargetPixels(Async)`, and `id = (r<<16)|(g<<8)|b`, with background 0.
- **depth pass**: depth has no per-object difference, so it can and should use `scene.overrideMaterial` + one ShaderMaterial that writes metric view-space depth (`-mvPosition.z`). **Since 2026-09 it is bit-packed into RGBA8**. GLSL3 `floatBitsToUint` writes the 4 bytes of the float32 into R/G/B/A in little-endian order (the `k/255 → unorm8` conversion is exact, the same contract as the ID pass), and the RGBA8 bytes read back are directly a Float32 LE array. The readback size goes from 4.9MB → 1.2MB (the same path and the same batched PBO readback as rgb/seg), and the JS loop that extracted the R channel pixel by pixel is gone. The wire format (channel 3 = raw float32 LE meters) is unchanged. The original scheme with a `FloatType RGBA32F` RT (the only float combination that WebGL2 guarantees to be readable with readPixels) is kept in the history as a reference. **Rejected alternatives**: `DepthTexture` cannot be read with readPixels directly and also needs its non-linearity undone, and `MeshDepthMaterial+RGBADepthPacking` packs non-linear NDC depth, which loses a lot of precision far away.

Key facts for bit-exactness (verified by reading the three source code):

- When rendering to an ordinary RT, three **automatically** skips toneMapping and the output color space transform (`WebGLPrograms.getParameters` forces NoToneMapping and an identity working→working transform for RTs). There is no need to turn off ColorManagement globally or to touch `renderer.outputColorSpace`.
- The only trap is the `Color` class. `setHex(id)` **performs an sRGB→Linear conversion by default, which destroys the ID**, so `color.setRGB(r/255, g/255, b/255)` (default working space, no conversion) or `setHex(id, LinearSRGBColorSpace)` must be used. The GLES3 float→unorm8 rounding guarantees that `k/255` returns exactly `k`.
- ID materials force `transparent:false, blending:NoBlending, fog:false, dithering:false`, and `side` is copied from the beauty material (inner surfaces of OBJ files often need DoubleSide, which keeps the ID silhouette consistent with RGB). `scene.background=null` during the ID/depth passes. Policy for translucent surfaces (MTL `d<1`): the ID pass renders them as opaque (the surface owns the pixel, consistent with depth).
- A self-test at engine startup renders a checkerboard of known IDs, reads it back and asserts exact equality, which is cheap insurance against exotic drivers.
- If the beauty canvas uses `antialias:true`, RGB edges and ID edges can differ by 1px. The headless engine canvas always uses `antialias:false` to guarantee pixel-to-pixel correspondence of the three channels (the interactive WebUI view is not bound by this constraint).

**Export formats**: segmentation = 8-bit RGB PNG (encoded from raw bytes on the server side) + `{id: link_name}` JSON. Depth = uint16 PNG in millimeters (the common convention of RGB-D datasets, 0–65.535m@1mm) + an optional lossless float32 `.npy` (for evaluation-grade data).

### 5.5 Concurrency (headless only): a three-level pool for 50 instances + in-page encoding

**Scope**: concurrency serves headless data production only (simulation stepping, recording, frame capture) with no real-time rendering requirement. Throughput is the only metric, and running slower than real time does not affect the correctness of the output (step-driven + simulation timestamps). The interactive WebUI mode is fixed to a single instance, and none of the mechanisms in this section apply to it.

**Conclusion**: 50-way concurrency is feasible, but it cannot be "50 pages each on their own + centralized handling of raw pixels". The following facts (verified in 2026-08 against the Chromium/SwiftShader source code, official documentation and production data from the community) determine the design:

**Constraining facts**

- **Each browser instance has only one GPU process**, and WebGL command decoding/execution of all pages is serialized on its GPU main thread (DrDC only moves the display compositor to a new thread and is enabled on Android only). No matter how many pages one browser instance opens, GL command processing + readPixels copies are squeezed onto ~1 core. The unit of parallelism must be the **browser instance** (each with its own GPU process and its own crash domain).
- **SwiftShader is a process-level singleton thread pool** (marl, min(cores,16) workers by default, tunable through `SwiftShader.ini ThreadCount`). A single draw is rasterized on several cores, but command decoding/translation is still serialized on the GPU main thread. Several browser instances = several thread pools, and too many oversubscribe the CPU.
- **Each renderer process holds at most ~16 active WebGL contexts** (beyond that the oldest is dropped, and the limit can be raised with `--max-active-webgl-contexts`). With too many same-origin pages Chromium also merges renderer processes (soft limit ≈ RAM/2÷85MB). The design of a single canvas per page + several envs sharing one context naturally sidesteps both.
- **Memory**: a headless browser instance is measured to start at ~0.5–0.7GB and creep up as it runs (production reports say 200MB→1GB+). 50 single-env browser instances ≈ 15–35GB of pure overhead. With several envs co-hosted on a page, assets are uploaded only once per context, and the increment for 50 envs is just the cloned Mesh nodes + the Rapier Worlds (the trimesh of the same furniture is duplicated in each World, a few MB).
- **Raw pixel arithmetic**: 720p RGBA = 3.69MB/frame, and 50×30fps = **5.53GB/s, 19.9TB/hour**. A single Node ws process (measured 0.25–2GB/s), sustained writes on any consumer NVMe, and 50 x264 streams (veryfast ≈ 0.5 core per stream → ~25 cores) are all blown past their limits. Direct per-frame PNG encoding (libpng ~5.5MP/s per core) would need hundreds of cores and is strictly forbidden.
- **Physics and JS are not the bottleneck**: a Rapier step with ~20 bodies is far below 1ms. A page has one JS thread, so its throughput ceiling ≈ 1 core, and envs per page and the page count only need to be balanced against the physical cores.

**Design**

1. **Three-level pool** `B×P×E` (browser instances × pages per instance × envs per page), with defaults aimed at 12–16 cores: B=6–8, P=1, E=6–8 ≈ 50. It is configurable with `--pool`, and the M3 benchmark script (`examples/bench_pool.ts`) sweeps the parameters per machine to settle the shape. When throughput is a hard requirement, running in batches of "two waves × 25" is also supported (step-driven workloads are naturally friendly to this).
2. **Video is encoded inside the page by default**: WebCodecs `VideoEncoder` (avc1. Playwright ≥1.57 bundles Chrome for Testing, which includes proprietary codecs, so on mac it uses VideoToolbox hardware encoding and on Linux/SwiftShader it uses OpenH264 software encoding. `isConfigSupported` is probed at startup, with a fallback to vp9/webm when unsupported). Step-driven simulation naturally provides flow control and exact timestamps (**timestamps carry simulation time, not wall-clock time**). The aggregate output of 50 instances drops from 5.53GB/s raw to an encoded stream of **~20–40MB/s**, which a single server process handles easily and muxes into mp4 with zero transcoding. This one decision removes the three problems of transport, centralized encoding and the NVENC limit of 8 sessions at once, and spreads the encoding load across the 50 Chromium processes that the OS is scheduling anyway.
3. **The raw pixel path is kept but governed**: segmentation/depth/lossless evaluation frames use the original readPixels→WS path, with a global token-bucket rate limit on the server + `bufferedAmount`/ffmpeg stdin `drain` backpressure. If 50 sustained raw streams are needed in the future, shard with one server process per 6–8 instances.
4. **Determinism boundary**: rendered pixels and physics are bit-deterministic (unaffected by the encoder). Hardware-encoded video (VideoToolbox/NVENC) is not bit-identical across machines/drivers, so use `hardwareAcceleration: 'prefer-software'` when bit-exact video is needed. The env creation order must be deterministic (Rapier determinism depends on insertion order), and round-robin stepping is safe.
5. **Large-scale runs on Linux**: evaluate Mesa llvmpipe (Xvfb + `--use-angle=gl`) first as a replacement for SwiftShader, since public benchmarks show 49% less CPU and 3–4× faster WebGL. NVIDIA machines use the Vulkan flags (§5.3).
6. **Recycling**: recycle envs at episode boundaries (World `.free()` + pooling), and recycle pages/browser instances by count on long runs, to counter memory creep.
7. **Several clients sharing one server** (AgentAssemble and similar: N workers each `lp.connect()` one env): ① concurrent `reserve()` calls share the free slots of in-flight pages. Previously in-flight pages were visible only to `warm()`, so N clients calling `create_envs(1)` at the same time each launched a page (60 clients → 60 pages / 60 WebGL contexts, ~110 Chromium processes, ~16 GB, with page crashes), and now ⌈N/E⌉ pages are launched. ② When an SDK socket drops, the hub reclaims the envs that this connection created and has not yet destroyed, so workers killed by SIGKILL or a timeout no longer occupy pool capacity forever (previously every leak pushed later requests into "over capacity → launch another page", and on long runs the page count grew monotonically until a crash). Measured (`--pool 8,1,8`, 60 clients × 5 waves, with 1/3 of each wave disconnecting without calling close): before the fix 8 → 24 pages and 100 dead envs left over, and after the fix always 8 pages and 0 left over.

**Measured benchmark** (M3 acceptance, 2026-08, MacBook 12 cores / 24GB, Metal, ArtVIP task_01/9, pool 8×1×7): 50 envs created in 6.9s, URDF broadcast load in 216ms, physics at 205k env-steps/s (300 steps × 50 envs in 73ms in total, so physics really is not the bottleneck), three-channel rendering (rgb+seg+depth, 640×480) at an aggregate 236 pass-frames/s, and no OOM or context-lost throughout. To reproduce: `node packages/server/dist/cli.js bench --urdf <path> --envs 50`.

**Server measurements** (2026-08-09, a Linux workstation with 96 cores / 900GB / Ubuntu 24.04, pure CPU SwiftShader): 50 envs (pool 8×1×7) at 163k steps/s and 207 sets/s. 200 envs on a single hub at 405k steps/s and ~190 sets/s (bottlenecked by the pixel return bandwidth of the hub, consistent with item 3 of this section). 200 envs with **4-hub sharding** at ~500k steps/s and **411 sets/s**. **400 envs run stably on a single hub** (created in 21s, 502k steps/s). **Single-machine limits** (2026-08-09, same machine type, Intel Xeon Sapphire Rapids 96 vCPU): **1,600 envs stable on a single hub** (created in 36.6s, and 800 envs in 22.3s). Peak physics of **638k env-steps/s** (@800 envs). Peak annotation throughput of **464 sets/s ≈ 1,392 single-channel frames/s** (8 hubs × 50 envs, 400 envs rendering at the same time in total), which is the saturation point of software rasterization on 96 vCPUs. In the sweep of sharding configurations (4/6/7/8/12 hubs), 8×50 is the best.

Container notes: `/tmp` may be extremely small (point TMPDIR to a large partition of the rootfs. CPFS/NFS do not support the unix sockets of Chromium and cannot be used as TMPDIR). The Linux launch arguments already include `--disable-dev-shm-usage`. `LIGHTPHYX_GPU=vulkan` needs an NVIDIA Vulkan ICD inside the container.

**2026-09 concurrency performance optimization (same 12-core MacBook / 24GB, Metal, same ArtVIP scene)**

| Metric | Before | After | Factor |
|---|---|---|---|
| Create 50 envs (including launching 8 browsers) | 3.2 s | 1.1 s | 3× |
| Three-channel rendering 640×480, 50 envs, in-process (`bench`, default pool) | 423 pass-frames/s | 1 699 pass-frames/s | 4.0× |
| Same, pool 5×1×10 | — | 1 845 pass-frames/s | — |
| Single page, 7 envs, three channels (page throughput ceiling) | 129 pass-frames/s | 738 pass-frames/s | 5.7× |
| Three channels over the `/ws` socket (SDK path, Node client) | 380 pass-frames/s | ~1 250 pass-frames/s | 3.3× |
| Python `vec.capture(rgb,seg,depth)`, 50 envs | 283 pass-frames/s (`render`×3) | 942 pass-frames/s | 3.3× |
| Python single env `cam.capture(3 channels)` | 35 calls/s | 81 calls/s | 2.3× |
| Physics, 300 steps × 50 envs | 205k env-steps/s | 200k env-steps/s | unchanged (never the bottleneck) |

Bottlenecks located layer by layer and their countermeasures (every item was profiled first and changed afterwards, with the numbers in the table above):

1. **In-page readback**: `readRenderTargetPixelsAsync` of three creates a new PBO + fence on every call and **always sleeps 4ms first** before polling, so 3 channels × 7 envs in series means ≥84ms of pure idling per round. It was replaced by our own `PixelReader`, in which all passes of one request are issued back to back (render + `readPixels` into a pooled PBO), **one fence** covers all of them, and the first probe is not delayed (`setTimeout(0)`, which Chromium does not clamp for the first few nesting levels). Pixels are read directly into a frame buffer that carries the 5-byte protocol header (`allocBinaryFrame`) and leave the page with zero copies.
2. **In-page pipelining**: the page queue used to wait for one request to **finish completely** (including the GPU readback) before starting the next. Now `render` calls `release()` on the queue as soon as its GL commands are issued and the scene state is restored, the rendering of the next env is issued immediately, and the fence waits overlap. Requests still **start** strictly in arrival order, and the correctness of the shared RT/material swapping is unchanged (the GL command stream preserves order).
3. **Depth readback size**: RGBA32F → bit-packed RGBA8 (§5.4), 4×.
4. **Hub unmasking**: browser→hub WS frames must be masked per the RFC, and without `bufferutil` installed `ws` XORs byte by byte in pure JS. Adding `bufferutil`/`utf-8-validate` (native) took hub CPU from 86% → 69%.
5. **The Network mirroring tax of Playwright** (§5.3 update): saves about 1ms per pass-frame on each of the hub and Chrome sides, and is the largest single item.
6. **Zero-copy forwarding in the hub**: page frames already carry the `[u32 id][u8 channel]` header, so when forwarding to the SDK the id is rewritten in place before `send`, and a 1.2MB `Buffer.concat` is no longer made.
7. **Parallel browser launch + warm-up**: `reserve()` registers the browser slots synchronously and then launches concurrently (it used to be a serial chain). `--warm-envs N` (passed automatically by the SDK `launch_vec`) lets the browsers start while the client is still connecting. `create_envs` is dispatched to the pages concurrently, and gids are still assigned in reservation order (deterministic).
8. **Outline edge geometry cache** (`Outliner`): `EdgesGeometry` used to be cached by mesh uuid, but every `load_urdf` yields fresh clones of the template. A workload that reloads the scene on every frame (the oracle of AgentAssemble: for every view it writes a flattened URDF → load → capture(outline) → remove) recomputed the edges of all meshes on every frame, about 18ms/frame at 512². Clones share the `BufferGeometry` of the template, so the cache is now page-level, keyed by source geometry uuid + threshold, and reference counted (LRU of 512 entries, evicting only unreferenced items). The AgentAssemble oracle was measured at 48 → 25 ms/frame.
9. **Python client**: our own RFC 6455 frame reader (`recv_into` reads large blocks directly, with no more 16KB fragments + join copies), and the binary payload goes zero-copy as a `memoryview` into `np.frombuffer`. seg decoding uses three vectorized operations on a uint32 view. Depth is zero-copy. `VecSim.capture(passes)` fetches several channels in one round trip (single-channel `render` is its special case).

**Remaining bottlenecks** (profiled again after the optimization): (a) with 50 envs in-process, queueing in the GPU processes (the fence wait per request goes from 7 → 13ms, with 5–8 browsers sharing one Metal GPU) and oversubscription of the 12 cores. (b) On the SDK path, "receive 1.2MB + send 1.2MB" on the hub main thread costs about 0.3ms/frame (kernel copies + GC), a single hub over `/ws` reaches about 1.2–1.3k pass-frames/s (≈1.5GB/s), and the Python side lands at ~0.95k after adding reception and decoding. Candidates for the next step (by benefit/cost): ① hub sharding with `worker_threads`, where each worker has its own WS port and takes over a group of pages, and frames are handed to the main thread for forwarding as zero-copy transferable ArrayBuffers, which reproduces the multi-hub effect of item 3 in §5.5 inside one process. ② RLE for the seg channel (flat-color regions, compression ratio >50×, 1–2ms of JS on the page side, decoded with `np.repeat` on the Python side), which removes almost 1/3 of the bytes. ③ 3-byte packing for rgb (−25%). ④ Switching the recording path to `VideoFrame(canvas)` so that the GPU feeds the encoder directly, with no readback. These are all protocol-level or architecture-level changes and were not done in this round.

**Memory usage** (measured 2026-09, `ps` RSS, hub + all Chromium processes, ArtVIP scene, 640×480, three channels):

| Configuration | After creating 50 envs | After 30 render rounds | Processes |
|---|---|---|---|
| 8×1×7, before | 5.5 GB | 6.3 GB (fluctuating within 6.2–7.4 GB over 300 rounds) | 49 |
| 8×1×7, after | 5.0 GB | 5.6 GB | 33 |
| 6×1×7 (default on this machine, 8 pages) | 4.1 GB | 5.8 GB | 27 |
| 5×1×10 | 3.2 GB | 4.8 GB | 21 |

Conclusion: **memory grows linearly with the number of browser instances B and is nearly independent of the number of envs**. A single browser with 7 envs is at about 750 MB after rendering (main process 231, renderer process 224, GPU process 211, Storage 81), while going from 1 env to 14 envs on the same page adds only ~30 MB (≈ 2 MB per env, ArtVIP scene). Throughput and memory therefore move in the same direction, and on this machine 5×1×10 is both the fastest and 1.5 GB lighter than 8×1×7. Two launch switches went into `browser.ts` (merged into `--disable/enable-features`, and merged automatically with switches appended through `LIGHTPHYX_CHROMIUM_ARGS`). `SpareRendererForSitePerProcess` turns off one spare renderer process per browser that would never be used (−100 MB), and `NetworkServiceInProcess2` folds the network service back into the main process (−90 MB). A single browser goes from 940 → 748 MB, the 50-env peak drops by about 1–1.5 GB, and there is no measurable difference in throughput (run-to-run variation is ±15% anyway). The renderer process itself went from 351 → 224 MB (the RGBA32F depth target and the per-call PBO allocations/base64 mirror strings are all gone). There is no monotonic growth over a 300-round run. After `destroy_envs` Chromium does not return the high-water mark (the V8/GPU heaps are retained), so long-lived services only need to recycle pages periodically as described in item 6 of this document. The hub process is stable at ~300 MB (of which ~100 MB is the `playwright` package itself, loaded only to locate the executable).

**Pool shape sweep on this machine** (50 envs, three channels, after the optimization, `--pool B,P,E`): 8×1×7 → 1 674, **5×1×10 → 1 845**, 6×1×8 (48 envs) → 1 407, 4×1×13 (52 envs) → 714 (4 browsers cannot feed enough). The page main thread now spends only ~2.5ms per request (issue 0.3 + copy 1.9 + send 0.3), so even 10 envs per page are only ~25% busy. The default is still B=⌈cores/2⌉≤8 and E=7, and large machines settle the shape with this sweep.

### 5.6 Evaluation of alternatives to Chrome (2026-09)

Question: rendering and physics both run inside a Chromium page, so is there a faster host? Each option is evaluated against the project premise of "keeping the same three.js + Rapier code path, with pixel-identical WebUI and headless":

| Option | Feasibility | Conclusion |
|---|---|---|
| **A. Node + `headless-gl` (WebGL without a browser)** | Not feasible | WebGL1 only. three r163+ has removed the WebGL1 backend, and float/multi-target RTs, `fenceSync` and PBOs are all missing. The project pins three 0.185. |
| **B. Deno / Node + native WebGPU (wgpu / Dawn bindings) + three `WebGPURenderer`** | Medium (POC level) | Removes the browser process, the GPU-process IPC and one WS hop. Readback uses a staging buffer with `mapAsync`, with no 6ms-level fence round trip, and several envs can share memory through `worker_threads`. Cost: all materials move to TSL/WGSL (OBJ/MTL → `MeshPhongMaterial` is already supported by WebGPURenderer, and the custom ID/depth shaders have to be rewritten), the WebGPU of Deno behaves differently from Chrome Dawn, the RT-only rendering path without a canvas lacks production validation, and WebCodecs recording is unavailable (an ffmpeg fallback is needed). The WebGPU readback bug mentioned in §5.4 also has to be re-verified in a browserless environment. Upper bound of the benefit: single-machine throughput moves from the "hub/GPU process" bottleneck to a pure GPU/CPU bottleneck, estimated at 2–4×. The price is rewriting the rendering layer and giving up the guarantee that "headless and WebUI share the same GL path and are pixel-identical". |
| **C. Native simulators (MuJoCo / SAPIEN / Genesis / Isaac, or Python `moderngl` EGL + in-house code)** | Feasible but changes the nature of the project | Throughput can be an order of magnitude higher (batched native rendering, no serialization), but URDF semantics, material appearance, the segmentation/depth contract, determinism and recording all have to be redone, and the WebUI (three.js) and headless are no longer the same renderer, so the core selling point of the project disappears. It suits a start-from-scratch option for "needing >5k frames/s on one machine" and is not the direction in which this project evolves. |
| **D1. Staying in Chrome: WebGPU backend (Dawn)** | High, best value in the medium term | three `WebGPURenderer` is available in Chrome. Readback with `buffer.mapAsync` is a truly asynchronous event and not polling, MSAA/Nearest/integer RTs are all there, and WebCodecs and zero-copy recording with `VideoFrame(texture)` are available. GPU-process serialization and IPC remain, but the GL fence polling and the synchronous PBO `getBufferSubData` are gone. Readback correctness (the concern of §5.4) and the bit-exactness of the ID pass need to be verified. A POC benchmark is recommended before deciding. |
| **D2. Staying in Chrome: `OffscreenCanvas` + Worker** | High, limited benefit | More JS threads per page and one GL context per worker. The GPU process is still the serial unit per browser (constraining facts of §5.5), so it is equivalent to a more memory-efficient version of P>1. The page main thread is now only ~25% busy and is not the current bottleneck. |
| **D3. `chromium_headless_shell` (old headless)** | Feasible, not recommended | Starts faster and uses less memory, but has no proprietary codecs (H.264 recording stops working). |
| **E. Multi-threaded / multi-process hub sharding** | High, multi-hub measurements already exist | See ① under "Remaining bottlenecks" in §5.5. It is the most direct next step outside Chrome. |

**Conclusion**: keep Chromium in the short term. This round already gained 4× without changing the architecture, and the largest bottlenecks (the Playwright mirroring tax and the three polling) were about how the host is used and not about the host itself. In the medium term, evaluate the WebGPU backend inside Chrome (D1) and do hub sharding (E). B/C are worth considering only when the single-machine target exceeds ~5k pass-frames/s or deployment without a browser is needed, and they come with the cost of rewriting the rendering layer and losing the pixel-consistency guarantee.

## 6. Coordinate system, units and ID conventions

- World coordinates = URDF coordinates: Z-up, meters, radians, `camera.up=(0,0,1)`, gravity `(0,0,-9.81)`.
- Link ids are numbered from 1 in URDF document order (0 is reserved for the background). Segmentation maps use the 24-bit encoding `id = R + G<<8 + B<<16`. Dynamically spawned objects continue in the same id space, and `seg_labels()` returns the full mapping.
- Camera: right-handed, with the output image origin at the top left. Intrinsics are expressed as `fov (vertical) + size`, and the result of `render()` carries the `K` matrix for downstream 3D tasks.

## 7. Milestones and acceptance

| Milestone | Content | Acceptance criteria |
|---|---|---|
| M1 Rendering | Repo scaffolding, URDF/OBJ/MTL loading, uniform lighting, WebUI (orbit + joint sliders) | ArtVIP `task_01/9` displays correctly with textures. Dragging a slider slides the drawer smoothly. Limits take effect |
| M2 Physics | Rapier integration, kinematic links + dynamic spawned objects, ground, collision events, deterministic stepping | A ball placed in the drawer → the ball is carried away when the drawer is pulled open. `contacts()` reports correctly. Two runs with the same seed are bit-identical |
| M3 Headless + capture | Server, WS protocol (env addressing), headless Playwright, rgb/seg/depth export, WebCodecs recording, B×P×E instance pool, benchmark script | One CLI command produces RGB + segmentation + depth + video for the example. The segmentation map matches the number/shape of links by eye and by program check. 50 envs finish the benchmark script stably and report aggregate fps with no OOM/context-lost |
| M4 SDK + polish | Python SDK (including the vectorized API), examples, README/API docs, CI | The two example scripts of §3.4 (single env + 50-env vectorized) run as written. CI is green (lint+unit+headless smoke+sdk e2e) |

One PR per milestone, with the full CI run before merging.

## 8. Testing and CI

- **Unit** (vitest): URDF parsing (including the 8-prismatic file of this example), FK values (checked against hand-computed matrices), ID encoding/decoding, protocol encoding/decoding.
- **Integration** (playwright): load the example headless → assert that the rendering is non-empty, that the id set of the segmentation map == the link set, and that the depth range is reasonable. Physics: a ball falls and comes to rest on the ground at z≈r.
- **SDK e2e** (pytest): for the scripts of §3.4, assert that the output files exist and that the number of ids in seg is correct.
- **CI** (GitHub Actions, ubuntu): npm build + vitest + playwright (SwiftShader software rendering) + pytest. Local development on macOS serves as verification on real hardware.

## 9. Risks and countermeasures

| Risk | Countermeasure |
|---|---|
| A dynamic object squeezed between a kinematic link and static geometry is ejected violently (kinematic force is unbounded) | Limit the per-step joint increment, interpolate large commands automatically, and optionally stop the joint when collision events detect an obstruction (§5.2) |
| Trimeshes have zero thickness, and extreme joint jumps can still tunnel | CCD + per-step displacement ≤ half the thinnest wall thickness. Large set_joint jumps are interpolated automatically at the SDK layer |
| ArtVIP collision meshes are high-poly copies of the visuals, so the narrow phase is expensive | Measure first. If slow, decimate the collision meshes offline or fall back to a convex hull per link (configurable) |
| `--enable-unsafe-swiftshader` is in a deprecation transition, and upstream may remove software-rendered WebGL entirely | Track blink-dev. Alternatives are Mesa llvmpipe or a CI runner with a GPU |
| Beauty pixels differ by backend between mac (Metal) and CI (SwiftShader) | Golden tests use per-backend reference images/tolerances. Segmentation maps are flat color + Nearest + no AA and are bit-identical across backends |
| urdf-loader 0.x may ship breaking minor versions | Pin 0.13.1. The combination of `<mesh scale>` + texture paths is covered by unit tests |
| CI rendering is slow on SwiftShader | CI only runs a 256×256 smoke test. Throughput benchmarks stay local. Evaluate Mesa llvmpipe for production runs (§5.5) |
| Large OBJ files load slowly | Accepted in v1. A slot is reserved for a glTF conversion script (non-goal) |
| Historical bug in sleep wake-up on kinematic contact (fixed in 0.19.2) | Version ≥0.19.3 + manual `wakeUp()` fallback + regression test (ball asleep in the drawer → pull it open) |
| Memory creep of headless Chromium on long runs (200MB→1GB+ is normal in production) | Recycle envs at episode boundaries, recycle pages/browsers by count, and monitor per-instance RSS |
| The Chrome for Testing bundled with Playwright may lack H.264 (open-source Chromium builds have no proprietary codecs) | Probe `VideoEncoder.isConfigSupported('avc1…')` at startup and fall back to vp9/webm. Pin the Playwright version |
| Shared materials/textures contaminate each other when randomized per env | Share read-only assets and clone anything changed per env. This is a code review checklist item |
| The aggregate throughput of 50 envs is limited by software rasterization, and planned values ≠ measured values | The M3 benchmark script comes first, capacity planning follows the measured fps, and production is not scheduled at 50×30fps |

## 10. Dependency list (runtime)

- engine: `three`(0.185.x), `urdf-loader`(pinned to 0.13.1), `@dimforge/rapier3d-compat`(0.19.3)
- server: `ws`, `sharp`(PNG encoding, including 16-bit), `playwright`(1.62.x, needed for headless only), system `ffmpeg`(optional, degrades to a PNG sequence when missing)
- webui: `lil-gui` (dev dependencies: vite, typescript, vitest)
- python: `websockets` (or websocket-client), `numpy`, `pillow`

The versions are those current at the time of the 2026-08 survey, and the lock file is authoritative during implementation.
