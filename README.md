# Light-3js-phyx

A lightweight rendering and physics engine for articulated objects, built on three.js and Rapier.
It is the simulation and rendering engine of [AgentAssemble](https://github.com/dougefla/AgentAssemble).

- **URDF import**: link and joint trees, OBJ meshes with MTL materials and textures, primitive geometry
- **Rendering**: uniform ambient lighting by default, so the image shows the albedo. RGB, per-link segmentation (bit-exact 24-bit ids), and metric depth
- **Physics**: Rapier with gravity and collision detection. Joints are driven kinematically (target tracking with a velocity limit, which prevents tunnelling) next to free rigid-body dynamics. Results are deterministic to the bit
- **Two modes**: an interactive WebUI (one instance) and headless batch capture share one render path and give identical pixels
- **Concurrency (headless)**: 50 instances on one machine through a pool of browsers, pages, and environments (B x P x E) that share assets, with a vectorized Python API
- **Recording**: in-page WebCodecs H.264, muxed to mp4 without transcoding
- **Agent-friendly Python SDK**: synchronous API, numpy images, a task in ten lines

## Quick start

```bash
npm ci && npm run build          # build the WebUI and the server
npx playwright install chromium  # needed once for headless mode (Playwright only installs and locates Chromium; the pool pages are driven by the built-in CDP client)

# Interactive WebUI
node packages/server/dist/cli.js serve
# open http://127.0.0.1:8323/?urdf=/files/absolute/path/to/scene.urdf

# Export RGB, segmentation, depth, and a video with one command
node packages/server/dist/cli.js capture --urdf path/to/scene.urdf --out out/

# Benchmark with 50 concurrent instances
node packages/server/dist/cli.js bench --urdf path/to/scene.urdf --envs 50
```

## Python SDK

```bash
pip install -e sdk/python
```

```python
import lightphyx as lp

sim = lp.launch(headless=True)              # physics=False gives a kinematic, render-only env (set_joint applies at once, no colliders)
obj = sim.load_urdf("path/to/scene.urdf")   # rpc_timeout / startup_attempts control timeouts and startup retries
sim.add_ground(z=0.0)
ball = sim.spawn_sphere(0.03, pos=(0.2, -0.4, 1.1))

cam = sim.frame_camera()                  # or sim.camera(pos=..., lookat=..., fov=50, size=(1280, 720))
with sim.record("out/clip.mp4", fps=30):
    for q in lp.linspace(0, -0.3, 90):
        obj.set_joint("joint_0", float(q))
        sim.step(1 / 30)

rgb   = cam.rgb()          # np.uint8  (H, W, 3)
seg   = cam.seg()          # np.int32  (H, W), cam.seg_labels() -> {id: name}
depth = cam.depth()        # np.float32 (H, W), metres
hits  = sim.contacts()
sim.close()
```

50 concurrent instances (headless only, in the style of gym.vector):

```python
vec = lp.launch_vec(num_envs=50)          # the B x P x E pool is sized automatically, or pass pool=(8, 1, 7)
vec.load_urdf("path/to/scene.urdf")
vec.set_joint("joint_0", lp.linspace(0, -0.3, 50))   # one value per env
vec.step(1 / 30, n=30)
segs = vec.render("seg", size=(320, 240))  # (50, H, W) np.int32
out = vec.capture(["rgb", "seg", "depth"])  # several channels in one round trip: {name: (50, H, W[, 3])}
env7 = vec[7]                              # handle of a single env
vec.close()
```

More examples are in [examples/python/](examples/python/).

## Documentation and tests

- [Usage](docs/USAGE.md): installation, the single-environment API, parallel use (vectorized, multi-threaded, and several processes on one server), pool sizing, long runs, and troubleshooting
- [Design](docs/DESIGN.md): architecture, protocol, the main technical decisions, the concurrency design, and measured benchmarks
- `npx vitest run` runs the unit tests, `node tests/smoke/headless.mjs` the end-to-end smoke test, and `pytest sdk/python/tests` the SDK end-to-end tests

## Benchmark

On a 12-core MacBook with 24 GB (Metal) and the default pool, 50 envs are created in 1.1 s, physics runs at
200k env-steps/s, and three-channel rendering reaches **1,699 pass-frames/s** at 640x480 in process
(1,845 with a 5x1x10 pool). The Python `vec.capture` call reaches 942 pass-frames/s for three channels,
with no out-of-memory error and no lost context. Memory is about 750 MB per browser instance and about
2 MB per env (4.8 GB peak for 50 envs on a 5x1x10 pool). [DESIGN.md §5.5](docs/DESIGN.md) describes the
optimizations and the remaining bottlenecks, and §5.6 evaluates alternatives to Chrome.

## License

MIT. See [LICENSE](LICENSE).
