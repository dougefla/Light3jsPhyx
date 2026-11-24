# light-3js-phyx Usage Guide

For users. This guide covers installation, running a single environment, and **running in parallel** (vectorized, multi-threaded, and multi-process with a shared server). For the architecture and design rationale see [DESIGN.md](DESIGN.md).

---

## 1. Installation

```bash
npm ci && npm run build            # build the WebUI and the server
npx playwright install chromium    # needed for headless (one-time)
pip install -e sdk/python          # Python SDK (depends on numpy / pillow / websocket-client)
```

The SDK walks up from its own location to find `packages/server/dist/cli.js`. If the repository is not in a parent directory of the SDK, point to it with `LIGHTPHYX_CLI=/abs/path/cli.js`.

---

## 2. Single-environment quick start

```python
import lightphyx as lp

sim = lp.launch(headless=True)              # start a private server + 1 env
obj = sim.load_urdf("path/to/scene.urdf")   # returns an ArticulatedObject
print(obj.joints)                            # {name: Joint(name, type, lower, upper, value)}

sim.add_ground(z=0.0)
ball = sim.spawn_sphere(0.03, pos=(0.2, -0.4, 1.1))

cam = sim.camera(pos=(1.5, -1.5, 1.5), lookat=(0, 0, 0.6), fov=50, size=(640, 480))
# or cam = sim.frame_camera(fill=0.7)       # automatic framing

obj.set_joint("joint_0", -0.2)               # target value, approached under the velocity limit while stepping
obj.set_joint("joint_0", -0.2, immediate=True)  # jump there immediately
sim.step(1 / 30, n=30)                       # advance simulation time by n×dt
sim.settle()                                 # step until every joint reaches its target

out = cam.capture(["rgb", "seg", "depth"])   # several channels in one round trip (faster than separate calls)
rgb, seg, depth = out["rgb"], out["seg"], out["depth"]
labels = out["seg_labels"]                   # {link_id: link_name}

with sim.record("out/clip.mp4", fps=30):     # H.264 encoding inside the page
    for q in lp.linspace(0, -0.3, 90):
        obj.set_joint("joint_0", float(q))
        sim.step(1 / 30)

sim.close()                                  # release the env, shut down the server and the browser
```

### 2.1 `launch()` parameters

| Parameter | Default | Description |
|---|---|---|
| `headless` | `True` | `False` opens a visible browser window (for watching) |
| `out_dir` | `out` | Default output directory on the server side (recordings, files written by `save=`) |
| `pool` | `(1,1,1)` | Usually left unchanged |
| `physics` | `True` | `False` = a purely kinematic/rendering env. `set_joint` takes effect immediately, no colliders are built, `step`/`contacts` are no-ops, and `add_ground`/`spawn` raise an error. Use it when you only render, because it loads faster and uses less memory |
| `self_collision` | `False` | `True` = the physics layer also detects link-to-link contacts (self-collision within one robot, and collisions between links of different robots). `sim.self_contacts()` returns the link pairs currently in contact, and `contacts()` also lists `link:robot/a`–`link:robot/b`. Parent and child links directly connected by a joint are never reported (they overlap in the URDF anyway). Trimesh-to-trimesh detection has a cost, so enable it only when needed |
| `rpc_timeout` | 120 s | Timeout of a single call. A timeout raises `lp.RpcError` |
| `startup_attempts` | 1 | Number of retries when the server or browser fails to start |

### 2.2 Output formats

| Channel | dtype / shape | Meaning |
|---|---|---|
| `rgb` | `uint8 (H, W, 3)` | Color under global uniform lighting. `outline=True` overlays CAD-style outlines (affects rgb only) |
| `seg` | `int32 (H, W)` | Per-pixel link id, 0 = background. `seg_labels` gives the id→name mapping |
| `depth` | `float32 (H, W)` | Metric view-space depth |

`cam.capture(passes, save={"dir": "/abs/dir", "prefix": "f001"})` makes the server write PNG / npy files to disk directly, and the return value gains a `files` list.

### 2.3 Scene operations cheat sheet

`load_urdf(path, name=, pos=, yaw=, dynamic=, stiffness=, damping=, density=)` · `set_joint_drive(name, stiffness=, damping=)` · `remove_robot(name)` · `spawn_box / spawn_sphere / spawn_cylinder` · `remove_object(name)` · `joints()` · `set_joints({name: v})` · `state()` · `contacts()` → `[(body_a, body_b)]` · `self_contacts(robot=)` → `[(link_a, link_b)]` (requires `self_collision=True`) · `reset()`

### 2.4 Self-collision detection

```python
sim = lp.launch(self_collision=True)
arm = sim.load_urdf("arm.urdf")
arm.set_joints({"elbow": 2.9, "wrist": -1.5})
sim.settle()
if sim.self_contacts():                      # [("upper_arm", "gripper"), ...]
    print("self-collision!", sim.self_contacts())
```

- **Detection only, no prevention.** Links are kinematically driven rigid bodies and the physics layer applies no constraint force to them. Joints still travel to their targets, interpenetration still happens, and `self_contacts()` only tells you about it. To avoid interpenetration, back off the joint value in your own control logic based on `self_contacts()`, for example by approaching step by step and returning to the last collision-free value on contact.

  ```python
  def move_until_clear(sim, obj, name, target, steps=20):
      start = sim.state()["joints"][name]
      last_ok = start
      for q in lp.linspace(start, target, steps):
          obj.set_joint(name, float(q))
          sim.settle()
          if sim.self_contacts():
              obj.set_joint(name, last_ok, immediate=True)   # back to the collision-free value
              sim.step(1 / 60)
              return last_ok
          last_ok = float(q)
      return last_ok
  ```

- The physics layer (Rapier) makes a geometric decision from the link colliders, independent of the rendering viewpoint. Links are hollow trimeshes, so a contact is reported when two link surfaces cross and is not reported when one link is fully swallowed inside another.
- Parent and child links (directly connected by a joint) are filtered out. Sibling links that already overlap in the URDF are reported all the time, which is a model issue.
- For pixel-level or view-dependent occlusion checks, use `cam.seg()` and compare masks yourself.
- To have the physics layer prevent interpenetration and let links push each other, use `dynamic=True` from the next section.

### 2.5 Dynamic links: `load_urdf(..., dynamic=True)`

In the default mode links are kinematic bodies whose poses are written directly by FK. This is exact and cheap, but nothing can push anything else. With `dynamic=True` the whole robot is modeled along the lines of an Isaac/PhysX articulation, in which links are dynamic rigid bodies, joints are Rapier constraints with PD motors, and contacts and joints are solved together in the same solver. As a result, pulling a drawer pushes open a passive door standing in front of it, a joint that hits a clamped object stops and does not pass through it, and objects placed in a drawer are carried along as usual.

```python
sim = lp.launch(self_collision=True)              # turn it on as well if you want to see link contact pairs
fridge = sim.load_urdf("fridge.urdf", dynamic=True)
fridge.set_drive("door_hinge", stiffness=0, damping=2)   # the door is not driven, it can only be pushed
fridge.set_joint("drawer_slide", -0.3)            # the drawer motor moves towards the target
sim.settle()                                      # dynamic mode: wait until all links are at rest
print(sim.state()["joints"]["door_hinge"])        # > 0: the drawer pushed the door open
```

| Parameter | Default | Description |
|---|---|---|
| `dynamic` | `False` | `True` enables dynamic links. The root link is fixed in the world, and links connected by fixed joints are merged into one rigid body |
| `stiffness` / `damping` | `400` / `40` | Default PD gains of all joint motors. Acceleration model, in units of 1/s² and 1/s, independent of mass. `set_joint_drive` changes them per joint, and `stiffness=0` makes a passive joint |
| `density` | `300` | kg/m³, used to estimate the mass and inertia of each link from the bounding box of its colliders (URDF `<inertial>` is ignored, since in datasets it is often empty or inconsistent) |

Behavioral differences:

- `set_joint(v)` becomes a motor target that is still approached step by step under the URDF velocity limit. The joint values in `state()` are the actual values solved back from the rigid-body poses, so a blocked joint does not reach its target.
- In dynamic mode `settle()` waits for "all targets issued + all links at rest". A blocked drawer resting against a door also counts as at rest.
- The Rapier JS API has no equivalent of the PhysX maxForce. The motor is a spring whose force ≈ stiffness × (target minus actual) × effective mass, with no hard cap. To make something "impossible to push", raise the stiffness of the pushed side (for example 1e5) or lower that of the pushing side.
- Joints use the maximal-coordinate impulse method, not Featherstone. Chains of two or three levels are fine, and long chains drift slightly under large contact forces. Rendering uses the ideal kinematic tree rebuilt from the joint values and is unaffected by the drift.
- Links are still hollow trimeshes, and contacts on very thin or open meshes are unstable. Dynamic mode costs more per step than kinematic mode, so enable it only for objects that need a passive response.
- `set_joint(..., immediate=True)` and `reset()` are still teleports, in which the rigid bodies are placed directly at the FK poses and their velocities are zeroed.

---

## 3. Parallelism: pick a mode first

All parallelism works only in headless mode, and all of it runs on the **B×P×E pool of a single server**:

```
server (hub, one Node process)
 ├─ browser 1 ── page 1 ── env × E      ← browser = unit of parallelism (own GPU process, own crash domain)
 ├─ browser 2 ── page 2 ── env × E      ← envs on one page share loaded assets (meshes/textures uploaded once)
 └─ … B browsers in total
```

Memory is essentially **proportional to the number of browsers B only** (about 0.75 GB each) and nearly independent of the number of envs (about 2 MB/env). To scale concurrency, increase E (envs per page) and avoid starting more servers or browsers.

| Mode | When to use | Entry point |
|---|---|---|
| **A. Vectorized** | One Python process drives N envs synchronously with uniform operations (batch data collection, RL vector environments) | `lp.launch_vec(N)` |
| **B. Multi-threaded** | N independent episodes in one process, each at its own pace | `launch_vec` + `vec[i]` inside threads |
| **C. Multi-process, shared server** | N independent worker processes (evaluation, agents, `multiprocessing`) that may be killed on timeout | The parent starts the server, the children call `lp.connect(url)` |
| ✗ Each process calls its own `lp.launch()` | **Do not do this** | N servers + N browsers ≈ N GB of memory, a startup storm at 30+ concurrent processes, and a certain crash at 60 |

---

## 4. Mode A: vectorized (`launch_vec`)

```python
import lightphyx as lp
import numpy as np

vec = lp.launch_vec(num_envs=60)               # pool sized automatically, pool=(B, P, E) sets it explicitly
# vec = lp.launch_vec(num_envs=60, browsers=3) # only the browser count: E becomes ⌈60/3⌉ = 20
vec.load_urdf("path/to/scene.urdf")            # broadcast to all envs
vec.add_ground(visual=False)
vec.set_cameras(size=(320, 240))
vec.frame_cameras()

vec.set_joint("joint_0", np.linspace(0, -0.3, 60))   # per-env values, a scalar is broadcast
vec.set_joints({"joint_0": 0.0, "joint_1": np.random.uniform(-1, 0, 60)})
vec.step(1 / 30, n=30)                         # parallel stepping across pages

out = vec.capture(["rgb", "seg", "depth"])     # {name: (60, H, W[, 3])}
segs = vec.render("seg")                       # single channel = special case of capture
states = vec.states()                          # one dict per env
vec.reset(indices=[3, 12])                     # partial reset

env7 = vec[7]                                  # single-env handle, same interface as Sim
env7.load_urdf("other.urdf", name="extra")     # fine-grained per-env control
with env7.record("/abs/out/env7.mp4"):
    env7.step(1 / 30, n=60)

vec.close()
```

Key points:

- `launch_vec` passes `--warm-envs N`, so the browsers start launching while Python is still connecting.
- `capture([...])` fetches several channels in one request, with roughly 2× the throughput of three separate `render` calls. Requests are pipelined per env, so you do not need to add concurrency yourself.
- Example: `examples/python/vec_50_envs.py`.

---

## 5. Mode B: multi-threaded

`Client` is thread-safe (sends are locked and a background thread receives), so several threads can share one `VecSim` as long as **each thread only operates on its own `vec[i]`**:

```python
from concurrent.futures import ThreadPoolExecutor
import lightphyx as lp

vec = lp.launch_vec(num_envs=16)
vec.load_urdf("path/to/scene.urdf")

def episode(i: int):
    env = vec[i]
    cam = env.frame_camera()
    env.step(1 / 30, n=10)
    return cam.capture(["rgb", "seg"])

with ThreadPoolExecutor(len(vec)) as ex:
    results = list(ex.map(episode, range(len(vec))))
vec.close()
```

Notes:

- Different threads must not operate on **the same** env at the same time (requests run in arrival order, but the state after interleaved calls is unpredictable).
- numpy decoding holds the GIL. When the Python side is CPU-bound, switch to mode C.

---

## 6. Mode C: multiple processes sharing one server (recommended for 30+ workers)

The parent process starts **one** server, warms it up, and hands the URL to the workers. Each worker gets its own env with `lp.connect()`.

```python
import multiprocessing as mp
import lightphyx as lp
from lightphyx import Client
from lightphyx.server import ServerProcess

N = 60

def worker(url: str, k: int):
    sim = lp.connect(url, physics=False)      # this process owns 1 env exclusively
    try:
        obj = sim.load_urdf("/abs/path/scene.urdf")
        sim.camera(size=(320, 240))
        return sim.frame_camera().capture(["rgb", "seg"])["seg"].max()
    finally:
        sim.close()                            # return the env (the server keeps running)

if __name__ == "__main__":
    browsers = min(8, N)
    server = ServerProcess(pool=(browsers, 1, -(-N // browsers)), warm_envs=N)
    try:
        # Warm-up: bring up all pages at once, then return the capacity to the pool
        c = Client(server.url, timeout=max(120.0, 5.0 * N))
        ids = c.call("create_envs", count=N, physics=False)["envs"]
        c.call("destroy_envs", env=ids)
        c.close()

        with mp.get_context("spawn").Pool(N) as pool:
            print(pool.starmap(worker, [(server.url, k) for k in range(N)]))
    finally:
        server.close()
```

Runnable version: `python examples/python/shared_server_workers.py <scene.urdf> <workers>`.

You can also start a long-lived server by hand and let any process or script on the machine connect to it:

```bash
node packages/server/dist/cli.js serve --port 8323 --pool 8,1,8 --warm-envs 60
# worker: lp.connect("ws://127.0.0.1:8323/ws")
```

### 6.1 Behavioral guarantees (server side)

- **Concurrent env creation does not open extra pages.** When N workers call `connect()` at the same time, they share the free slots of pages that are still starting, and only ⌈N/E⌉ pages are launched. Before the fix each worker launched its own page, which at 60 concurrent workers meant about 110 Chromium processes, 16 GB, and crashes.
- **Automatic reclamation on disconnect.** When a worker is killed by SIGKILL or a timeout before it can call `close()`, the server destroys the envs it created as soon as its socket drops, so pool capacity is not slowly eaten away.
  - This also means that **an env belongs to the connection that created it**. The env disappears when the connection drops, and an env cannot be "handed over" across connections.
- **Page crash isolation.** When the renderer process of a page crashes, only the envs on that page become invalid and the related calls raise `RpcError` (`page page_N crashed` / `page disconnected`). Other pages are unaffected. A worker should catch the exception, call `connect()` again to get a new env, and retry.

### 6.2 Recommendations for workers

```python
import lightphyx as lp

def run_with_retry(url, job, attempts=2):
    for i in range(attempts):
        sim = lp.connect(url, physics=False, rpc_timeout=60)
        try:
            return job(sim)
        except lp.RpcError:
            if i == attempts - 1:
                raise
        finally:
            try:
                sim.close()
            except Exception:
                pass
```

- Cache one `Sim` per process and reuse it. Do not `connect` on every frame. `load_urdf` and `remove_robot` can be run repeatedly on the same env, and mesh templates are cached with reference counting.
- Pass `physics=False` when you only render and do not need physics.

---

## 7. Choosing the pool size (B×P×E)

| Parameter | Meaning | Recommendation |
|---|---|---|
| B browsers | Unit of parallelism, each with its own GPU process, about 0.75 GB | ≈ physical cores / 2, capped at 8 (with a single GPU, more only queue up) |
| P pages per browser | One more JS thread, about 0.2 GB more | Keep at 1 |
| E envs per page | Shared assets, about 2 MB/env | `⌈total envs / B⌉`. The page main thread spends about 2.5 ms per request, so even E=10 is only 25% busy |

- Defaults: `B = clamp(cores/2, 2, 8)`, `P = 1`, `E = 7`.

### 7.1 Tuning only the browser count (`browsers`)

Most of the time you only need to decide "how many browsers to open" and let the server compute the rest. The three forms below have the same effect, listed from highest to lowest priority:

| Form | Example |
|---|---|
| Python argument | `lp.launch_vec(num_envs=60, browsers=3)`, `ServerProcess(..., browsers=3)` |
| CLI argument | `cli.js serve --browsers 3 --warm-envs 60`, `cli.js bench --envs 60 --browsers 3` |
| Environment variable | `LIGHTPHYX_BROWSERS=3` (no change to the calling code, for example AgentAssemble) |

Computation rules:

- **No `pool` given**: E = ⌈expected env count / B⌉, with 1 page per browser. The expected env count comes from `num_envs` (Python), `--warm-envs` (serve) or `--envs` (bench). Without an expected count E stays at the default of 7.
- **`pool` given as well**: `browsers` replaces B while **the total capacity B×P×E is kept unchanged**, and E is recomputed for the new B. For example `pool=(8,1,8)` with `browsers=2` becomes `(2,1,32)`, which is still a capacity of 64 envs, so no extra pages are stacked on because of a smaller capacity.
- The value must be a positive integer, otherwise startup fails with an error.

How to choose: fewer browsers → less memory (about 0.75 GB each), but all rendering queues up in fewer GPU processes, and one browser crash takes more envs with it. More browsers → faster rendering but more memory, with very little gain beyond about 8 (there is only one GPU). Use 2–3 when memory is tight and throughput matters little, and use the default when you want throughput.
- Exceeding `B×P×E` total envs does not raise an error. The server **appends pages** to the existing browsers (equivalent to a larger P), and every page costs one more share of memory. It is therefore best to keep capacity ≥ peak concurrency.

Reference configurations (measured on a 12-core / 24 GB Mac):

| Concurrent envs | Recommended `pool` | Expected memory (browser part) |
|---|---|---|
| ≤ 16 | `(4,1,4)` | ~3 GB |
| 32 | `(8,1,4)` | ~6 GB |
| 50 | `(5,1,10)` (fastest on this machine) | ~5 GB |
| 60 | `(8,1,8)` | ~6 GB |
| 100+ | `(8,1,13)` and up. Lower B to 6 when memory is tight | ~6 GB |

Throughput reference (640×480, three channels, 50 envs): about 1 700–1 850 pass-frames/s in-process, and about 940 pass-frames/s with Python `vec.capture`. Physics is not the bottleneck (about 200k env-steps/s).

Tuning tool:

```bash
node packages/server/dist/cli.js bench --urdf scene.urdf --envs 60 --pool 8,1,8 --size 640x480
```

---

## 8. Long-running jobs

- The memory of the Chromium GPU process creeps up over time (the V8/GPU heaps are not returned). Long-running jobs should shut the server down and start a new one at a **safe boundary** (when no rendering is in progress, for example between rounds): call `server.close()` first, then create a new `ServerProcess`. The URL changes, so workers need to `connect` again.
- Loaded meshes are released by reference counting. Templates removed by `remove_robot` and no longer referenced are kept as a hot cache of at most 64 entries, and the rest release their GPU memory.
- To inspect the pool state: `Client(url).call("pool_stats")` → `{"pages": [{"id", "envs"}], "envs": total}`. If `envs` keeps growing, some client has not called `close()` and its connection has not dropped either.

---

## 9. CLI

```bash
node packages/server/dist/cli.js serve   [--port 8323] [--pool B,P,E] [--browsers B] [--warm-envs N] [--out DIR] [--headed] [--exit-on-stdin-close]
node packages/server/dist/cli.js capture --urdf scene.urdf [--out out/] [--size 1280x720] [--seconds 3] [--fps 30] [--no-video]
node packages/server/dist/cli.js bench   --urdf scene.urdf [--envs 50] [--pool B,P,E] [--browsers B] [--steps 300] [--renders 10] [--size 640x480]
```

- `serve` also hosts the interactive WebUI at `http://127.0.0.1:8323/?urdf=/files/<absolute path>`. The headless pages of the pool start only on the first `create_envs` (or with `--warm-envs`).
- The server listens on `127.0.0.1` only and is reachable from the local machine only. Cross-machine use requires your own port forwarding.

## 10. Environment variables

| Variable | Purpose |
|---|---|
| `LIGHTPHYX_CLI` | Path to `cli.js` (when the SDK cannot find the repository) |
| `LIGHTPHYX_BROWSERS` | Default browser count, equivalent to `--browsers` (rules in §7.1). An explicitly passed `browsers` argument takes precedence |
| `LIGHTPHYX_CHROMIUM_ARGS` | Extra Chromium launch arguments (space-separated). `--enable/disable-features` are merged automatically with the built-in ones |
| `LIGHTPHYX_GPU=vulkan` | Linux + NVIDIA: use ANGLE/Vulkan. By default Linux uses the SwiftShader software rasterizer |
| `TMPDIR` | Location of the browser user-data-dir. When `/tmp` is too small inside a container, point it to a large local partition (NFS/CPFS cannot be used) |

## 11. FAQ

| Symptom | Cause / fix |
|---|---|
| Crashes as soon as concurrency rises, Chromium process count explodes | Most likely each process calls its own `lp.launch()`. Switch to mode C, and make sure the server is a version that includes this fix |
| `RpcError: request N timed out` | The machine is overloaded or a page is stuck. Increase `rpc_timeout` or reduce B |
| `page page_N crashed` / `page disconnected` | The renderer process crashed (commonly from running out of memory). `connect` again and retry. Check memory, and lower B or raise E |
| Segmentation ids are not contiguous | seg ids are a bit-exact 24-bit encoding with no guarantee of contiguity. Rely on `seg_labels` |
| Recording fails | The current Chromium lacks an H.264 encoder (for example the headless shell). Install the full build with `npx playwright install chromium` |
