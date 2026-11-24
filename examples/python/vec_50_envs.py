"""50 concurrent envs: randomized joint states, batched segmentation render.
Headless-only concurrency demo (DESIGN.md §5.5)."""
import sys
import time

import numpy as np

import lightphyx as lp

URDF = sys.argv[1] if len(sys.argv) > 1 else "tests/assets/mini_cabinet.urdf"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 50

t0 = time.time()
vec = lp.launch_vec(num_envs=N)
print(f"{N} envs up in {time.time() - t0:.1f}s")
try:
    vec.load_urdf(URDF)
    vec.add_ground(visual=False)
    vec.frame_cameras()

    rng = np.random.default_rng(0)
    joints = [j.name for j in vec[0].joints()]
    lows = {j.name: j.lower for j in vec[0].joints()}
    for name in joints:
        vec.set_joint(name, rng.uniform(lows[name], 0, size=N))
    vec.step(1 / 30, n=30)

    t0 = time.time()
    rgbs = vec.render("rgb", size=(320, 240))
    segs = vec.render("seg")
    dt = time.time() - t0
    print(f"rendered {N}×(rgb+seg) in {dt:.1f}s → {2 * N / dt:.0f} frames/s")
    print("rgb batch:", rgbs.shape, "| seg batch:", segs.shape)
    print("per-env unique ids:", [int(np.unique(s).size) for s in segs[:8]], "…")
finally:
    vec.close()
