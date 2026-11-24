"""Many worker processes sharing one pre-warmed server (docs/USAGE.md §4.3).

The parent owns the server and pre-launches every page; each worker connects
with lp.connect() and gets one env of its own. A worker that dies without
close() gets its env reclaimed by the server when its socket drops.

    python examples/python/shared_server_workers.py [scene.urdf] [workers]
"""
import multiprocessing as mp
import os
import sys
import time

import lightphyx as lp
from lightphyx import Client
from lightphyx.server import ServerProcess

URDF = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "tests/assets/mini_cabinet.urdf")
WORKERS = int(sys.argv[2]) if len(sys.argv) > 2 else 16


def worker(url: str, k: int) -> float:
    sim = lp.connect(url, physics=False)  # one env, owned by this process
    try:
        obj = sim.load_urdf(URDF)
        sim.camera(size=(320, 240))
        cam = sim.frame_camera()
        for name, j in obj.joints.items():
            obj.set_joint(name, j.lower * (k % 4) / 3)
        out = cam.capture(["rgb", "seg", "depth"])
        return float(out["depth"].mean())
    finally:
        sim.close()


def main() -> None:
    browsers = min(8, WORKERS)
    per_page = -(-WORKERS // browsers)  # ceil
    server = ServerProcess(pool=(browsers, 1, per_page), warm_envs=WORKERS)
    try:
        # Pre-warm: launch every page now, then hand the slots back.
        c = Client(server.url, timeout=max(120.0, 5.0 * WORKERS))
        envs = c.call("create_envs", count=WORKERS, physics=False)["envs"]
        c.call("destroy_envs", env=envs)
        c.close()

        t0 = time.time()
        with mp.get_context("spawn").Pool(WORKERS) as pool:
            depths = pool.starmap(worker, [(server.url, k) for k in range(WORKERS)])
        print(f"{WORKERS} workers done in {time.time() - t0:.1f}s; mean depth {sum(depths) / len(depths):.3f} m")
    finally:
        server.close()


if __name__ == "__main__":
    main()
