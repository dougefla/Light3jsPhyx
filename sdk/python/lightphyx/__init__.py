"""lightphyx — agent-friendly Python SDK for the light-3js-phyx engine.

>>> import lightphyx as lp
>>> sim = lp.launch(headless=True)
>>> obj = sim.load_urdf("path/to/scene.urdf")
>>> obj.set_joint("joint_0", -0.2)
>>> sim.step(1 / 30, n=30)
>>> rgb = sim.frame_camera().rgb()
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .client import Client, RpcError
from .server import ServerProcess
from .sim import ArticulatedObject, Camera, Joint, Sim, VecSim

__all__ = [
    "launch",
    "launch_vec",
    "connect",
    "linspace",
    "Sim",
    "VecSim",
    "Camera",
    "Joint",
    "ArticulatedObject",
    "Client",
    "RpcError",
]


def launch(
    headless: bool = True,
    out_dir: Optional[str] = None,
    pool: Optional[Tuple[int, int, int]] = None,
    *,
    physics: bool = True,
    self_collision: bool = False,
    rpc_timeout: Optional[float] = None,
    startup_attempts: int = 1,
) -> Sim:
    """Start a local server (+ one env) and return a Sim bound to it.

    physics=False creates a pure kinematic/render env: set_joint applies
    instantly, no colliders are built on load_urdf, step()/contacts() are
    no-ops (add_ground/spawn raise). self_collision=True makes the physics
    layer also detect link-link contacts (robot self-collision and links of
    different robots touching); read them with Sim.self_contacts() /
    contacts(). Off by default: trimesh-vs-trimesh detection costs step time.
    rpc_timeout overrides the 120 s client deadline. startup_attempts retries
    a failed server start/connect.
    """
    attempts = max(1, int(startup_attempts))
    last: Optional[BaseException] = None
    for _ in range(attempts):
        server = ServerProcess(pool=pool or (1, 1, 1), out_dir=out_dir, headless=headless, warm_envs=1)
        try:
            client = Client(server.url, timeout=rpc_timeout or 120.0)
            env_id = client.call(
                "create_envs", count=1, physics=physics, self_collision=self_collision
            )["envs"][0]
            return Sim(client, env_id, server=server, owns_client=True)
        except BaseException as exc:
            # Reap the child now instead of at interpreter exit (port + Chromium
            # would otherwise stay allocated for the caller's whole lifetime).
            server.close()
            last = exc
            if isinstance(exc, KeyboardInterrupt):
                raise
    assert last is not None
    raise last


def launch_vec(
    num_envs: int,
    headless: bool = True,
    pool: Optional[Tuple[int, int, int]] = None,
    out_dir: Optional[str] = None,
    *,
    physics: bool = True,
    self_collision: bool = False,
    rpc_timeout: Optional[float] = None,
    browsers: Optional[int] = None,
) -> VecSim:
    """Start a local server with a B×P×E pool hosting num_envs envs.

    browsers=N picks only the browser count and spreads the envs over them
    (E = ceil(num_envs / N)); each browser is one GPU process (~0.75 GB), so
    fewer saves memory and more renders in parallel. With pool also given it
    replaces pool[0] and keeps the pool's capacity B×P×E. Default: pool, else $LIGHTPHYX_BROWSERS, else cores/2
    clamped to 2..8.
    """
    server = ServerProcess(
        pool=pool, out_dir=out_dir, headless=headless, warm_envs=num_envs, browsers=browsers
    )
    try:
        client = Client(server.url, timeout=rpc_timeout or 120.0)
        env_ids = client.call(
            "create_envs", count=num_envs, physics=physics, self_collision=self_collision
        )["envs"]
    except BaseException:
        server.close()
        raise
    return VecSim(client, env_ids, server=server, owns_client=True)


def connect(
    url: str,
    num_envs: int = 1,
    *,
    physics: bool = True,
    self_collision: bool = False,
    rpc_timeout: Optional[float] = None,
):
    """Attach to an already-running server (ws://host:port/ws)."""
    client = Client(url, timeout=rpc_timeout or 120.0)
    env_ids = client.call(
        "create_envs", count=num_envs, physics=physics, self_collision=self_collision
    )["envs"]
    if num_envs == 1:
        return Sim(client, env_ids[0], owns_client=True)
    return VecSim(client, env_ids, owns_client=True)


def linspace(start: float, stop: float, steps: int) -> np.ndarray:
    return np.linspace(start, stop, steps)
