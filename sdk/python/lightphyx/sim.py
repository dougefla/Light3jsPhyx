"""Synchronous, numpy-first facades over the wire protocol."""
from __future__ import annotations

import os
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple, Union

import numpy as np

from .client import CHANNEL_DEPTH, CHANNEL_RGB, CHANNEL_SEG, Binaries, Client
from .server import ServerProcess

Vec3 = Tuple[float, float, float]


@dataclass
class Joint:
    name: str
    type: str
    lower: float
    upper: float
    value: float


class ArticulatedObject:
    """Handle for one loaded URDF within a Sim."""

    def __init__(self, sim: "Sim", name: str, joints: List[Joint], dynamic: bool = False) -> None:
        self._sim = sim
        self.name = name
        self.joints: Dict[str, Joint] = {j.name: j for j in joints}
        #: True when loaded with ``dynamic=True`` (links are dynamic bodies).
        self.dynamic = dynamic

    def set_joint(self, name: str, value: float, immediate: bool = False) -> None:
        self._sim.set_joint(name, value, immediate=immediate, robot=self.name)

    def set_joints(self, values: Dict[str, float], immediate: bool = False) -> None:
        self._sim.set_joints(values, immediate=immediate, robot=self.name)

    def set_drive(self, name: str, stiffness: Optional[float] = None, damping: Optional[float] = None) -> None:
        """PD gains of one joint motor (dynamic robots only); stiffness 0 = passive."""
        self._sim.set_joint_drive(name, stiffness=stiffness, damping=damping, robot=self.name)

    def __repr__(self) -> str:
        return f"ArticulatedObject({self.name!r}, joints={list(self.joints)})"


class Camera:
    """Per-env camera handle; every call renders fresh pixels."""

    def __init__(self, sim: "Sim") -> None:
        self._sim = sim
        self._labels: Dict[int, str] = {}

    def capture(
        self,
        passes: Sequence[str] = ("rgb",),
        save: Optional[Dict[str, str]] = None,
        outline: bool = False,
    ) -> Dict[str, Any]:
        """Render several passes in ONE call.

        Returns {pass_name: ndarray, ...} plus 'seg_labels' when 'seg' is
        requested and 'files' when save={'dir': ..., 'prefix': ...} asked the
        server to encode PNG/npy to disk directly (no client round-trip cost
        beyond the arrays themselves).

        outline=True overlays CAD-style feature edges on the RGB pass only;
        seg and depth are byte-for-byte identical whether it is on or off.
        """
        params: Dict[str, Any] = {"env": self._sim.env_id, "passes": list(passes)}
        if save is not None:
            params["save"] = save
        if outline:
            params["outline"] = True
        result, binaries = self._sim._client.call_binary("render", **params)
        h, w = result["height"], result["width"]
        out: Dict[str, Any] = {p: _decode_pass(p, h, w, binaries) for p in passes}
        if result.get("seg_labels"):
            self._labels = {int(k): v for k, v in result["seg_labels"].items()}
            out["seg_labels"] = dict(self._labels)
        if result.get("files"):
            out["files"] = list(result["files"])
        return out

    def rgb(self, outline: bool = False) -> np.ndarray:
        return self.capture(["rgb"], outline=outline)["rgb"]

    def seg(self) -> np.ndarray:
        return self.capture(["seg"])["seg"]

    def depth(self) -> np.ndarray:
        return self.capture(["depth"])["depth"]

    def seg_labels(self) -> Dict[int, str]:
        if not self._labels:
            res = self._sim._client.call("seg_labels", env=self._sim.env_id)
            self._labels = {int(k): v for k, v in res["labels"].items()}
        return dict(self._labels)


def _channel(binaries: Binaries, channel: int) -> memoryview:
    for ch, payload in binaries:
        if ch == channel:
            return payload
    raise RuntimeError(f"missing channel {channel} in render response")


def _decode_pass(pass_name: str, h: int, w: int, binaries: Binaries, copy: bool = True) -> np.ndarray:
    """Wire bytes → ndarray. The buffers are owned by this response, so views
    are safe; copy=False lets a batched caller (np.stack) do the single copy."""
    if pass_name == "rgb":
        rgb = np.frombuffer(_channel(binaries, CHANNEL_RGB), np.uint8).reshape(h, w, 4)[..., :3]
        return rgb.copy() if copy else rgb
    if pass_name == "seg":
        # id = (r<<16)|(g<<8)|b; bytes arrive RGBA little-endian → one uint32 view
        # and three vector ops instead of an int32 upcast of the whole RGBA plane.
        u = np.frombuffer(_channel(binaries, CHANNEL_SEG), np.uint32).reshape(h, w)
        return (((u & 0xFF) << 16) | (u & 0xFF00) | ((u >> 16) & 0xFF)).astype(np.int32, copy=False)
    if pass_name == "depth":
        # Raw float32 LE metres (bit-exact); the buffer is ours → no copy needed.
        return np.frombuffer(_channel(binaries, CHANNEL_DEPTH), np.float32).reshape(h, w)
    raise ValueError(f"unknown pass {pass_name}")


class Sim:
    """One simulation environment (synchronous facade)."""

    def __init__(
        self,
        client: Client,
        env_id: int,
        server: Optional[ServerProcess] = None,
        owns_client: bool = False,
    ) -> None:
        self._client = client
        self.env_id = env_id
        self._server = server
        # True when this handle created the Client (launch/connect); False for
        # a vec[i] view that shares its parent's Client.
        self._owns_client = owns_client

    # -- scene ---------------------------------------------------------------

    def load_urdf(
        self,
        path: str,
        name: Optional[str] = None,
        pos: Optional[Vec3] = None,
        yaw: Optional[float] = None,
        *,
        dynamic: bool = False,
        stiffness: Optional[float] = None,
        damping: Optional[float] = None,
        density: Optional[float] = None,
    ) -> ArticulatedObject:
        """Load a URDF into this env.

        dynamic=False (default): links are kinematic — set_joint moves them
        exactly, nothing pushes back. dynamic=True: links are dynamic bodies
        joined by motored joints (Isaac-style articulation): set_joint drives
        a PD motor, link–link contacts are solved, so a drawer slid into a
        passive door pushes it open and a blocked joint stalls. stiffness /
        damping are the default motor gains (acceleration model, 1/s² and
        1/s; defaults 400 / 40), density (kg/m³, default 300) sets link mass
        from the collider bounding box. Per-joint gains: set_joint_drive().
        """
        params: Dict[str, Any] = {
            "env": self.env_id,
            "path": os.path.abspath(os.path.expanduser(path)),
            "name": name,
        }
        if pos is not None:
            params["pos"] = list(pos)
        if yaw is not None:
            params["yaw"] = yaw
        if dynamic:
            params["dynamic"] = True
            for k, v in (("stiffness", stiffness), ("damping", damping), ("density", density)):
                if v is not None:
                    params[k] = float(v)
        result = self._client.call("load_urdf", **params)
        joints = [Joint(j["name"], j["type"], j["lower"], j["upper"], j["value"]) for j in result["joints"]]
        return ArticulatedObject(self, result["name"], joints, dynamic=bool(result.get("dynamic", False)))

    def set_joint_drive(
        self,
        name: str,
        stiffness: Optional[float] = None,
        damping: Optional[float] = None,
        robot: Optional[str] = None,
    ) -> None:
        """Change the PD gains of a dynamic robot's joint motor.

        stiffness=0 makes the joint passive (moves only when pushed or by
        gravity). Raises for robots loaded without ``dynamic=True``.
        """
        params: Dict[str, Any] = {"env": self.env_id, "name": name, "robot": robot}
        if stiffness is not None:
            params["stiffness"] = float(stiffness)
        if damping is not None:
            params["damping"] = float(damping)
        self._client.call("set_joint_drive", **params)

    def add_ground(self, z: float = 0.0, visual: bool = True) -> None:
        self._client.call("add_ground", env=self.env_id, z=z, visual=visual)

    def spawn(self, shape: str, size: Sequence[float], pos: Vec3, **kw: Any) -> str:
        return self._client.call("spawn", env=self.env_id, shape=shape, size=list(size), pos=list(pos), **kw)["name"]

    def spawn_sphere(self, radius: float, pos: Vec3, **kw: Any) -> str:
        return self.spawn("sphere", [radius], pos, **kw)

    def spawn_box(self, size: Vec3, pos: Vec3, **kw: Any) -> str:
        return self.spawn("box", list(size), pos, **kw)

    def spawn_cylinder(self, radius: float, height: float, pos: Vec3, **kw: Any) -> str:
        return self.spawn("cylinder", [radius, height], pos, **kw)

    def remove_object(self, name: str) -> bool:
        return bool(self._client.call("remove_object", env=self.env_id, name=name)["removed"])

    def remove_robot(self, name: str) -> bool:
        """Unload a robot from this env (its links, seg ids and physics bodies)."""
        return bool(self._client.call("remove_robot", env=self.env_id, name=name)["removed"])

    # -- joints & stepping -----------------------------------------------------

    def joints(self, robot: Optional[str] = None) -> List[Joint]:
        params: Dict[str, Any] = {"env": self.env_id}
        if robot is not None:
            params["robot"] = robot
        return [
            Joint(j["name"], j["type"], j["lower"], j["upper"], j["value"])
            for j in self._client.call("list_joints", **params)
        ]

    def set_joint(
        self, name: str, value: float, immediate: bool = False, robot: Optional[str] = None
    ) -> None:
        params: Dict[str, Any] = {"env": self.env_id, "name": name, "value": value, "immediate": immediate}
        if robot is not None:
            params["robot"] = robot
        self._client.call("set_joint", **params)

    def set_joints(
        self, values: Dict[str, float], immediate: bool = False, robot: Optional[str] = None
    ) -> None:
        params: Dict[str, Any] = {"env": self.env_id, "values": values, "immediate": immediate}
        if robot is not None:
            params["robot"] = robot
        self._client.call("set_joints", **params)

    def step(self, dt: float = 1 / 60, n: int = 1) -> None:
        self._client.call("step", env=self.env_id, dt=dt, n=n)

    def settle(self, max_seconds: float = 5.0, dt: float = 1 / 60) -> None:
        """Step until all joints reach their targets (or the time budget ends)."""
        steps = int(max_seconds / dt)
        for _ in range(steps):
            self.step(dt)
            if self.state()["settled"]:
                return

    def contacts(self) -> List[Tuple[str, str]]:
        return [tuple(pair) for pair in self._client.call("contacts", env=self.env_id)["contacts"]]

    def self_contacts(self, robot: Optional[str] = None) -> List[Tuple[str, str]]:
        """Link pairs of one robot currently touching each other.

        Needs ``launch(self_collision=True)``; otherwise always ``[]``. Links
        joined directly by a joint (parent/child) are never reported. Names
        are bare link names (no ``link:robot/`` prefix); ``robot`` defaults to
        the first loaded robot.

        Detection only: links are kinematic, so the physics never stops a
        joint from driving into another link. To avoid interpenetration,
        back the joint off in your own control loop when this is non-empty.
        """
        res = self._client.call("self_contacts", env=self.env_id, robot=robot)
        return [tuple(pair) for pair in res["contacts"]]

    def state(self) -> Dict[str, Any]:
        return self._client.call("get_state", env=self.env_id)

    def reset(self) -> None:
        self._client.call("reset", env=self.env_id)

    # -- camera & capture ------------------------------------------------------

    def camera(
        self,
        pos: Optional[Vec3] = None,
        lookat: Optional[Vec3] = None,
        fov: Optional[float] = None,
        size: Optional[Tuple[int, int]] = None,
        near: Optional[float] = None,
        far: Optional[float] = None,
    ) -> Camera:
        # near/far default to an automatic scene-fit on the engine side; pass
        # them only to pin the depth clip range explicitly.
        params: Dict[str, Any] = {"env": self.env_id}
        if pos is not None:
            params["pos"] = list(pos)
        if lookat is not None:
            params["lookat"] = list(lookat)
        if fov is not None:
            params["fov"] = fov
        if size is not None:
            params["width"], params["height"] = size
        if near is not None:
            params["near"] = near
        if far is not None:
            params["far"] = far
        self._client.call("set_camera", **params)
        return Camera(self)

    def frame_camera(self, fill: float = 0.7) -> Camera:
        """Auto-place the camera to frame the loaded object."""
        self._client.call("frame_camera", env=self.env_id, fill=fill)
        return Camera(self)

    @contextmanager
    def record(
        self,
        path: str,
        fps: int = 30,
        camera: Optional[Camera] = None,
        outline: bool = False,
    ) -> Iterator[None]:
        self._client.call(
            "start_record", env=self.env_id, fps=fps, path=os.path.abspath(path), outline=outline
        )
        try:
            yield
        finally:
            self._client.call("stop_record", env=self.env_id)

    def close(self) -> None:
        try:
            self._client.call("destroy_envs", env=self.env_id)
        finally:
            # Close the client whenever this handle owns it (connect() too, not
            # just launch()) so its socket + reader thread don't leak.
            if self._owns_client:
                self._client.close()
            if self._server is not None:
                self._server.close()


class VecSim:
    """Vectorized facade over N envs (gym.vector style)."""

    def __init__(
        self,
        client: Client,
        env_ids: List[int],
        server: Optional[ServerProcess] = None,
        owns_client: bool = False,
    ) -> None:
        self._client = client
        self.env_ids = list(env_ids)
        self._server = server
        self._owns_client = owns_client
        self._size: Tuple[int, int] = (640, 480)

    def __len__(self) -> int:
        return len(self.env_ids)

    def __getitem__(self, i: int) -> Sim:
        return Sim(self._client, self.env_ids[i])

    def _broadcast(self, method: str, **params: Any) -> Any:
        return self._client.call(method, env=self.env_ids, **params)

    def load_urdf(self, path: str, **kw: Any) -> None:
        """Load the same URDF into every env (kwargs as Sim.load_urdf, e.g. dynamic=True)."""
        params: Dict[str, Any] = {"path": os.path.abspath(os.path.expanduser(path))}
        if kw.pop("dynamic", False):
            params["dynamic"] = True
        params.update({k: v for k, v in kw.items() if v is not None})
        self._broadcast("load_urdf", **params)

    def set_joint_drive(
        self,
        name: str,
        stiffness: Optional[float] = None,
        damping: Optional[float] = None,
        robot: Optional[str] = None,
    ) -> None:
        params: Dict[str, Any] = {"name": name, "robot": robot}
        if stiffness is not None:
            params["stiffness"] = float(stiffness)
        if damping is not None:
            params["damping"] = float(damping)
        self._broadcast("set_joint_drive", **params)

    def add_ground(self, z: float = 0.0, visual: bool = True) -> None:
        self._broadcast("add_ground", z=z, visual=visual)

    def spawn(self, shape: str, size: Sequence[float], pos: Vec3, **kw: Any) -> None:
        self._broadcast("spawn", shape=shape, size=list(size), pos=list(pos), **kw)

    def set_joint(
        self,
        name: str,
        values: Union[float, Sequence[float], np.ndarray],
        immediate: bool = False,
        robot: Optional[str] = None,
    ) -> None:
        vals = np.broadcast_to(np.asarray(values, dtype=float), (len(self),))
        extra = {} if robot is None else {"robot": robot}
        self._client.call_many(
            [
                ("set_joint", {"env": env, "name": name, "value": float(v), "immediate": immediate, **extra})
                for env, v in zip(self.env_ids, vals)
            ]
        )

    def set_joints(
        self,
        values: Dict[str, Union[float, Sequence[float], np.ndarray]],
        immediate: bool = False,
        robot: Optional[str] = None,
    ) -> None:
        per_env: List[Dict[str, float]] = [{} for _ in self.env_ids]
        for name, vs in values.items():
            arr = np.broadcast_to(np.asarray(vs, dtype=float), (len(self),))
            for i, v in enumerate(arr):
                per_env[i][name] = float(v)
        extra = {} if robot is None else {"robot": robot}
        self._client.call_many(
            [
                ("set_joints", {"env": env, "values": vals, "immediate": immediate, **extra})
                for env, vals in zip(self.env_ids, per_env)
            ]
        )

    def step(self, dt: float = 1 / 60, n: int = 1) -> None:
        self._broadcast("step", dt=dt, n=n)

    def frame_cameras(self, fill: float = 0.7) -> None:
        self._broadcast("frame_camera", fill=fill)

    def set_cameras(self, **params: Any) -> None:
        if "size" in params:
            params["width"], params["height"] = params.pop("size")
        self._broadcast("set_camera", **params)

    def capture(
        self,
        passes: Sequence[str] = ("rgb", "seg", "depth"),
        size: Optional[Tuple[int, int]] = None,
        save: Optional[Dict[str, str]] = None,
        outline: bool = False,
    ) -> Dict[str, np.ndarray]:
        """Render several passes on every env in ONE round trip per env.

        Returns {pass_name: stacked array (N, H, W[, 3])}. Prefer this over
        three render() calls: every pass of an env shares one GPU fence and one
        request, so it is ~2× the throughput of rendering the passes one by one.
        """
        if size is not None:
            self.set_cameras(size=size)
        requests = []
        for env in self.env_ids:
            params: Dict[str, Any] = {"env": env, "passes": list(passes)}
            if save is not None:
                params["save"] = save
            if outline:
                params["outline"] = True
            requests.append(("render", params))
        responses = self._client.call_many(requests)
        return {
            p: np.stack([_decode_pass(p, r["height"], r["width"], b, copy=False) for r, b in responses])
            for p in passes
        }

    def render(
        self,
        pass_name: str = "rgb",
        size: Optional[Tuple[int, int]] = None,
        save: Optional[Dict[str, str]] = None,
        outline: bool = False,
    ) -> np.ndarray:
        """Render one pass on every env → stacked array (N, H, W[, 3]).

        save={'dir': ...} asks the server to also encode files to disk per
        env (prefix defaults to 'env<gid>' server-side). outline=True overlays
        CAD edges on the rgb pass only. For several passes use capture().
        """
        return self.capture([pass_name], size=size, save=save, outline=outline)[pass_name]

    def reset(self, indices: Optional[Sequence[int]] = None) -> None:
        envs = self.env_ids if indices is None else [self.env_ids[i] for i in indices]
        self._client.call("reset", env=envs)

    def states(self) -> List[Dict[str, Any]]:
        res = self._broadcast("get_state")
        return [res["results"][str(env)] for env in self.env_ids]

    def contacts(self) -> List[List[Tuple[str, str]]]:
        """Per-env contact pairs, in env order."""
        res = self._broadcast("contacts")
        return [[tuple(p) for p in res["results"][str(env)]["contacts"]] for env in self.env_ids]

    def self_contacts(self, robot: Optional[str] = None) -> List[List[Tuple[str, str]]]:
        """Per-env self-collision link pairs (needs ``self_collision=True``)."""
        res = self._broadcast("self_contacts", robot=robot)
        return [[tuple(p) for p in res["results"][str(env)]["contacts"]] for env in self.env_ids]

    def close(self) -> None:
        try:
            self._client.call("destroy_envs", env=self.env_ids)
        finally:
            if self._owns_client:
                self._client.close()
            if self._server is not None:
                self._server.close()
