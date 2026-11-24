"""Launches and supervises a local light-3js-phyx server process."""
from __future__ import annotations

import atexit
import os
import socket
import subprocess
import time
import urllib.request
from pathlib import Path
from typing import List, Optional, Tuple


def _find_cli() -> List[str]:
    env = os.environ.get("LIGHTPHYX_CLI")
    if env:
        return ["node", env] if env.endswith(".js") else [env]
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "packages" / "server" / "dist" / "cli.js"
        if candidate.exists():
            return ["node", str(candidate)]
    return ["npx", "light-3js-phyx"]


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _positive_int(value: object, what: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{what} must be a positive integer, got {value!r}")
    return value


class ServerProcess:
    def __init__(
        self,
        pool: Optional[Tuple[int, int, int]] = None,
        out_dir: Optional[str] = None,
        port: Optional[int] = None,
        headless: bool = True,
        warm_envs: Optional[int] = None,
        browsers: Optional[int] = None,
    ) -> None:
        self.port = port or _free_port()
        cmd = _find_cli() + ["serve", "--port", str(self.port), "--exit-on-stdin-close"]
        if pool is not None:
            cmd += ["--pool", ",".join(str(x) for x in pool)]
        if browsers is not None:
            # Overrides B in `pool` keeping its capacity B×P×E; without
            # `pool` the server spreads warm_envs over them (E = ⌈warm/B⌉).
            cmd += ["--browsers", str(_positive_int(browsers, "browsers"))]
        if out_dir is not None:
            cmd += ["--out", out_dir]
        if not headless:
            cmd += ["--headed"]  # watch-along mode: launch visible browsers
        if warm_envs:
            # Browsers start launching before we even finish connecting.
            cmd += ["--warm-envs", str(int(warm_envs))]
        # stdin is a pipe: if this Python process dies (even SIGKILL), the OS
        # closes it and the server shuts itself down (issue #13).
        self._proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=None
        )
        # Belt-and-braces for normal interpreter exits that forgot close().
        atexit.register(self.close)
        self._wait_ready()

    @property
    def url(self) -> str:
        return f"ws://127.0.0.1:{self.port}/ws"

    def _wait_ready(self, timeout: float = 30.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._proc.poll() is not None:
                raise RuntimeError(f"server exited with code {self._proc.returncode}")
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{self.port}/", timeout=1)
                return
            except Exception:
                time.sleep(0.2)
        # Do not leave a half-started server behind on timeout.
        self.close()
        raise TimeoutError("server did not become ready in time")

    def close(self) -> None:
        if self._proc.poll() is None:
            try:
                if self._proc.stdin:
                    self._proc.stdin.close()
            except Exception:
                pass
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
