"""Issue #13 regression: SDK-launched servers must never orphan.

Two escape hatches are exercised in real child processes:
- atexit: parent exits normally without calling close()
- stdin parent-watch: parent is SIGKILLed (atexit never runs)
"""
import os
import subprocess
import sys
import time

CHILD = """
import lightphyx as lp
import os, signal, sys
sim = lp.launch(headless=True)
print(sim._server._proc.pid, flush=True)
{ending}
"""


def _wait_dead(pid: int, timeout: float = 20.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            os.kill(pid, 0)
        except (ProcessLookupError, PermissionError):
            return True
        time.sleep(0.3)
    return False


def _run_child_get_server_pid(ending: str) -> int:
    proc = subprocess.Popen(
        [sys.executable, "-c", CHILD.format(ending=ending)],
        stdout=subprocess.PIPE,
        text=True,
    )
    line = proc.stdout.readline().strip()
    server_pid = int(line)
    proc.wait(timeout=120)
    return server_pid


def test_server_exits_when_parent_forgets_close():
    server_pid = _run_child_get_server_pid("sys.exit(0)  # no sim.close()")
    assert _wait_dead(server_pid), "server survived a normal parent exit without close()"


def test_server_exits_when_parent_is_sigkilled():
    server_pid = _run_child_get_server_pid("os.kill(os.getpid(), signal.SIGKILL)")
    assert _wait_dead(server_pid), "server survived parent SIGKILL (stdin watch failed)"
