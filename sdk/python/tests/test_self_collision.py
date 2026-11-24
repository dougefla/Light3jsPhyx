"""self_collision=True: link-link contacts flow through server + SDK.

mini_cabinet has one movable pair (base/drawer) joined by a joint, so the
same robot never self-collides. Two copies loaded at the same pose do cross:
their links show up in contacts(); self_contacts() stays per-robot.
"""
import os
from pathlib import Path

import lightphyx as lp

ASSET = str(Path(__file__).resolve().parents[3] / "tests" / "assets" / "mini_cabinet.urdf")
OUT = os.environ.get("LIGHTPHYX_OUT", "/tmp/lightphyx-test")


def _link_pairs(pairs):
    return [p for p in pairs if p[0].startswith("link:") and p[1].startswith("link:")]


def test_self_collision_reports_link_pairs():
    sim = lp.launch(headless=True, self_collision=True, out_dir=OUT, rpc_timeout=60.0, startup_attempts=2)
    try:
        a = sim.load_urdf(ASSET, name="a")
        sim.step(1 / 60, n=3)
        # base/drawer are joint-adjacent → filtered even though they overlap
        assert sim.self_contacts(a.name) == []
        assert _link_pairs(sim.contacts()) == []

        sim.load_urdf(ASSET, name="b", pos=(0.1, 0.0, 0.0))  # overlaps robot "a"
        sim.step(1 / 60, n=3)
        cross = _link_pairs(sim.contacts())
        assert cross, "overlapping robots must report link-link contacts"
        assert all({p[0].split(":")[1].split("/")[0], p[1].split(":")[1].split("/")[0]} == {"a", "b"} for p in cross)
        assert sim.self_contacts("a") == [] and sim.self_contacts("b") == []

        sim.remove_robot("b")
        sim.step(1 / 60, n=2)
        assert _link_pairs(sim.contacts()) == []
    finally:
        sim.close()


def test_default_env_ignores_link_pairs():
    sim = lp.launch(headless=True, out_dir=OUT, rpc_timeout=60.0, startup_attempts=2)
    try:
        sim.load_urdf(ASSET, name="a")
        sim.load_urdf(ASSET, name="b", pos=(0.1, 0.0, 0.0))
        sim.step(1 / 60, n=3)
        assert _link_pairs(sim.contacts()) == []
        assert sim.self_contacts() == []
    finally:
        sim.close()
