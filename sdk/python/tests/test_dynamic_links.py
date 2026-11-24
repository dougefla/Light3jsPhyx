"""dynamic=True: links are motored dynamic bodies; contacts push other links."""
import os
from pathlib import Path

import lightphyx as lp

ASSET = str(Path(__file__).resolve().parents[3] / "tests" / "assets" / "mini_fridge.urdf")
OUT = os.environ.get("LIGHTPHYX_OUT", "/tmp/lightphyx-test")


def test_dynamic_drawer_pushes_passive_door_open():
    sim = lp.launch(headless=True, self_collision=True, out_dir=OUT, rpc_timeout=60.0, startup_attempts=2)
    try:
        fridge = sim.load_urdf(ASSET, dynamic=True)
        assert fridge.dynamic
        fridge.set_drive("hinge", stiffness=0, damping=2)  # door swings freely
        sim.step(1 / 60, n=30)
        assert abs(sim.state()["joints"]["hinge"]) < 0.02

        fridge.set_joint("slide", -0.3)
        max_hinge = 0.0
        touched = False
        for _ in range(180):
            sim.step(1 / 60)
            if ("door", "drawer") in sim.self_contacts():
                touched = True
            max_hinge = max(max_hinge, sim.state()["joints"]["hinge"])
        assert touched
        assert max_hinge > 0.3, max_hinge
        assert sim.state()["joints"]["slide"] < -0.2
        # the drawer's kinematic tree follows the bodies, so rendering still works
        rgb = sim.frame_camera().rgb()
        assert rgb.shape[2] == 3
    finally:
        sim.close()


def test_kinematic_default_passes_through_the_door():
    sim = lp.launch(headless=True, out_dir=OUT, rpc_timeout=60.0, startup_attempts=2)
    try:
        fridge = sim.load_urdf(ASSET)
        assert not fridge.dynamic
        fridge.set_joint("slide", -0.3)
        sim.settle()
        j = sim.state()["joints"]
        assert abs(j["slide"] + 0.3) < 1e-6 and abs(j["hinge"]) < 1e-9  # nothing pushed back
        try:
            fridge.set_drive("hinge", stiffness=0)
        except lp.RpcError:
            pass
        else:
            raise AssertionError("set_drive must fail for kinematic robots")
    finally:
        sim.close()
