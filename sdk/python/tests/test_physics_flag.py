"""physics=False envs are kinematic: set_joint applies without step()."""
import os
from pathlib import Path

import lightphyx as lp

ASSET = Path(__file__).resolve().parents[3] / "tests" / "assets" / "mini_cabinet.urdf"


def test_kinematic_env_applies_joint_immediately():
    sim = lp.launch(headless=True, physics=False, rpc_timeout=60.0, startup_attempts=2)
    try:
        robot = sim.load_urdf(str(ASSET))
        joints = [j for j in sim.joints(robot.name) if j.type != "fixed"]
        assert joints, "asset must expose a movable joint"
        j = joints[0]
        target = j.lower + 0.5 * (j.upper - j.lower) if j.upper > j.lower else 0.3
        sim.set_joint(j.name, float(target))
        state = sim.state()
        key = j.name if j.name in state["joints"] else f"{robot.name}/{j.name}"
        assert abs(state["joints"][key] - target) < 1e-6
        # step()/contacts() are no-ops instead of errors
        sim.step(1 / 60)
        assert sim.contacts() == []
        rgb = sim.frame_camera().rgb()
        assert rgb.shape[2] == 3 and rgb.size > 0
    finally:
        sim.close()
