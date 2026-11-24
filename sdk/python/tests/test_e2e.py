"""End-to-end SDK test against a real server + headless Chromium.

Requires: `npm run build` at the repo root and Playwright Chromium installed.
Uses the self-contained primitive URDF from tests/assets.
"""
import os
from pathlib import Path

import numpy as np
import pytest

import lightphyx as lp

URDF = str(Path(__file__).resolve().parents[3] / "tests" / "assets" / "mini_cabinet.urdf")


@pytest.fixture(scope="module")
def sim():
    sim = lp.launch(headless=True, out_dir=os.environ.get("LIGHTPHYX_OUT", "/tmp/lightphyx-test"))
    yield sim
    sim.close()


def test_load_and_joints(sim):
    obj = sim.load_urdf(URDF)
    assert "slide" in obj.joints
    j = obj.joints["slide"]
    assert (j.type, j.lower, j.upper) == ("prismatic", -0.3, 0.0)


def test_joint_chases_target_under_velocity_limit(sim):
    sim.set_joint("slide", -0.3)
    sim.step(1 / 60, n=5)  # 5 substeps × min(1·dt, 0.02) → far from target
    partial = sim.state()["joints"]["slide"]
    assert -0.3 < partial < 0.0
    sim.settle()
    assert abs(sim.state()["joints"]["slide"] - (-0.3)) < 1e-9


def test_render_passes(sim):
    sim.add_ground(visual=False)
    cam = sim.frame_camera()
    rgb = cam.rgb()
    seg = cam.seg()
    depth = cam.depth()
    assert rgb.shape == (480, 640, 3) and rgb.dtype == np.uint8
    assert seg.shape == (480, 640) and seg.dtype == np.int32
    assert depth.shape == (480, 640) and depth.dtype == np.float32

    labels = cam.seg_labels()
    names = set(labels.values())
    assert {"mini_cabinet/base", "mini_cabinet/drawer"} <= names
    drawer_id = next(k for k, v in labels.items() if v.endswith("drawer"))
    assert (seg == drawer_id).sum() > 100  # drawer visible
    fg = seg > 0
    assert np.array_equal(fg, depth > 0)  # pixel-exact pass alignment
    assert 0.1 < depth[fg].min() < depth.max() < 100


def test_spawn_and_physics(sim):
    sim.spawn_sphere(0.04, pos=(0.0, -0.4, 1.2), name="ball")
    sim.step(1 / 60, n=180)
    ball = sim.state()["objects"]["ball"]
    assert ball["pos"][2] < 1.0  # it fell
    assert any("ball" in a or "ball" in b for a, b in sim.contacts())


def test_record(sim, tmp_path):
    out = tmp_path / "clip.mp4"
    with sim.record(str(out), fps=30):
        sim.set_joint("slide", 0.0)
        sim.step(1 / 30, n=15)
    produced = list(tmp_path.glob("clip.*"))  # .mp4, or .h264 without ffmpeg
    assert produced and produced[0].stat().st_size > 500


def test_reset(sim):
    sim.reset()
    state = sim.state()
    assert state["joints"]["slide"] == 0.0
    assert state["objects"] == {}


def test_multi_robot_addressing_and_remove(sim):
    # Issue #9 / #11: robot-addressed joints and protocol-level robot unload.
    obj2 = sim.load_urdf(URDF, name="second")
    obj2.set_joint("slide", -0.2, immediate=True)  # routes robot="second"
    joints = sim.state()["joints"]
    assert joints["second/slide"] == -0.2
    assert joints["mini_cabinet/slide"] == 0.0
    assert {j.name for j in sim.joints(robot="second")} == {"slide"}

    assert sim.remove_robot("second") is True
    labels = sim.frame_camera().seg_labels()
    assert not any(v.startswith("second/") for v in labels.values())
    assert "slide" in sim.state()["joints"]  # back to single-robot key form


def test_capture_multi_pass_and_server_save(sim, tmp_path):
    # Issue #10: one RPC for several passes + server-side PNG/npy export.
    cam = sim.frame_camera()
    out = cam.capture(["rgb", "seg", "depth"], save={"dir": str(tmp_path), "prefix": "cap"})
    assert out["rgb"].shape[2] == 3
    assert out["seg"].dtype == np.int32
    assert out["depth"].dtype == np.float32
    assert "seg_labels" in out
    names = {Path(f).name for f in out["files"]}
    assert {"cap_rgb.png", "cap_seg.png", "cap_seg_labels.json", "cap_depth.png", "cap_depth.npy"} <= names
    for f in out["files"]:
        assert Path(f).stat().st_size > 0


def test_vec_api():
    vec = lp.launch_vec(num_envs=4, pool=(1, 1, 4))
    try:
        vec.load_urdf(URDF)
        vec.add_ground(visual=False)
        vec.frame_cameras()
        vec.set_joint("slide", lp.linspace(0, -0.3, 4))
        vec.step(1 / 30, n=30)
        states = vec.states()
        vals = [s["joints"]["slide"] for s in states]
        assert vals[0] == 0.0 and abs(vals[3] + 0.3) < 1e-9
        segs = vec.render("seg", size=(320, 240))
        assert segs.shape == (4, 240, 320)
        assert all((segs[i] > 0).sum() > 100 for i in range(4))
        env2 = vec[2]
        assert -0.3 < env2.state()["joints"]["slide"] < 0.0
    finally:
        vec.close()


def test_vec_browsers_spreads_envs():
    # browsers=2 alone: 5 envs → E = ceil(5/2) = 3, one page per browser.
    vec = lp.launch_vec(num_envs=5, browsers=2, physics=False)
    try:
        pages = vec._client.call("pool_stats")["pages"]
        assert sorted(p["envs"] for p in pages) == [2, 3]
    finally:
        vec.close()


def test_vec_browsers_rejects_bad_value():
    with pytest.raises(ValueError):
        lp.launch_vec(num_envs=2, browsers=0)
