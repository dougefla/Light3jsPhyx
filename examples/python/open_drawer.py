"""The DESIGN.md §3.4 example: load a cabinet, drop a ball into the open
drawer, film the drawer closing, then export segmentation/depth."""
import sys

import lightphyx as lp

URDF = sys.argv[1] if len(sys.argv) > 1 else "tests/assets/mini_cabinet.urdf"

sim = lp.launch(headless=True, out_dir="out")
try:
    obj = sim.load_urdf(URDF)
    print("joints:", obj.joints)
    sim.add_ground(z=0.0, visual=False)

    first = next(iter(obj.joints.values()))
    far = first.lower if abs(first.lower) > abs(first.upper) else first.upper

    cam = sim.frame_camera()
    with sim.record("out/open_drawer.mp4", fps=30):
        for q in lp.linspace(0, far, 45):
            obj.set_joint(first.name, float(q))
            sim.step(1 / 30)
        sim.spawn_sphere(0.03, pos=(0.0, -0.35, 1.0), name="ball")
        sim.step(1 / 30, n=30)
        for q in lp.linspace(far, 0, 45):
            obj.set_joint(first.name, float(q))
            sim.step(1 / 30)

    rgb, seg, depth = cam.rgb(), cam.seg(), cam.depth()
    print("rgb", rgb.shape, "| seg ids:", sorted(set(seg.flatten().tolist()))[:10])
    print("labels:", cam.seg_labels())
    print("depth range:", float(depth[depth > 0].min()), "-", float(depth.max()), "m")
    print("contacts:", sim.contacts())
    print("video: out/open_drawer.mp4")
finally:
    sim.close()
