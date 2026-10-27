"""Open the exported scene in the actual Isaac Sim GUI, without rerunning tests."""
import argparse
from pathlib import Path
from isaacsim import SimulationApp

p = argparse.ArgumentParser()
p.add_argument("--scene", type=Path, default=Path(__file__).resolve().parents[2]/"wheeled_navigation/artifacts/isaac_demo/navigation_scene.usda")
p.add_argument("--eye", nargs=3, type=float, default=[11.,-14.,15.])
p.add_argument("--target", nargs=3, type=float, default=[0.,0.,0.])
p.add_argument("--cutaway-ceiling", action="store_true", help="Session-only ceiling visibility override")
a = p.parse_args()
if not a.scene.is_file():
    p.error(f"Run the demo first; scene not found: {a.scene}")
app = SimulationApp({"headless":False, "width":1280, "height":800, "renderer":"RayTracedLighting"})
try:
    import numpy as np
    import omni.timeline
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.utils.viewports import set_camera_view
    enable_extension("isaacsim.sensors.physx")
    open_stage(str(a.scene.resolve()))
    if a.cutaway_ceiling:
        import omni.usd
        from pxr import UsdGeom
        stage=omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        for prim in stage.Traverse():
            if 'ceiling' in prim.GetName().lower() and prim.IsA(UsdGeom.Imageable):
                UsdGeom.Imageable(prim).MakeInvisible()
    set_camera_view(eye=np.array(a.eye), target=np.array(a.target))
    omni.timeline.get_timeline_interface().play()
    print("NAV_SCENE_READY " + str(a.scene.resolve()), flush=True)
    while app.is_running():
        app.update()
finally:
    app.close()
