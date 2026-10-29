"""Read-only runtime probe; run with Isaac Sim's python.bat."""
import json
import importlib.util
import sys
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 960, "height": 640,
                     "renderer": "RayTracedLighting", "anti_aliasing": 0})
try:
    import omni.kit.app
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.sensors.physx")
    from isaacsim.sensors.physx import _range_sensor
    import numpy as np
    print("NAV_PROBE " + json.dumps({
        "python": sys.version, "numpy": np.__version__,
        "isaaclab": bool(importlib.util.find_spec("isaaclab")),
        "rclpy": bool(importlib.util.find_spec("rclpy")),
        "physx_lidar": bool(_range_sensor.acquire_lidar_sensor_interface()),
        "pillow": bool(importlib.util.find_spec("PIL")),
        "scipy": bool(importlib.util.find_spec("scipy")),
        "matplotlib": bool(importlib.util.find_spec("matplotlib")),
    }), flush=True)
finally:
    app.close()
