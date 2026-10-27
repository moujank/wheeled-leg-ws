"""Isaac Sim 4.5 sensor mapping and goal planning validation.

Run with Isaac Sim python.bat. Motion is an explicitly labelled kinematic
demonstration; existing training and MuJoCo runners are not changed.
"""
from __future__ import annotations
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

MODULES = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(MODULES / "wheeled_navigation"))

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=MODULES / "wheeled_navigation" / "artifacts" / "isaac_demo")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--hold", action="store_true", help="Keep the GUI open after validation")
parser.add_argument("--tour-step", type=float, default=0.30)
parser.add_argument("--goal", nargs=2, type=float, default=[5.4, 3.5])
parser.add_argument("--start", nargs=2, type=float, default=[-5.5, -3.5])
parser.add_argument("--ros2", action="store_true", help="Publish scans, ideal odometry, map and path")
parser.add_argument("--external-slam", action="store_true", help="Leave /map and map->odom exclusively to SLAM Toolbox")
parser.add_argument("--smoke", action="store_true", help="Only validate one real PhysX scan")
args = parser.parse_args()
if args.hold and not args.gui:
    parser.error("--hold requires --gui")
if args.external_slam and not args.ros2:
    parser.error("--external-slam requires --ros2")
if args.tour_step <= 0 or args.tour_step > 0.5:
    parser.error("--tour-step must be in (0, 0.5]")

# Isaac 4.5's Windows bridge checks DLL loading in a child process. Configure
# this process before SimulationApp; no persistent machine environment changes.
_ros_dll_handle = None
if args.ros2 and sys.platform == "win32" and not os.environ.get("ROS_DISTRO"):
    isaac_root = Path(os.environ.get("ISAAC_PATH", Path(sys.executable).parents[2]))
    ros_lib = isaac_root / "exts/isaacsim.ros2.bridge/humble/lib"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["PATH"] = str(ros_lib) + os.pathsep + os.environ.get("PATH", "")
    _ros_dll_handle = os.add_dll_directory(str(ros_lib))

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 800,
                     "renderer": "RayTracedLighting", "anti_aliasing": 0})


def main():
    import numpy as np
    import omni.kit.commands
    import omni.usd
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    from scan_nav.core import OccupancyMapper, follow_path, path_length
    from scan_nav.scene_spec import BOXES, tour_samples, clearance, path_clearance
    from scan_nav.report import write_report

    enable_extension("isaacsim.sensors.physx")
    from isaacsim.sensors.physx import _range_sensor
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    world = World(stage_units_in_meters=1.0, physics_dt=1/60, rendering_dt=1/60)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    def cube(path, centre, size, color, collision=True):
        obj = UsdGeom.Cube.Define(stage, path)
        obj.CreateSizeAttr(1.0)
        obj.AddTranslateOp().Set(Gf.Vec3d(*centre))
        obj.AddScaleOp().Set(Gf.Vec3f(*size))
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        return obj

    cube("/World/Floor", (0, 0, -0.08), (15.0, 11.0, 0.16), (0.16, 0.20, 0.25))
    for name, centre, size in BOXES:
        cube("/World/Environment/" + name, centre, size,
             (0.86, 0.46, 0.16) if "crate" in name else (0.60, 0.68, 0.74))
    dome = UsdLux.DomeLight.Define(stage, "/World/Light")
    dome.CreateIntensityAttr(1500)

    root = UsdGeom.Xform.Define(stage, "/World/Robot")
    position_op = root.AddTranslateOp()
    orientation_op = root.AddOrientOp()
    asset = MODULES / "wheeled_description/wheeled_leg_v2/urdf/wheeled_leg_v2.usd"
    model = UsdGeom.Xform.Define(stage, "/World/Robot/Model")
    if not asset.is_file():
        raise FileNotFoundError(f"Robot USD missing: {asset}")
    model.GetPrim().GetReferences().AddReference(str(asset))
    translate = model.GetPrim().GetAttribute("xformOp:translate")
    if translate:
        translate.Set(Gf.Vec3d(0, 0, 0.45))
    else:
        model.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.45))
    app.update()
    # Imported robot visuals are instanceable. De-instance in the new scene's
    # override layer so meshes can be inspected and demo colliders disabled.
    for _ in range(4):
        instances = [p for p in Usd.PrimRange(model.GetPrim()) if p.IsInstance()]
        if not instances:
            break
        for prim in instances:
            prim.SetInstanceable(False)
    mesh_count = 0
    for prim in Usd.PrimRange(model.GetPrim()):
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
    if mesh_count == 0:
        raise RuntimeError("Robot reference has no loaded meshes; check LFS and USD dependencies")
    # Small visible sensor housing, no collider so it cannot self-occlude.
    cube("/World/Robot/LidarHousing", (0, 0, 0.86), (0.14, 0.14, 0.08), (0.1, 0.8, 0.65), False)
    ok, lidar = omni.kit.commands.execute(
        "RangeSensorCreateLidar", path="/World/Robot/Lidar", parent=None,
        min_range=0.15, max_range=20.0, draw_points=True, draw_lines=False,
        horizontal_fov=360.0, vertical_fov=1.0, horizontal_resolution=1.0,
        vertical_resolution=1.0, rotation_rate=0.0, high_lod=False, yaw_offset=0.0)
    if not ok:
        raise RuntimeError("PhysX lidar creation failed")
    lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, 0.92))
    sensor = _range_sensor.acquire_lidar_sensor_interface()
    sensor_path = str(lidar.GetPath())
    set_camera_view(eye=np.array([11.0, -14.0, 15.0]), target=np.array([0, 0, 0]))
    mapper = OccupancyMapper()
    scans, poses, stamps, angle_history = [], [], [], []
    endpoint_errors = []
    bridge = None
    if args.ros2:
        enable_extension("isaacsim.ros2.bridge")
        for _ in range(5):
            app.update()
        from ros2_sensor_bridge import Ros2SensorBridge
        bridge = Ros2SensorBridge(external_slam=args.external_slam)

    def set_pose(pose):
        position_op.Set(Gf.Vec3d(float(pose[0]), float(pose[1]), 0.0))
        orientation_op.Set(Gf.Quatf(math.cos(pose[2]/2), 0, 0, math.sin(pose[2]/2)))

    def read_scan(pose, record=True):
        # Two physics steps after each pose write avoid reusing the old scan.
        for _ in range(3):
            world.step(render=True)
        raw = np.array(sensor.get_depth_data(sensor_path), copy=True)
        azimuth = np.array(sensor.get_azimuth_data(sensor_path), copy=True).reshape(-1)
        zenith = np.array(sensor.get_zenith_data(sensor_path), copy=True).reshape(-1)
        if raw.size == 0 or azimuth.size == 0:
            raise RuntimeError("PhysX returned an empty laser scan")
        raw = raw.reshape(len(azimuth), -1)
        column = int(np.argmin(abs(zenith - math.pi/2))) if len(zenith) == raw.shape[1] else raw.shape[1]//2
        ranges = raw[:, column].astype(float) * 20.0 / 65535.0
        # PhysX azimuth is in radians in Isaac Sim 4.5.
        if np.max(abs(azimuth)) > 2 * math.pi + 0.1:
            raise RuntimeError("Unexpected lidar angular units; refusing to build a misleading map")
        mapper.integrate(pose, azimuth, ranges, 20.0)
        if record:
            scans.append(ranges.copy())
            poses.append(tuple(pose))
            stamps.append(world.current_time)
            angle_history.append(azimuth.copy())
            hits = ranges < 19.99
            ex = pose[0] + ranges[hits] * np.cos(pose[2] + azimuth[hits])
            ey = pose[1] + ranges[hits] * np.sin(pose[2] + azimuth[hits])
            endpoint_errors.extend(clearance(float(x), float(y)) for x, y in zip(ex[::8], ey[::8]))
        if bridge:
            bridge.publish(world.current_time, pose, azimuth, ranges)
        return azimuth, ranges

    set_pose((*args.start, 0.0))
    world.reset()
    for _ in range(20):
        world.step(render=True)
    first_angles, first_ranges = read_scan((*args.start, 0.0))
    probe = {"beams": int(len(first_ranges)), "min_range": float(first_ranges.min()),
             "max_range": float(first_ranges.max()), "angles": [float(first_angles.min()), float(first_angles.max())],
             "robot_mesh_count": mesh_count}
    print("SENSOR_PROBE " + json.dumps(probe), flush=True)
    if args.smoke:
        (out / "sensor_probe.json").write_text(json.dumps(probe, indent=2), encoding="utf-8")
        if bridge:
            bridge.close()
        return

    started = time.perf_counter()
    progress = []
    for i, pose in enumerate(tour_samples(args.tour_step)):
        if not app.is_running():
            raise RuntimeError("Simulation closed during mapping")
        set_pose(pose)
        read_scan(pose)
        if i % 20 == 0:
            progress.append({"scan": mapper.scan_count, "observed_cells": int(mapper.observed.sum())})
            print(f"MAPPING scan={mapper.scan_count} known={mapper.observed.sum()}", flush=True)
    if np.percentile(endpoint_errors, 95) > 0.10:
        raise AssertionError("Laser endpoints do not align with scene geometry; check coordinate/time conventions")
    mapper.save(out)
    np.savez_compressed(out / "scans.npz", ranges=np.array(scans), angles=np.array(angle_history),
                        poses=np.array(poses), times=np.array(stamps), max_range=20.0)

    radius = 0.45
    cases = [(args.start, args.goal), ([-5.5, 3.5], [5.5, -3.5]), ([-1.5, -3.5], [5.5, 3.5])]
    planned = []
    for start, goal in cases:
        route = mapper.plan(start, goal, radius)
        gap = path_clearance(route)
        if gap < radius:
            raise AssertionError(f"Independent ground-truth clearance failed: {gap}")
        planned.append({"start": list(start), "goal": list(goal), "path": route,
                        "length_m": path_length(route), "min_clearance_m": gap})
    for bad_goal in [(2.8, 1.9), (0, 4), (20, 20)]:
        try:
            mapper.plan(args.start, bad_goal, radius)
        except ValueError:
            continue
        raise AssertionError(f"Blocked/out-of-map goal accepted: {bad_goal}")

    route = planned[0]["path"]
    curve = UsdGeom.BasisCurves.Define(stage, "/World/PlannedPath")
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr([len(route)])
    curve.CreatePointsAttr([Gf.Vec3f(x, y, 0.07) for x, y in route])
    curve.CreateWidthsAttr([0.055])
    curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curve.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.9, 0.65)])
    pose, index, reached = (*args.start, 0.0), 0, False
    trajectory, controls = [pose], []
    set_pose(pose)
    if bridge:
        bridge.publish_map(mapper, world.current_time)
        bridge.publish_path(route, world.current_time)
    for step in range(6000):
        next_pose, index, reached, cmd = follow_path(pose, route, index)
        if path_clearance([pose[:2], next_pose[:2]]) < radius:
            raise AssertionError("Kinematic driver violates robot footprint")
        pose = next_pose
        set_pose(pose)
        trajectory.append(pose)
        controls.append(cmd)
        for _ in range(6):
            world.step(render=(step % 3 == 0))
        if bridge:
            bridge.spin_once()
        if step % 20 == 0:
            read_scan(pose, record=False)
        if reached:
            break
    if not reached:
        raise AssertionError("Demo follower did not reach the goal before timeout")
    error = math.dist(pose[:2], args.goal)
    metrics = {
        "status": "passed", "simulator": "Isaac Sim 4.5.0", "sensor": "PhysX 360-degree lidar",
        "mapping": "log-odds occupancy integration from real simulator scans",
        "odometry": "ideal simulator pose", "slam_loop_closure": False,
        "motion": "kinematic unicycle demo; RL policy not executed",
        "robot_asset": str(asset), "robot_mesh_count": mesh_count,
        "map_source": "laser measurements only; scene boxes never supplied to mapper/planner",
        "scan_count": len(scans), "beams_per_scan": len(first_ranges),
        "known_cells": int(mapper.observed.sum()), "occupied_cells": int(mapper.occupied().sum()),
        "free_cells": int(mapper.free().sum()), "robot_radius_m": radius,
        "endpoint_distance_median_m": float(np.median(endpoint_errors)),
        "endpoint_distance_p95_m": float(np.percentile(endpoint_errors, 95)),
        "goal_reached": reached, "goal_error_m": error, "follower_steps": len(controls),
        "trajectory_min_clearance_m": path_clearance([p[:2] for p in trajectory]),
        "blocked_goal_rejections": 3, "planning_cases": planned,
        "mapping_progress": progress, "wall_time_s": time.perf_counter() - started,
        "ros2_enabled": args.ros2,
    }
    (out / "trajectory.json").write_text(json.dumps({"poses": trajectory, "commands_vx_wz": controls}), encoding="utf-8")
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    # Export a viewable local USD scene with the robot at its starting location.
    set_pose((*args.start, 0.0))
    stage.GetRootLayer().Export(str(out / "navigation_scene.usda"))
    for _ in range(20):
        world.step(render=True)
    if bridge:
        # Capture the actual advertised graph after DDS discovery.
        for _ in range(10):
            bridge.spin_once()
        graph = dict(bridge.node.get_topic_names_and_types())
        (out / "ros2_topics.json").write_text(json.dumps(graph, indent=2), encoding="utf-8")
        if bridge.received["scan"] == 0 or bridge.received["odom"] == 0:
            raise AssertionError("ROS 2 scan/odometry messages were not received by diagnostic subscriptions")
        if not args.external_slam and bridge.received["map"] == 0:
            raise AssertionError("ROS 2 map was not received by diagnostic subscription")
        metrics["ros2_received"] = dict(bridge.received)
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        task = capture_viewport_to_file(get_active_viewport(), file_path=str(out / "isaac_scene.png"))
        for _ in range(30):
            app.update()
    except Exception as exc:
        metrics["screenshot_error"] = str(exc)
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    write_report(out, mapper, planned, trajectory, scans, poses, angle_history, progress)
    print("NAV_VALIDATION " + json.dumps({k:v for k,v in metrics.items() if k not in ("planning_cases", "mapping_progress")}), flush=True)
    if bridge:
        bridge.close()
    if args.hold:
        while app.is_running():
            world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()
