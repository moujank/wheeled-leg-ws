"""Real Isaac PhysX scans in a local Code-as-Room asset; scripted pose baseline.

The known-geometry tour is only a collection itinerary. A fresh mapper receives
only measured ranges and ideal poses; the goal planner receives only that map.
This does not execute a policy or validate the robot's dynamic/full 3D footprint.
"""
from __future__ import annotations
import argparse
import json
import math
import sys
import time
from pathlib import Path

MODULES = Path(__file__).resolve().parents[2]
WORKSPACE = MODULES.parent
sys.path.insert(0, str(MODULES/'wheeled_navigation'))
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--scene', type=Path, default=WORKSPACE/'research/scene_audit/code_as_room/scene.usda')
p.add_argument('--geometry', type=Path, default=WORKSPACE/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json')
p.add_argument('--output',type=Path,default=MODULES/'wheeled_navigation/artifacts/code_as_room_demo')
p.add_argument('--gui',action='store_true')
p.add_argument('--probe',action='store_true')
p.add_argument('--tour-step',type=float,default=.12)
a=p.parse_args()
if not 0<a.tour_step<=.3: p.error('--tour-step must be in (0, .3]')
for source in [a.scene,a.geometry]:
    if not source.is_file(): raise FileNotFoundError(source)

from isaacsim import SimulationApp
app=SimulationApp({'headless':not a.gui,'width':1920,'height':1280,
    'renderer':'RayTracedLighting','anti_aliasing':0,'fast_shutdown':True,
    'extra_args':['--/plugins/carb.tasking.plugin/threadCount=4',
                  '--/rtx/hydra/enableMultithreading=false']})

def main():
    import numpy as np
    import omni.usd
    import omni.kit.commands
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdLux
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    from scan_nav.core import OccupancyMapper,path_length
    from scan_nav.code_room import MAP_SPEC,load_geometry,collection_tour,sample_tour,path_clearance,clearance

    out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    (out/'metrics.json').write_text(json.dumps({'status':'running'}),encoding='utf-8')
    # Put every new edit in a separate top layer, preserving the room source.
    wrapper=out/'navigation_scene.usda'
    layer=Usd.Stage.CreateInMemory()
    layer.GetRootLayer().subLayerPaths=[a.scene.resolve().as_posix()]
    layer.GetRootLayer().Export(str(wrapper))
    open_stage(str(wrapper))
    stage=omni.usd.get_context().get_stage()
    # Stage metadata must be authored on this wrapper too: a bare root layer
    # otherwise falls back to Y-up, rotating new lidar scan planes vertically.
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    import omni.timeline
    omni.timeline.get_timeline_interface().stop()
    for _ in range(10):app.update()
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    geometry=load_geometry(a.geometry)
    tour,accepted,rejected=collection_tour(geometry,radius=.22)
    poses=list(sample_tour(tour,a.tour_step))
    enable_extension('isaacsim.sensors.physx')
    from isaacsim.sensors.physx import _range_sensor

    robot=UsdGeom.Xform.Define(stage,'/World/Robot')
    trans=robot.AddTranslateOp(); orient=robot.AddOrientOp()
    model=UsdGeom.Xform.Define(stage,'/World/Robot/Model')
    asset=WORKSPACE/'research/scene_audit/code_as_room/robot_visual.usdc'
    if not asset.is_file():raise FileNotFoundError('Run research/prepare_robot_visual.py first')
    model.GetPrim().GetReferences().AddReference(asset.as_posix())
    app.update()
    for _ in range(5):
        instances=[q for q in Usd.PrimRange(model.GetPrim()) if q.IsInstance()]
        if not instances: break
        for q in instances:q.SetInstanceable(False)
    # Centre the visual AABB and place the default-pose mesh on the floor.
    bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],useExtentsHint=True)
    bounds=bbox.ComputeLocalBound(model.GetPrim()).ComputeAlignedRange()
    vmin,vmax=np.array(bounds.GetMin()),np.array(bounds.GetMax())
    offset=Gf.Vec3d(-float((vmin[0]+vmax[0])/2),-float((vmin[1]+vmax[1])/2),-float(vmin[2])+.015)
    attr=model.GetPrim().GetAttribute('xformOp:translate')
    if attr:attr.Set(offset)
    else:model.AddTranslateOp().Set(offset)
    meshes=0
    for q in Usd.PrimRange(model.GetPrim()):
        meshes+=int(q.IsA(UsdGeom.Mesh))
        if q.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(q).CreateRigidBodyEnabledAttr(False)
        if q.HasAPI(UsdPhysics.ArticulationRootAPI):q.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if q.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
        if q.IsA(UsdPhysics.Joint):UsdPhysics.Joint(q).CreateJointEnabledAttr(False)
    if not meshes:raise RuntimeError('Robot meshes missing')
    heights=[.18,.38,.62,.90]
    paths=[]
    for i,height in enumerate(heights):
        ok,lidar=omni.kit.commands.execute('RangeSensorCreateLidar',
            path=f'/World/Robot/Lidar{i}',parent=None,min_range=.15,max_range=20,
            draw_points=False,draw_lines=False,horizontal_fov=360,vertical_fov=1,
            horizontal_resolution=1,vertical_resolution=1,rotation_rate=0,
            high_lod=False,yaw_offset=0)
        if not ok:raise RuntimeError('Lidar creation failed')
        lidar.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(0,0,height))
        paths.append(str(lidar.GetPath()))
    housing=UsdGeom.Cylinder.Define(stage,'/World/Robot/LidarHousing')
    housing.CreateRadiusAttr(.07);housing.CreateHeightAttr(.08)
    housing.AddTranslateOp().Set(Gf.Vec3d(0,0,1.38))
    housing.CreateDisplayColorAttr([Gf.Vec3f(.02,.8,.64)])
    interface=_range_sensor.acquire_lidar_sensor_interface()
    for _ in range(10):app.update()
    world=World(stage_units_in_meters=1,physics_dt=1/60,rendering_dt=1/30,
                physics_prim_path='/World/PhysicsScene')
    def set_pose(pose):
        trans.Set(Gf.Vec3d(float(pose[0]),float(pose[1]),0))
        orient.Set(Gf.Quatf(math.cos(pose[2]/2),0,0,math.sin(pose[2]/2)))
    def scan(pose):
        set_pose(pose)
        # Wait for new physics frames after the explicit root transform write.
        for _ in range(3):world.step(render=True)
        up=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(paths[0])).TransformDir(Gf.Vec3d(0,0,1))
        if abs(up[2]-1)>.001:raise AssertionError('Lidar scan plane is not horizontal in world Z-up coordinates')
        angles=np.array(interface.get_azimuth_data(paths[0]),copy=True).reshape(-1)
        if angles.size!=360 or np.max(abs(angles))>2*math.pi+.1:
            raise AssertionError('Unexpected PhysX scan dimensions/angular convention')
        raw=np.stack([np.array(interface.get_depth_data(path),copy=True).reshape(360,-1)[:,0] for path in paths])
        ranges=raw.astype(float)*20/65535
        if not np.isfinite(ranges).all() or not np.any((ranges>.15)&(ranges<19.99)):
            raise AssertionError('No valid measured returns')
        # Nearest return across four horizontal slices, with a common XY origin.
        # A below-minimum sentinel is invalid, not a free-space observation.
        valid=np.where(ranges>=.15,ranges,np.inf)
        fused=np.min(valid,axis=0)
        fused[np.all(ranges<.15,axis=0)]=np.nan
        return angles,raw,ranges,fused
    set_pose(poses[0]);world.reset()
    for _ in range(20):world.step(render=True)
    if a.probe:
        import omni.physx
        probe=[]
        for pose in [poses[0],(.9,-1.4,0),(3.1,1.5,0)]:
            angles,raw,ranges,fused=scan(pose)
            cache=UsdGeom.XformCache()
            row=dict(pose=pose,world_sensor_matrix=str(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(paths[0]))),
                     angles=angles[::30].tolist(),ranges=ranges[:,::30].tolist(),casts=[])
            for angle in [0,math.pi/2,math.pi,3*math.pi/2]:
                hit=omni.physx.get_physx_scene_query_interface().raycast_closest((pose[0],pose[1],.38),(math.cos(angle),math.sin(angle),0),20)
                row['casts'].append(str(hit))
            probe.append(row)
        (out/'probe.json').write_text(json.dumps(probe,indent=2),encoding='utf-8')
        set_camera_view(eye=np.array([4,-5,12]),target=np.array([0,0,.3]))
        for _ in range(35):app.update()
        capture_viewport_to_file(get_active_viewport(),file_path=str(out/'probe.png'))
        for _ in range(30):app.update()
        return
    mapper=OccupancyMapper(MAP_SPEC)
    raw_history=[];fused_history=[];times=[];snapshots=[];endpoint_errors=[]
    started=time.perf_counter()
    for i,pose in enumerate(poses):
        angles,raw,ranges,fused=scan(pose)
        mapper.integrate(pose,angles,fused,20)
        raw_history.append(raw);fused_history.append(fused);times.append(world.current_time)
        hit=np.isfinite(fused)&(fused>.15)&(fused<19.99)
        ex=pose[0]+fused[hit]*np.cos(pose[2]+angles[hit])
        ey=pose[1]+fused[hit]*np.sin(pose[2]+angles[hit])
        endpoint_errors.extend(max(0,clearance(float(x),float(y),geometry)) for x,y in zip(ex[::6],ey[::6]))
        if i in [0,len(poses)//4,len(poses)//2,3*len(poses)//4,len(poses)-1]:
            snapshots.append((i,mapper.log_odds.copy(),mapper.observed.copy()))
        if i%25==0: print(f'CODE_ROOM_MAPPING {i+1}/{len(poses)} observed={mapper.observed.sum()}',flush=True)
    mapper.save(out)
    np.savez_compressed(out/'scans.npz',raw_depth_uint16=np.array(raw_history),
        ranges=np.array(fused_history),angles=angles,poses=np.array(poses),times=times,
        heights=heights,max_range=20,depth_scale=20/65535)
    np.savez_compressed(out/'map_progress.npz',indices=[s[0] for s in snapshots],
        log_odds=np.array([s[1] for s in snapshots]),observed=np.array([s[2] for s in snapshots]))
    # Only this measured mapper is allowed in goal planning. The independent
    # scene geometry below audits results; it does not supply missing free cells.
    radius=.25
    cases=[('跨房间导航',(.9,-1.4),(3.1,1.5)),
           ('绕过中央岛台',(3.4,-1.25),(3.1,1.5)),
           ('卧室东侧通道',(.9,-1.4),(.92,1.6))]
    plans=[]
    for name,start,goal in cases:
        route=mapper.plan(start,goal,radius)
        gap=path_clearance(route,geometry)
        if gap<radius: raise AssertionError(f'Geometry audit failed {name}: {gap}')
        plans.append(dict(name=name,start=start,goal=goal,path=route,
            length_m=path_length(route),min_clearance_m=gap))
    negative=[]
    for name,point in [('床体内部',(-.5,.4)),('衣柜内部',(4.15,.8)),('房间外部',(6,0)),('休闲区未连通',(-3.65,0))]:
        try:mapper.plan(cases[0][1],point,radius)
        except ValueError as exc:negative.append(dict(name=name,point=point,reason=str(exc)))
        else:raise AssertionError(f'Invalid goal accepted: {name}')
    # Replay the planned route using explicit poses too. This is not locomotion.
    replay=[]
    for pose in sample_tour(plans[0]['path'],.08):
        set_pose(pose);world.step(render=True);replay.append(pose)
    final=plans[0]['goal'];set_pose((*final,replay[-1][2]))
    for _ in range(3):world.step(render=True)
    replay.append((*final,replay[-1][2]))
    (out/'scripted_replay.json').write_text(json.dumps(dict(poses=replay,
        mode='explicit USD root transforms, no velocity or torque control')),encoding='utf-8')

    def line(path,points,color,width=.035,z=.035):
        obj=UsdGeom.BasisCurves.Define(stage,path)
        obj.CreateTypeAttr('linear');obj.CreateCurveVertexCountsAttr([len(points)])
        obj.CreatePointsAttr([Gf.Vec3f(float(x),float(y),z) for x,y in points])
        obj.CreateWidthsAttr([width]);obj.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    line('/World/NavigationOverlay/Survey',tour,(.16,.47,1),.025,.032)
    line('/World/NavigationOverlay/Planned',plans[0]['path'],(.02,.85,.62),.055,.055)
    for name,(x,y),color in [('Start',cases[0][1],(.16,.47,1)),('Goal',cases[0][2],(1,.58,.10))]:
        obj=UsdGeom.Cylinder.Define(stage,f'/World/NavigationOverlay/{name}')
        obj.CreateRadiusAttr(.13);obj.CreateHeightAttr(.03)
        obj.AddTranslateOp().Set(Gf.Vec3d(x,y,.07));obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    set_pose((.9,-.9,1.57))
    stage.GetRootLayer().Save()

    def capture(filename,eye,target):
        set_camera_view(eye=np.array(eye),target=np.array(target))
        for _ in range(35):app.update()
        capture_viewport_to_file(get_active_viewport(),file_path=str(out/filename))
        for _ in range(30):app.update()
        if not (out/filename).is_file():raise RuntimeError('Viewport capture failed')
    import carb
    carb.settings.get_settings().set('/persistent/app/viewport/displayOptions',0)
    capture('isaac_full_room.png',[5,-7,12],[0,0,.4])
    # A dedicated top camera makes the original floor layout readable.
    cam=UsdGeom.Camera.Define(stage,'/World/PreviewTopCamera')
    # Long focal length perspective avoids RTX's black orthographic preview.
    cam.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    cam.CreateHorizontalApertureAttr(36);cam.CreateVerticalApertureAttr(24)
    cam.CreateFocalLengthAttr(50);cam.CreateClippingRangeAttr(Gf.Vec2f(.1,100))
    matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(0,0,15),Gf.Vec3d(0,0,0),Gf.Vec3d(0,1,0)).GetInverse()
    cam.AddTransformOp().Set(matrix)
    viewport=get_active_viewport();viewport.set_active_camera(str(cam.GetPath()))
    for _ in range(35):app.update()
    capture_viewport_to_file(viewport,file_path=str(out/'isaac_topdown.png'))
    for _ in range(30):app.update()
    stage.GetRootLayer().Save()
    # Hide only foreground wall visuals AFTER measurements and all audits.
    # Visibility does not change collision attributes. Override is session-only.
    stage.SetEditTarget(stage.GetSessionLayer())
    hidden=[]
    for q in stage.Traverse():
        name=str(q.GetPath()).lower()
        if q.IsA(UsdGeom.Mesh) and ('wall' in name or 'partition' in name):
            b=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeWorldBound(q).ComputeAlignedRange()
            lo,hi=b.GetMin(),b.GetMax()
            if hi[1]<-2.85 or lo[0]>4.4 or (lo[0]>1.4 and hi[0]<1.7) or (lo[0]>-2.7 and hi[0]<-2.4):
                UsdGeom.Imageable(q).MakeInvisible();hidden.append(str(q.GetPath()))
    viewport.set_active_camera('/OmniverseKit_Persp')
    capture('isaac_cutaway.png',[7.8,-10.5,13.0],[0,0,.35])
    capture('isaac_robot_closeup.png',[2.8,-3.5,2.3],[.9,-.85,.7])
    wall_time=time.perf_counter()-started
    metrics=dict(status='passed',simulator='Isaac Sim 4.5.0',source_scene=str(a.scene.resolve()),
        motion='scripted USD root position/orientation at preset samples; no policy/dynamics',
        mapping='PhysX measured ranges only, inverse sensor log-odds grid',
        pose_source='ideal simulator root XY/yaw; no estimated odometry or loop closure',
        sensor='4 horizontal PhysX 360-degree slices, nearest-return XY fusion',
        scan_count=len(poses),beams_per_slice=360,slices=len(heights),heights_m=heights,
        raw_ray_count=len(poses)*360*len(heights),resolution_m=MAP_SPEC.resolution,
        observed_area_m2=float(mapper.observed.sum()*MAP_SPEC.resolution**2),
        free_area_m2=float(mapper.free().sum()*MAP_SPEC.resolution**2),
        occupied_cells=int(mapper.occupied().sum()),
        nominal_floor_area_m2=54,known_fraction_of_floor=float(mapper.observed[16:256,16:376].mean()),
        collection_tour_length_m=path_length(tour),collection_step_m=a.tour_step,
        collection_clearance_to_AABBs_m=path_clearance(tour,geometry),
        collection_disk_radius_m=.22,planning_disk_radius_m=radius,
        footprint_scope='illustrative 2D disk; full robot mesh, arms, dynamics and low obstacles between slices not certified',
        robot_mesh_count=meshes,robot_visual_bounds_size_m=(vmax-vmin).tolist(),
        endpoint_AABB_distance_p95_m=float(np.percentile(endpoint_errors,95)),
        accepted_survey_positions=accepted,rejected_survey_positions=rejected,
        planning_cases=plans,negative_goal_tests=negative,
        scripted_replay_endpoint_error_m=math.dist(replay[-1][:2],final),
        preview_hidden_wall_meshes=hidden,wall_time_s=wall_time,
        data_flow='scene geometry -> tour design / final audit only; scans + ideal pose -> occupancy -> A*',
        scan_saved_before_visual_cutaway=True)
    if metrics['endpoint_AABB_distance_p95_m']>.12:raise AssertionError('Scan endpoints inconsistent with scene geometry')
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False),encoding='utf-8')
    from scan_nav.code_room_report import write_report
    write_report(out)
    print('CODE_ROOM_VALIDATION '+json.dumps({k:metrics[k] for k in ['status','scan_count','raw_ray_count','free_area_m2','wall_time_s']}),flush=True)
    world.stop()

try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    args_out=a.output.resolve()
    args_out.mkdir(parents=True,exist_ok=True)
    (args_out/'metrics.json').write_text(json.dumps({'status':'failed','traceback':traceback.format_exc()}),encoding='utf-8')
    sys.stderr.flush();sys.stdout.flush()
    raise
finally:app.close()
