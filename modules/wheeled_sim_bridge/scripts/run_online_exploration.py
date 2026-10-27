"""Isaac closed loop: sensors -> online frontier/Nav2 -> vx,wz -> actuator.

The simulator owns USD/PhysX. The decision modules receive only scans, measured
maps and odometry. No collection_tour, collider JSON or waypoint file is read.
The actuator is explicitly kinematic; this does not validate balancing physics.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

MODULES=Path(__file__).resolve().parents[2]
WORKSPACE=MODULES.parent
sys.path.insert(0,str(MODULES/'wheeled_navigation'))
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--mode',choices=['native','ros2'],default='native')
p.add_argument('--scene',choices=['code-as-room','validation'],default='code-as-room')
p.add_argument('--scene-usd',type=Path,help='Independent USD scene variant, never passed to planners')
p.add_argument('--seconds',type=float,default=200.)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--odometry-drift',type=float,default=.03,help='fractional wheel distance error, ROS stage only')
p.add_argument('--range-noise',type=float,default=0.,help='Gaussian range standard deviation in metres')
p.add_argument('--seed',type=int,default=7)
p.add_argument('--domain-id',type=int,default=42)
p.add_argument('--lidar-resolution',type=float,default=.5,help='Native azimuth resolution in degrees')
p.add_argument('--realtime',action='store_true')
p.add_argument('--dynamic-test',action='store_true')
a=p.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
if a.scene_usd and (a.scene!='code-as-room' or not a.scene_usd.is_file()):
    p.error('--scene-usd must be an existing file in code-as-room mode')
(a.output/'status.json').write_text(json.dumps({'status':'starting','mode':a.mode}),encoding='utf-8')
if a.mode=='ros2':
    isaac_root=Path(os.environ.get('ISAAC_PATH','D:/isaacsim'))
    ros_lib=isaac_root/'exts/isaacsim.ros2.bridge/humble/lib'
    os.environ.update(ROS_DOMAIN_ID=str(a.domain_id),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    os.environ['PATH']=str(ros_lib)+os.pathsep+os.environ.get('PATH','')
    dll_handle=os.add_dll_directory(str(ros_lib))
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':720,'renderer':'RayTracedLighting',
    'anti_aliasing':0,'fast_shutdown':True,
    'extra_args':['--/plugins/carb.tasking.plugin/threadCount=4','--/rtx/hydra/enableMultithreading=false']})


def main():
    import numpy as np
    import omni.usd
    import omni.timeline
    import omni.kit.commands
    from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdLux
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.utils.extensions import enable_extension
    from scan_nav.core import GridSpec,OccupancyMapper,path_length
    from scan_nav.exploration import ExplorationMap,FrontierExplorer,velocity_to_path,integrate_velocity,measured_stop,map_stop,ROBOT_RADIUS
    from scan_nav.footprint import measured_polygon_stop,ROBOT_POLYGON,PADDING

    wrapper=a.output/'online_scene.usda'
    layer=Usd.Stage.CreateInMemory()
    scene_source=None
    if a.scene=='code-as-room':
        scene_source=(a.scene_usd or WORKSPACE/'research/scene_audit/code_as_room/scene.usda').resolve()
        layer.GetRootLayer().subLayerPaths=[scene_source.as_posix()]
    UsdGeom.SetStageUpAxis(layer,'Z');UsdGeom.SetStageMetersPerUnit(layer,1.)
    layer.GetRootLayer().Export(str(wrapper));open_stage(str(wrapper))
    stage=omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    for _ in range(8):app.update()
    UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    if a.scene=='validation':
        UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
        UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1000.)
        def box(name,centre,size,color):
            obj=UsdGeom.Cube.Define(stage,'/World/'+name);obj.CreateSizeAttr(1.)
            obj.AddTranslateOp().Set(Gf.Vec3d(*centre));obj.AddScaleOp().Set(Gf.Vec3f(*size))
            obj.CreateDisplayColorAttr([Gf.Vec3f(*color)]);UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        box('Floor',(0,0,-.1),(10,8,.2),(.55,.55,.6))
        for name,c,s in [('West',(-5,0,1),( .15,8,2)),('East',(5,0,1),(.15,8,2)),
                         ('North',(0,4,1),(10,.15,2)),('South',(0,-4,1),(10,.15,2)),
                         ('PartitionLeft',(0,2.3,1),(.15,3.4,2)),
                         ('PartitionRight',(0,-2.3,1),(.15,3.4,2)),
                         ('Cabinet',(-2.2,.6,.5),(1.4,1.2,1.)),
                         ('Island',(2.4,.4,.5),(1.5,1.5,1.))]:
            box(name,c,s,(.7,.62,.52))
    # Snapshot actual composed geometry ONLY for the post-run evaluator. The
    # ROS process receives scans/odometry and never receives this geometry file.
    bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
    floor_box=bbox.ComputeWorldBound(stage.GetPrimAtPath('/World/Floor')).ComputeAlignedRange()
    floor_bounds=[float(floor_box.GetMin()[0]),float(floor_box.GetMin()[1]),
                  float(floor_box.GetMax()[0]),float(floor_box.GetMax()[1])]
    collider_records=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Boundable):
            box=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            collider_records.append({'path':str(prim.GetPath()),'min':list(box.GetMin()),'max':list(box.GetMax())})
    geometry_file=a.output/'scene_colliders.json'
    geometry_file.write_text(json.dumps(collider_records,indent=2),encoding='utf-8')
    enable_extension('isaacsim.sensors.physx')
    if a.mode=='ros2': enable_extension('isaacsim.ros2.bridge')
    from isaacsim.sensors.physx import _range_sensor
    robot=UsdGeom.Xform.Define(stage,'/World/Robot')
    trans=robot.AddTranslateOp();orient=robot.AddOrientOp()
    model=UsdGeom.Xform.Define(stage,'/World/Robot/WheeledArm')
    model.GetPrim().GetReferences().AddReference((WORKSPACE/'research/scene_audit/code_as_room/wheeled_arm_v2_bent_visual.usdc').as_posix())
    paths=[];heights=[];mount=(.16,0.,.84)
    for i,height in enumerate([mount[2]]):
        path=f'/World/Robot/Lidar{i}'
        ok,lidar=omni.kit.commands.execute('RangeSensorCreateLidar',path=path,parent=None,
            min_range=.15,max_range=8.,draw_points=False,draw_lines=False,horizontal_fov=360,
            vertical_fov=140,horizontal_resolution=a.lidar_resolution,vertical_resolution=1,rotation_rate=0,high_lod=True,yaw_offset=0)
        if not ok:raise RuntimeError('Lidar initialization failed')
        lidar.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(mount[0],0,height));paths.append(path)
    housing=UsdGeom.Cylinder.Define(stage,'/World/Robot/LidarHousing')
    housing.CreateRadiusAttr(.045);housing.CreateHeightAttr(.04)
    housing.AddTranslateOp().Set(Gf.Vec3d(*mount));housing.CreateDisplayColorAttr([Gf.Vec3f(.03,.8,.6)])
    interface=_range_sensor.acquire_lidar_sensor_interface()
    pose=(.9,-1.4,0.) if a.scene=='code-as-room' else (-3.6,-2.6,0.)
    start=pose;odom=pose;dt=.1
    def set_pose(pose):
        trans.Set(Gf.Vec3d(pose[0],pose[1],0.))
        orient.Set(Gf.Quatf(math.cos(pose[2]/2),0,0,math.sin(pose[2]/2)))
    set_pose(pose)
    world=World(stage_units_in_meters=1.,physics_dt=1/30,rendering_dt=1/30,physics_prim_path='/World/PhysicsScene')
    world.reset()
    for _ in range(20):world.step(render=True)
    mapper=ExplorationMap(GridSpec(.05,-12.,-12.,480,480))
    rng=np.random.default_rng(a.seed)
    # The simple baseline follower leaves extra room for its tracking error.
    # Nav2 has its own costmap inflation and collision-aware controller.
    explorer=FrontierExplorer(radius=ROBOT_RADIUS+.06)
    bridge=None;ros_status={'phase':'waiting'}
    if a.mode=='ros2':
        from ros2_sensor_bridge import Ros2SensorBridge
        from std_msgs.msg import String
        bridge=Ros2SensorBridge(external_slam=True,base_height=.5561404345,lidar_mount=mount,max_range=8.)
        def on_status(msg):
            ros_status.update(json.loads(msg.data))
        status_sub=bridge.node.create_subscription(String,'/exploration/status',on_status,10)
    rows=[];scans=[];raws=[];events=[];snapshots=[];plans=[];active=None;path=[];goal_started=0.;replans=0
    phase='exploring';empty_rounds=0;stops=0;blocked_since=None;command=(0.,0.);injected=False
    last_progress=0.;last_progress_pose=pose;begin=time.monotonic();last_choice=-10.
    best_area=0.;last_gain=5.
    def event(kind,t,**kwargs):
        row={'event':kind,'seconds':round(t,3),**kwargs};events.append(row)
        print('ONLINE_EVENT '+json.dumps(row),flush=True)
    event('start',0.,planner_inputs=['scan','measured_map','odometry'],preplanned_route=False)
    try:
        for i in range(round(a.seconds/dt)):
            tick=time.monotonic();seconds=5.+i*dt
            # The only motion update consumes vx/wz, never a target or route.
            pose=integrate_velocity(pose,command,dt);set_pose(pose)
            if a.mode=='ros2':
                odom_cmd=(command[0]*(1.+a.odometry_drift),command[1]+(.0015 if abs(command[0])>.01 else 0.))
                odom=integrate_velocity(odom,odom_cmd,dt)
            else:odom=pose
            for _ in range(3):world.step(render=True)
            angles=np.array(interface.get_azimuth_data(paths[0]),copy=True).reshape(-1)
            raw=np.array(interface.get_depth_data(paths[0]),copy=True).reshape(len(angles),-1).T
            ranges=raw.astype(float)*8./65535.
            cloud=np.array(interface.get_point_cloud_data(paths[0]),copy=True).reshape(len(angles),-1,3).transpose(1,0,2)
            hit_heights=mount[2]+cloud[...,2]
            valid=(ranges>=.15)&(ranges<7.99)&(hit_heights>=.06)&(hit_heights<=1.12)
            # A virtual planar obstacle scan from TRUE 3D returns. The floor is
            # excluded; low furniture and thin bedding between old scan slices
            # remain visible. This is not a commercial 2D-lidar sensor model.
            xy_range=np.linalg.norm(cloud[...,:2],axis=2)
            fused=np.min(np.where(valid,xy_range,np.inf),axis=0)
            fused[~np.isfinite(fused)]=8.
            if a.range_noise:
                hit=(fused>=.15)&(fused<7.99)
                fused[hit]=np.clip(fused[hit]+rng.normal(0.,a.range_noise,int(hit.sum())),.15,7.98)
            if angles.size<360 or not np.any(fused<7.99):raise RuntimeError('No measured scan returns')
            sensor_pose=(pose[0]+.16*math.cos(pose[2]),pose[1]+.16*math.sin(pose[2]),pose[2])
            # Truth-aligned map is a baseline/evaluation artifact only in ROS mode;
            # it is NEVER published to SLAM/Nav2 and cannot select their goals.
            mapper.integrate(sensor_pose,angles,fused,8.)
            area=float(mapper.observed.sum())*mapper.spec.resolution**2
            if area>best_area+.15:best_area=area;last_gain=seconds
            if bridge:
                bridge.publish(seconds,odom,angles,fused)
                command=bridge.command();phase=ros_status.get('phase','waiting')
                active=ros_status.get('goal');path=ros_status.get('path',[])
            else:
                if phase=='exploring' and seconds-last_gain>50 and len(explorer.attempts)>=3:
                    event('exploration_stagnation',seconds,seconds_without_new_area=seconds-last_gain)
                    phase='returning';active=start[:2];goal_started=seconds
                    try:path=mapper.plan(pose[:2],active,explorer.radius)
                    except ValueError:phase='blocked_by_measured_map';active=None
                if active and math.dist(pose[:2],active)<.18:
                    event('goal_reached',seconds,goal=active,phase=phase)
                    explorer.record(active,True,seconds);active=None;path=[]
                    if phase=='returning':phase='complete'
                if active and (seconds-goal_started>45 or (blocked_since is not None and seconds-blocked_since>5)):
                    event('goal_failed',seconds,goal=active,reason='timeout_or_sensor_stop')
                    explorer.record(active,False,seconds);active=None;path=[];blocked_since=None
                    if phase=='returning':phase='return_failed'
                if phase=='exploring' and active is None and seconds-last_choice>=1.:
                    last_choice=seconds;chosen=explorer.choose(mapper,pose,seconds)
                    if chosen:
                        active=chosen.position;path=chosen.path;goal_started=seconds;empty_rounds=0
                        event('frontier_goal',seconds,goal=active,information_cells=chosen.information_cells,
                              known_cells=int(mapper.observed.sum()),path=path)
                    else:
                        empty_rounds+=1
                        if empty_rounds>=3:
                            event('no_reachable_frontier',seconds,diagnostics=explorer.last_diagnostics)
                            phase='returning';active=start[:2];goal_started=seconds
                            try:path=mapper.plan(pose[:2],active,explorer.radius)
                            except ValueError:phase='blocked_by_measured_map';active=None
                            event('return_to_start_goal',seconds,goal=active)
                if active and i%10==0:
                    try:
                        path=mapper.plan(pose[:2],active,explorer.radius);replans+=1
                        plans.append({'seconds':seconds,'goal':active,'path':path})
                    except ValueError:
                        event('path_invalidated',seconds,goal=active)
                        explorer.record(active,False,seconds);active=None;path=[]
                command,reached=velocity_to_path(pose,path,speed=.25,lookahead=.08,heading_gate=.2)
            if bridge:command,stop=measured_polygon_stop(command,angles,fused)
            else:command,stop=measured_stop(command,pose,sensor_pose,angles,fused)
            if not bridge:
                command,map_blocked=map_stop(mapper,pose,command)
                stop=stop or map_blocked
            stops+=int(stop)
            blocked_since=seconds if stop and blocked_since is None else (blocked_since if stop else None)
            if a.dynamic_test and not injected and i>200 and command[0]>.15:
                # Test fixture only: an unforeseen box appears ahead of the robot.
                # Neither explorer nor Nav2 receives its location or identity.
                obj=UsdGeom.Cube.Define(stage,'/World/UnexpectedObstacle');obj.CreateSizeAttr(1.)
                location=(pose[0]+1.1*math.cos(pose[2]),pose[1]+1.1*math.sin(pose[2]),.5)
                obj.AddTranslateOp().Set(Gf.Vec3d(*location));obj.AddScaleOp().Set(Gf.Vec3f(.45,.45,1.))
                obj.CreateDisplayColorAttr([Gf.Vec3f(.8,.15,.1)]);UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
                event('test_fixture_obstacle_inserted',seconds,world_position=location);injected=True
            rows.append({'seconds':seconds,'truth':pose,'odom':odom,'command':command,'phase':phase,
                         'known_cells':int(mapper.observed.sum()),'sensor_stop':stop})
            scans.append(fused.astype(np.float32));raws.append(raw)
            if i%150==0:
                snapshots.append((seconds,mapper.log_odds.copy(),mapper.observed.copy()))
                (a.output/'progress.json').write_text(json.dumps(rows[-1],indent=2),encoding='utf-8')
                print(f'ONLINE_PROGRESS {i} {seconds:.1f}s {phase} pose={pose} known={mapper.observed.sum()}',flush=True)
            if phase in ('complete','blocked_by_measured_map','return_failed','budget_reached') and i>30:break
            if bridge or a.realtime:
                time.sleep(max(0.,dt-(time.monotonic()-tick)))
        command=(0.,0.)
        mapper.save(a.output/'measured_baseline')
        np.savez_compressed(a.output/'sensor_history.npz',angles=angles,ranges=np.array(scans),
                            raw_depth=np.array(raws),heights=heights,max_range=8.,
                            truth=np.array([r['truth'] for r in rows]),odom=np.array([r['odom'] for r in rows]),
                            times=np.array([r['seconds'] for r in rows]))
        np.savez_compressed(a.output/'map_progress.npz',times=[s[0] for s in snapshots],
                            log_odds=np.array([s[1] for s in snapshots]),observed=np.array([s[2] for s in snapshots]))
        (a.output/'motion.json').write_text(json.dumps(rows),encoding='utf-8')
        (a.output/'events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
        (a.output/'plans.json').write_text(json.dumps(plans),encoding='utf-8')
        metrics={'status':'completed' if phase=='complete' else 'incomplete','execution_finished':True,'mode':a.mode,'scene':a.scene,'phase':phase,
            'sim_seconds':len(rows)*dt,'wall_seconds':time.monotonic()-begin,
            'distance_m':path_length([r['truth'][:2] for r in rows]),'robot_radius_m':ROBOT_RADIUS,
            'sensor':f'PhysX 3D lidar, {len(angles)} azimuth x {raw.shape[0]} elevation channels, height-filtered planar projection',
            'preplanned_route':False,'ground_truth_map_used_for_decisions':False,
            'kinematic_actuator':True,'policy_checkpoint_used':False,'sensor_stops':stops,
            'replans':replans,'known_area_m2':float(mapper.observed.sum()*.05**2),
            'goals_selected':ros_status.get('goals_sent',0) if bridge else sum(e['event']=='frontier_goal' for e in events),
            'goals_reached':ros_status.get('goals_reached',0) if bridge else sum(e['event']=='goal_reached' for e in events),
            'odometry_drift_fraction':a.odometry_drift if bridge else 0.,'ros_status':ros_status if bridge else None}
        metrics.update(range_noise_std_m=a.range_noise,seed=a.seed,
                       odometry_yaw_bias_rad_s=.0015 if bridge else 0.,
                       navigation_footprint={'polygon':ROBOT_POLYGON,'padding':PADDING} if bridge else {'radius':ROBOT_RADIUS},
                       ground_truth_pose_used_for_mapping=not bool(bridge),
                       scene_usd=str(scene_source) if scene_source else None,
                       scene_geometry_file=str(geometry_file),floor_bounds=floor_bounds,
                       floor_area_m2=(floor_bounds[2]-floor_bounds[0])*(floor_bounds[3]-floor_bounds[1]))
        (a.output/'status.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
        stage.GetRootLayer().Export(str(wrapper))
        print('ONLINE_RESULT '+json.dumps(metrics),flush=True)
    finally:
        if bridge:bridge.close()

try:
    main()
except BaseException as exc:
    traceback.print_exc()
    (a.output/'status.json').write_text(json.dumps({'status':'failed','error':str(exc),'traceback':traceback.format_exc()}),encoding='utf-8')
finally:
    app.close()
