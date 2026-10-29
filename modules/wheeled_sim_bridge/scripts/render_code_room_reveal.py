"""Render a native Isaac scene and reveal it using new measured lidar scans.

RGB, camera depth/labels and robot motion are rendered in Isaac Sim. Brightness
is an explanatory composite: measured XY visibility plus hit mesh parts, not a
physical spotlight or a reconstructed/fully observed 3D surface model.
"""
from __future__ import annotations
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

MODULES=Path(__file__).resolve().parents[2]
WORKSPACE=MODULES.parent
sys.path.insert(0,str(MODULES/'wheeled_navigation'))
sys.path.insert(0,str(WORKSPACE/'research/video_deps'))
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--preview',action='store_true',help='Short 8 fps sample of the whole tour')
p.add_argument('--fps',type=int,default=24)
p.add_argument('--tour-seconds',type=float,default=34.)
p.add_argument('--output',type=Path,default=MODULES/'wheeled_navigation/artifacts/code_as_room_video')
a=p.parse_args()
if a.preview:
    a.fps=8;a.tour_seconds=12
    a.output=a.output.parent/'code_as_room_video_preview'
if a.fps<=0 or a.tour_seconds<=0:p.error('fps and tour-seconds must be positive')
import imageio_ffmpeg
FFMPEG=imageio_ffmpeg.get_ffmpeg_exe()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'renderer':'RayTracedLighting',
    'anti_aliasing':0,'fast_shutdown':True,
    'extra_args':['--/plugins/carb.tasking.plugin/threadCount=4',
                  '--/rtx/hydra/enableMultithreading=false']})

def main():
    import numpy as np
    import omni.usd
    import omni.timeline
    import carb
    import cv2
    from PIL import Image,ImageDraw
    from pxr import Gf,Usd,UsdGeom,UsdShade,Sdf
    import omni.kit.commands
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.semantics import add_update_semantics
    from isaacsim.sensors.camera import Camera
    from scan_nav.code_room import MAP_SPEC,collection_tour,load_geometry
    from scan_nav.reveal_video import RevealComposer,route_motion,font

    out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    (out/'render_status.json').write_text(json.dumps({'status':'running'}),encoding='utf-8')
    source=MODULES/'wheeled_navigation/artifacts/code_as_room_demo/navigation_scene.usda'
    geometry=WORKSPACE/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json'
    route,accepted,rejected=collection_tour(load_geometry(geometry),.22)
    intro,outro=2.,4.
    motion=list(route_motion(route,a.fps,a.tour_seconds,intro,outro))
    layer=Usd.Stage.CreateInMemory();layer.GetRootLayer().subLayerPaths=[source.as_posix()]
    wrapper=out/'reveal_scene.usda';layer.GetRootLayer().Export(str(wrapper))
    open_stage(str(wrapper));stage=omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1)
    omni.timeline.get_timeline_interface().stop()
    for _ in range(10):app.update()
    carb.settings.get_settings().set('/persistent/app/viewport/displayOptions',0)
    stage.GetPrimAtPath('/World/NavigationOverlay').SetActive(False)
    robot=stage.GetPrimAtPath('/World/Robot')
    stage.GetPrimAtPath('/World/Robot/Model').SetActive(False)
    robot_asset=WORKSPACE/'research/scene_audit/code_as_room/wheeled_arm_v2_bent_visual.usdc'
    if not robot_asset.is_file():raise FileNotFoundError('Run research/prepare_wheeled_arm_visual.py first')
    replacement=UsdGeom.Xform.Define(stage,'/World/Robot/WheeledArm')
    replacement.GetPrim().GetReferences().AddReference(robot_asset.as_posix())
    trans=robot.GetAttribute('xformOp:translate');orient=robot.GetAttribute('xformOp:orient')
    def pose_root(pose):
        trans.Set(Gf.Vec3d(float(pose[0]),float(pose[1]),0))
        orient.Set(Gf.Quatf(math.cos(pose[2]/2),0,0,math.sin(pose[2]/2)))
    pose_root(motion[0][1])
    add_update_semantics(robot,'robot')
    housing=stage.GetPrimAtPath('/World/Robot/LidarHousing')
    mount=np.array([.16,0,.84])
    housing.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*map(float,mount)))
    UsdGeom.Cylinder(housing).GetRadiusAttr().Set(.045)
    UsdGeom.Cylinder(housing).GetHeightAttr().Set(.04)

    # Every actual room mesh has an independent render label and hit-path ID.
    # A furniture mesh part fades in only after a measured ray hits that part.
    part_paths={};part_labels={};surface_paths=[];mesh_bounds={}
    bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
    for q in stage.Traverse():
        if not q.IsA(UsdGeom.Mesh) or str(q.GetPath()).startswith('/World/Robot'):continue
        name=str(q.GetPath());bounds=bbox.ComputeWorldBound(q).ComputeAlignedRange()
        lo,hi=bounds.GetMin(),bounds.GetMax()
        mesh_bounds[name]=(np.array(lo),np.array(hi))
        is_surface=('wall' in name.lower() or 'partition' in name.lower() or hi[2]<.06)
        if is_surface:
            add_update_semantics(q,'surface');surface_paths.append(name)
        else:
            idx=len(part_paths)+1;part_paths[name]=idx
            label=f'part{idx:04d}';part_labels[label]=idx;add_update_semantics(q,label)
    enable_extension('isaacsim.sensors.physx')
    from isaacsim.sensors.physx import _range_sensor
    for i in range(4):stage.GetPrimAtPath(f'/World/Robot/Lidar{i}').SetActive(False)
    # A vertical field of view lets actual rays hit bedding/table surfaces.
    # The earlier 2D mapping experiment and its files are left unchanged.
    sensor_path='/World/Robot/RevealLidar'
    ok,lidar=omni.kit.commands.execute('RangeSensorCreateLidar',path=sensor_path,parent=None,
        min_range=.15,max_range=20,draw_points=False,draw_lines=False,horizontal_fov=360,
        vertical_fov=80,horizontal_resolution=1,vertical_resolution=5,rotation_rate=0,
        high_lod=True,yaw_offset=0,enable_semantics=True)
    if not ok:raise RuntimeError('3D display lidar creation failed')
    lidar.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(*map(float,mount)))
    interface=_range_sensor.acquire_lidar_sensor_interface()

    def material(path,color,opacity=1):
        mat=UsdShade.Material.Define(stage,path)
        shader=UsdShade.Shader.Define(stage,path+'/Shader');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(opacity)
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        return mat
    laser_mat=material('/World/RevealMaterials/Laser',(.08,.85,.65))
    UsdGeom.Xform.Define(stage,'/World/RevealOverlay')
    route_mat=material('/World/RevealMaterials/Route',(.10,.22,.38))
    trail_mat=material('/World/RevealMaterials/Trail',(.03,.5,.37))
    fan_mat=material('/World/RevealMaterials/Fan',(.06,.50,.39),.11)
    def curves(path,width,mat):
        c=UsdGeom.BasisCurves.Define(stage,path);c.CreateTypeAttr('linear')
        c.CreateWrapAttr('nonperiodic')
        c.CreateCurveVertexCountsAttr([2]);c.CreatePointsAttr([Gf.Vec3f(.9,-1.4,.05),Gf.Vec3f(.9002,-1.4,.05)])
        c.CreateWidthsAttr([width]);c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
        add_update_semantics(c.GetPrim(),'scan')
        return c
    laser=curves('/World/RevealOverlay/Laser',.008,laser_mat)
    trail=curves('/World/RevealOverlay/Trail',.025,trail_mat)
    itinerary=curves('/World/RevealOverlay/Itinerary',.010,route_mat)
    itinerary.CreateCurveVertexCountsAttr([len(route)])
    itinerary.CreatePointsAttr([Gf.Vec3f(x,y,.035) for x,y in route])
    add_update_semantics(itinerary.GetPrim(),'guide')
    fan=UsdGeom.Mesh.Define(stage,'/World/RevealOverlay/Fan')
    fan.CreateDoubleSidedAttr(True);fan.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(fan.GetPrim()).Bind(fan_mat)
    add_update_semantics(fan.GetPrim(),'fan')
    endpoints=UsdGeom.Points.Define(stage,'/World/RevealOverlay/Returns')
    endpoints.CreateWidthsAttr([.032]);endpoints.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    UsdShade.MaterialBindingAPI.Apply(endpoints.GetPrim()).Bind(laser_mat)
    add_update_semantics(endpoints.GetPrim(),'scan')
    UsdGeom.Imageable(stage.GetPrimAtPath('/World/RevealOverlay')).MakeInvisible()
    UsdGeom.Imageable(robot).MakeInvisible()

    world=World(stage_units_in_meters=1,physics_dt=1/120,rendering_dt=1/60,
                physics_prim_path='/World/PhysicsScene')
    world.reset()
    for _ in range(12):world.step(render=True)
    # A render-only cutaway. Static colliders remain in PhysX for every scan.
    hidden=[]
    for q in list(stage.Traverse()):
        name=str(q.GetPath()).lower()
        if q.IsA(UsdGeom.Mesh) and ('wall' in name or 'partition' in name):
            b=bbox.ComputeWorldBound(q).ComputeAlignedRange();lo,hi=b.GetMin(),b.GetMax()
            # Keep BOTH interior partitions, including their doorway segments.
            # Only the near exterior wall is cut away for a readable camera.
            if 'partition' not in name and hi[1]<-2.85:
                UsdGeom.Imageable(q).MakeInvisible();hidden.append(str(q.GetPath()))
    width,height,render_width=1920,1080,1536
    camera_prim=UsdGeom.Camera.Define(stage,'/World/RevealCamera')
    camera_prim.CreateProjectionAttr('perspective')
    camera_prim.CreateFocalLengthAttr(38)
    camera_prim.CreateHorizontalApertureAttr(36)
    camera_prim.CreateVerticalApertureAttr(36*height/render_width)
    camera_prim.CreateClippingRangeAttr(Gf.Vec2f(.1,100))
    camera=Camera('/World/RevealCamera',resolution=(render_width,height))
    camera.initialize();camera.add_distance_to_image_plane_to_frame();camera.add_semantic_segmentation_to_frame()
    xf=UsdGeom.Xformable(camera_prim.GetPrim());xf.ClearXformOpOrder()
    overview_matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(3,-4.5,16),Gf.Vec3d(0,0,.30),Gf.Vec3d(0,0,1)).GetInverse()
    camera_op=xf.AddTransformOp();camera_op.Set(overview_matrix)
    for _ in range(30):world.step(render=True)
    def frame_data():
        frame=camera.get_current_frame()
        rgb=np.asarray(camera.get_rgb()).copy()
        seg=frame.get('semantic_segmentation')
        if rgb.shape!=(height,render_width,3) or not isinstance(seg,dict):
            raise RuntimeError(f'Camera annotators not ready: rgb={rgb.shape}, seg={type(seg)}')
        return rgb,np.array(seg['data'],copy=True).reshape(height,render_width),seg['info']['idToLabels']
    base_rgb,base_seg,labels=frame_data()
    depth=np.array(camera.get_depth(),copy=True)
    if depth.shape!=(height,render_width):raise AssertionError(f'Bad depth: {depth.shape}')
    yy,xx=np.indices(depth.shape)
    safe_depth=np.where(np.isfinite(depth)&(depth>0)&(depth<100),depth,0)
    xyz=camera.get_world_points_from_image_coords(np.c_[xx.ravel(),yy.ravel()],safe_depth.ravel()).reshape(height,render_width,3)
    scene_mask=(safe_depth>0)&(xyz[...,2]>-.15)&(xyz[...,0]>-4.8)&(xyz[...,0]<4.8)&(xyz[...,1]>-3.3)&(xyz[...,1]<3.3)
    part_ids=np.zeros(depth.shape,dtype=np.int32)
    for key,value in labels.items():
        label=value.get('class','')
        if label in part_labels:part_ids[base_seg==int(key)]=part_labels[label]
    if not scene_mask.any() or not part_ids.any():raise AssertionError('Camera depth/mesh labels missing')
    Image.fromarray(base_rgb).save(out/'native_lit_background.png')
    np.savez_compressed(out/'camera_surface_projection.npz',world_xyz=xyz.astype(np.float32),
        part_ids=part_ids,scene_mask=scene_mask)
    composer=RevealComposer(width,height,render_width,MAP_SPEC,xyz[...,:2],scene_mask,part_ids,len(part_paths))
    composer.scan_layers=np.asarray(interface.get_depth_data(sensor_path)).reshape(360,-1).shape[1]
    display_seen=np.zeros((MAP_SPEC.height,MAP_SPEC.width),dtype=np.uint8)
    UsdGeom.Imageable(robot).MakeVisible()
    # Inspect the corrected URDF pose at ground level before the overview.
    start=motion[0][1]
    camera_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(start[0]+1.15,start[1]-2.1,1.25),
        Gf.Vec3d(start[0],start[1],.55),Gf.Vec3d(0,0,1)).GetInverse())
    for _ in range(25):world.step(render=True)
    Image.fromarray(np.array(camera.get_rgb(),copy=True)).save(out/'robot_pose_closeup.png')
    camera_op.Set(overview_matrix)
    for _ in range(20):world.step(render=True)
    UsdGeom.Imageable(stage.GetPrimAtPath('/World/RevealOverlay')).MakeVisible()
    # A finite laser strip is only drawn after the measured scan is available.
    UsdGeom.Imageable(laser).MakeInvisible();UsdGeom.Imageable(fan).MakeInvisible();UsdGeom.Imageable(endpoints).MakeInvisible()
    video=out/'code_as_room_lidar_reveal.mp4'
    cmd=[FFMPEG,'-y','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
         '-s',f'{width}x{height}','-r',str(a.fps),'-i','pipe:0','-an',
         '-c:v','libx264','-preset','fast','-crf','19','-threads','4','-pix_fmt','yuv420p','-movflags','+faststart',str(video)]
    encoder=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    history=[];raw_history=[];recorded_angles=None;recorded_zenith=None;scan_frames=[];hit_history=[];trail_points=[];stills=[]
    point_history=[];sensor_poses=[];endpoint_audit=[];layer_count=None
    begin=time.perf_counter();areas=[];meshcounts=[]
    fractions={0,round(len(motion)*.25),round(len(motion)*.5),round(len(motion)*.75),len(motion)-1}
    try:
        for i,(t,pose,command,progress) in enumerate(motion):
            pose_root(pose)
            # Actual sensor update after the new scripted root transform.
            for _ in range(3):world.step(render=True)
            phase=0 if t<intro else (1 if t<intro+a.tour_seconds else 2)
            hit_parts=set()
            if phase>0:
                angles=np.array(interface.get_azimuth_data(sensor_path),copy=True).reshape(-1)
                zenith=np.array(interface.get_zenith_data(sensor_path),copy=True).reshape(-1)
                raw=np.array(interface.get_depth_data(sensor_path),copy=True).reshape(360,-1).T
                distance=raw.astype(float)*20/65535
                local=np.array(interface.get_point_cloud_data(sensor_path),copy=True).reshape(360,-1,3).transpose(1,0,2)
                layer_count=raw.shape[0];composer.scan_layers=layer_count
                ranges=np.linalg.norm(local[...,:2],axis=2)
                if angles.size!=360 or layer_count<8 or not np.any(distance<19.99):raise AssertionError(f'Bad 3D lidar: angles={angles.shape}, raw={raw.shape}, min={distance.min()}, max={distance.max()}')
                transform=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(sensor_path))
                up=transform.TransformDir(Gf.Vec3d(0,0,1))
                if abs(up[2]-1)>.001:raise AssertionError('Lidar plane is not horizontal')
                sensor_origin=np.array(transform.ExtractTranslation())
                rotation=np.array(transform)[:3,:3].T
                world_hits=local@rotation.T+sensor_origin
                lidar_pose=(sensor_origin[0],sensor_origin[1],pose[2])
                hit_paths=np.array(interface.get_prim_data(sensor_path),dtype=object).reshape(360,-1).T
                horizontal=int(np.argmin(np.median(abs(local[:,:,2])/np.maximum(distance,1e-6),axis=1)))
                lower_layers=np.flatnonzero(np.median(local[:,:,2],axis=1)<=.02)
                if not lower_layers.size:raise AssertionError('Lidar has no downward/horizontal rays')
                # Native local points must land on the same USD collider AABBs.
                if len(raw_history)<3:
                    for k in range(layer_count):
                        for beam in range(0,360,20):
                            bounds=mesh_bounds.get(str(hit_paths[k,beam]))
                            if bounds is None or not .15<distance[k,beam]<19.99:continue
                            lo,hi=bounds;point=world_hits[k,beam]
                            endpoint_audit.append(float(np.linalg.norm(np.maximum(np.maximum(lo-point,point-hi),0))))
                # A 48 degree sector rotates once every 1.8 seconds. Raw sensors
                # provide 360 deg; only the displayed sector updates the reveal.
                spin=(t-intro)*2*math.pi/1.8
                relative=np.arctan2(np.sin(angles+pose[2]-spin),np.cos(angles+pose[2]-spin))
                active=np.flatnonzero(abs(relative)<math.radians(24))
                active=active[np.argsort(relative[active])]
                for k in range(layer_count):
                    # Interpolate the one-degree measured rays into a solid
                    # sector for display; no geometry-derived free space.
                    if k in lower_layers:
                        vv=(distance[k,active]>=.15)&(distance[k,active]<19.99)
                        polygon=np.r_[[sensor_origin[:2]],world_hits[k,active[vv],:2]]
                        pixels=np.c_[(polygon[:,0]-MAP_SPEC.xmin)/MAP_SPEC.resolution,
                                     (polygon[:,1]-MAP_SPEC.ymin)/MAP_SPEC.resolution]
                        if len(pixels)>=3:cv2.fillPoly(display_seen,[np.rint(pixels).astype(np.int32)],1)
                    for beam in active:
                        idx=part_paths.get(str(hit_paths[k,beam]),0)
                        if idx:hit_parts.add(idx)
                composer.observe(display_seen.astype(bool),hit_parts,t)
                recorded_angles=angles;recorded_zenith=zenith;raw_history.append(raw);scan_frames.append(i)
                point_history.append(local.astype(np.float32));sensor_poses.append(sensor_origin)
                hit_history.append(sorted(hit_parts))
                # Native 3D ray endpoints, drawn from the actual sensor origin.
                points=[];returns=[]
                for k in range(0,layer_count,3):
                    for beam in active[::6]:
                        if not .15<=distance[k,beam]<19.99:continue
                        end=world_hits[k,beam]
                        points.extend([sensor_origin,end]);returns.append(end)
                laser.CreateCurveVertexCountsAttr([2]*(len(points)//2))
                laser.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in points])
                endpoints.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in returns])
                poly=[(sensor_origin[0],sensor_origin[1],.048)]
                poly.extend((world_hits[horizontal,j,0],world_hits[horizontal,j,1],.048) for j in active if .15<=distance[horizontal,j]<19.99)
                fan.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in poly])
                fan.CreateFaceVertexCountsAttr([3]*max(0,len(poly)-2))
                fan.CreateFaceVertexIndicesAttr([v for j in range(1,len(poly)-1) for v in [0,j,j+1]])
                for obj in [laser,fan,endpoints]:UsdGeom.Imageable(obj).MakeVisible()
                add_trail=not trail_points or math.dist(trail_points[-1][:2],pose[:2])>.015
                if add_trail:trail_points.append((float(pose[0]),float(pose[1]),.05))
                if add_trail and len(trail_points)>=2:
                    trail.CreateCurveVertexCountsAttr([len(trail_points)])
                    trail.CreatePointsAttr([Gf.Vec3f(*p) for p in trail_points])
            world.step(render=True)
            rgb,seg,labels=frame_data()
            native_mean=float(rgb[scene_mask].mean())
            if native_mean<35:
                Image.fromarray(rgb).save(out/f'native_frame_failure_{i:04d}.png')
                raise AssertionError(f'Native RGB became unexpectedly dark at frame {i}: {native_mean}')
            robot_ids=[int(k) for k,v in labels.items() if v.get('class')=='robot']
            scan_ids=[int(k) for k,v in labels.items() if v.get('class')=='scan']
            robot_mask=np.isin(seg,robot_ids);scan_mask=np.isin(seg,scan_ids)
            if i>5 and robot_mask.sum()<10:raise AssertionError('Robot missing from native render')
            frame=composer.compose(rgb,robot_mask,scan_mask,t,pose,progress,route,phase,len(raw_history))
            encoder.stdin.write(np.asarray(frame).tobytes())
            if i in fractions:
                filename=f'frame_{i:04d}.png';frame.save(out/filename);stills.append((t,filename))
            history.append(dict(time_s=t,pose=pose.tolist(),command_world_vx_vy_wz=command.tolist(),route_progress=progress))
            areas.append(float(np.isfinite(composer.first[16:256,16:376]).mean()))
            meshcounts.append(int(np.isfinite(composer.part_first).sum()))
            if i%(a.fps*2)==0:print(f'REVEAL_FRAME {i}/{len(motion)} time={t:.1f}s floor={areas[-1]:.3f} parts={meshcounts[-1]} rgb={native_mean:.1f}',flush=True)
    finally:
        encoder.stdin.close();errors=encoder.stderr.read().decode('utf-8',errors='replace');code=encoder.wait()
    if code:raise RuntimeError('Video encoding failed: '+errors)
    if not raw_history or max(areas)<.3:raise AssertionError('Insufficient scan reveal')
    if not np.all(np.diff(areas)>=0) or not np.all(np.diff(meshcounts)>=0):raise AssertionError('Reveal regressed')
    if not endpoint_audit or np.percentile(endpoint_audit,95)>.03:raise AssertionError(f'3D endpoint coordinate audit failed: {endpoint_audit}')
    np.savez_compressed(out/'animation_scans.npz',raw_depth_uint16=np.array(raw_history),angles=recorded_angles,zenith=recorded_zenith,
        local_hit_points=np.array(point_history),sensor_world_positions=np.array(sensor_poses),
        frame_indices=scan_frames,poses=np.array([h['pose'] for h in history]),
        first_observed_time=composer.first,part_first_hit_time=composer.part_first,
        xy_coverage_fraction=areas,hit_part_count=meshcounts)
    (out/'motion_commands.json').write_text(json.dumps(history),encoding='utf-8')
    (out/'hit_parts.json').write_text(json.dumps(dict(parts=part_paths,frame_hits=hit_history)),encoding='utf-8')
    # Export is an independent overlay; room and earlier navigation files stay untouched.
    stage.GetRootLayer().Save()
    sheet=Image.new('RGB',(1920,1250),'#0b1722');draw=ImageDraw.Draw(sheet)
    draw.text((40,24),'从暗处出发，让扫描逐步照亮房间',font=font(39,True),fill='#edf7f7')
    for j,(t,name) in enumerate([stills[0],stills[1],stills[2],stills[-1]]):
        pic=Image.open(out/name);pic.thumbnail((925,520))
        x=25+(j%2)*950;y=100+(j//2)*560
        sheet.paste(pic,(x,y));draw.text((x+15,y+15),f'{t:.1f} s',font=font(25,True),fill='#82efd5')
    sheet.save(out/'storyboard.png')
    # GIF uses a separate palette for a portable inline preview.
    gif=out/'code_as_room_lidar_reveal.gif'
    filters='fps=8,scale=960:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4'
    subprocess.run([FFMPEG,'-y','-hide_banner','-loglevel','error','-i',str(video),'-filter_complex',filters,'-loop','0',str(gif)],check=True)
    result=dict(status='passed',simulator='Isaac Sim 4.5.0',video=str(video),gif=str(gif),
        robot_asset=str(robot_asset),robot_pose_source='wheeled_arm_v2 training nominal joint configuration; URDF forward kinematics',
        fps=a.fps,resolution=[width,height],native_scene_resolution=[render_width,height],
        frames=len(motion),duration_s=len(motion)/a.fps,scan_frames=len(raw_history),
        raw_rays=len(raw_history)*layer_count*360,vertical_channels=layer_count,vertical_fov_deg=80,
        endpoint_audit_p95_m=float(np.percentile(endpoint_audit,95)),final_xy_floor_coverage_fraction=areas[-1],
        revealed_mesh_parts=meshcounts[-1],total_mesh_parts=len(part_paths),
        reveal_scope='measured 3D lidar downward/horizontal visibility projected to XY + actual hit mesh parts; explanatory brightness, not reconstructed surfaces',
        motion='integrated world-frame planar velocity/yaw commands on preset route; no policy or dynamics',
        monotonically_accumulated=True,render_only_hidden_walls=hidden,
        runtime_s=time.perf_counter()-begin)
    (out/'render_status.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    (out/'preview.html').write_text('''<!doctype html><meta charset="utf-8"><title>Code-as-Room 雷达探索</title><style>body{background:#0b1722;color:#edf7f7;font:18px/1.7 "Microsoft YaHei";margin:35px}video,img{max-width:100%;border-radius:14px}main{max-width:1500px;margin:auto}a{color:#79e0c7}</style><main><h1>Code-as-Room：雷达扫描逐步点亮房间</h1><video controls autoplay muted loop poster="frame_0000.png" src="code_as_room_lidar_reveal.mp4"></video><p>Isaac Sim 原生三维渲染，使用 wheeled_arm_v2 的弯腿、收臂姿态。机器人沿预设路线运动，虚拟三维雷达实际测距；亮度表示向下及水平扫描的地面可见投影与命中家具部件，未观测区域保持暗色。家具采用部件级显现，并不表示每个表面都被测量。运动由平面指令直接更新模型，无 RL 策略。</p><p><a href="code_as_room_lidar_reveal.mp4">MP4 视频</a> · <a href="code_as_room_lidar_reveal.gif">GIF 动图</a> · <a href="robot_pose_closeup.png">机器人姿态近景</a></p><img src="storyboard.png"></main>''',encoding='utf-8')
    print('REVEAL_COMPLETE '+json.dumps(result,ensure_ascii=False),flush=True)
    world.stop()

try:
    main()
except Exception:
    import traceback
    traceback.print_exc();sys.stderr.flush();sys.stdout.flush()
    a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'render_status.json').write_text(json.dumps({'status':'failed','error':traceback.format_exc()}),encoding='utf-8')
    raise
finally:app.close()
