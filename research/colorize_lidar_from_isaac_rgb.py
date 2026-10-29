"""Register native Isaac RGB views to already-recorded lidar returns.

Replays camera locations for appearance only; no new navigation or geometry is
generated. Point positions in the output remain the existing SLAM positions.
Recorded simulator poses register native camera pixels with their original
sensor rays; those poses are never substituted into the reconstructed geometry.
"""
from pathlib import Path
import argparse,json,traceback
p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
p.add_argument('--views',type=int,default=26)
a=p.parse_args();RUN=a.run.resolve();OUT=RUN/'reconstruction'
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':768,'height':768,'renderer':'RayTracedLighting',
    'anti_aliasing':0,'fast_shutdown':True,
    'extra_args':['--/plugins/carb.tasking.plugin/threadCount=4','--/rtx/hydra/enableMultithreading=false']})
try:
    import numpy as np
    from pxr import UsdGeom,Gf
    import omni.usd
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.api import World
    from isaacsim.sensors.camera import Camera
    from PIL import Image
    status=json.loads((RUN/'status.json').read_text())
    open_stage(status['scene_usd']);stage=omni.usd.get_context().get_stage()
    # All edits remain in memory. The room's saved USD is never modified.
    for _ in range(8):app.update()
    samples=np.load(OUT/'cloud_samples.npz')
    points=samples['points'];hits=samples['local_hits'];ids=samples['source_frames']
    with np.load(RUN/'sensor_history.npz') as h:truth=h['truth'];times=h['times']
    pose=truth[ids];cs=np.cos(pose[:,2]);sn=np.sin(pose[:,2])
    xyz=np.empty_like(hits)
    xyz[:,0]=pose[:,0]+cs*(hits[:,0]+.16)-sn*hits[:,1]
    xyz[:,1]=pose[:,1]+sn*(hits[:,0]+.16)+cs*hits[:,1]
    xyz[:,2]=hits[:,2]+.84
    path=np.r_[0,np.cumsum(np.linalg.norm(np.diff(truth[:,:2],axis=0),axis=1))]
    frame_ids=np.unique(np.searchsorted(path,np.linspace(0,path[-1],a.views)))
    n=768;camprim=UsdGeom.Camera.Define(stage,'/World/RgbRegistrationCamera')
    camprim.CreateFocalLengthAttr(18.);camprim.CreateHorizontalApertureAttr(36.)
    camprim.CreateVerticalApertureAttr(36.);camprim.CreateClippingRangeAttr(Gf.Vec2f(.05,30.))
    world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/60)
    world.reset();camera=Camera(str(camprim.GetPath()),resolution=(n,n));camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    xf=UsdGeom.Xformable(stage.GetPrimAtPath('/World/RgbRegistrationCamera'))
    xf.ClearXformOpOrder();op=xf.AddTransformOp()
    colors=np.full(points.shape,180,dtype=np.uint8);best=np.full(len(points),np.inf)
    face_axes=[((1,0,0),(0,0,1)),((-1,0,0),(0,0,1)),((0,1,0),(0,0,1)),
        ((0,-1,0),(0,0,1)),((0,0,1),(0,1,0)),((0,0,-1),(0,1,0))]
    captures=[]
    def capture(frame,pass_name):
        pose=truth[frame];origin=np.array([pose[0]+.16*np.cos(pose[2]),pose[1]+.16*np.sin(pose[2]),.84])
        delta=xyz-origin
        before=int(np.isfinite(best).sum())
        for face,(direction,up) in enumerate(face_axes):
            forward=np.asarray(direction,dtype=float);up=np.asarray(up,dtype=float)
            right=np.cross(forward,up);vertical=np.cross(right,forward)
            depth=delta@forward
            ids_view=np.flatnonzero((depth>.15)&(abs(delta@right)<depth*.995)&(abs(delta@vertical)<depth*.995))
            if not len(ids_view):continue
            op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*origin),Gf.Vec3d(*(origin+forward)),Gf.Vec3d(*up)).GetInverse())
            for _ in range(8 if not captures and face==0 else 3):world.step(render=True)
            rgb=np.asarray(camera.get_rgb());image_depth=camera.get_depth()
            if rgb.shape!=(n,n,3) or image_depth is None:raise RuntimeError('RGB/depth camera is not ready')
            u=np.clip((n/2+(delta[ids_view]@right)/depth[ids_view]*(n/2)).astype(int),0,n-1)
            v=np.clip((n/2-(delta[ids_view]@vertical)/depth[ids_view]*(n/2)).astype(int),0,n-1)
            measured=np.asarray(image_depth)[v,u]
            diff=abs(measured-depth[ids_view])
            visible=np.isfinite(measured)&(diff<.055)
            quality=depth[ids_view]+diff*30
            use=visible&(quality<best[ids_view])
            selected=ids_view[use];colors[selected]=rgb[v[use],u[use]];best[selected]=quality[use]
            if len(captures)<2 and face in (0,5):Image.fromarray(rgb).save(OUT/f'rgb_reference_{frame}_{face}.png')
        captures.append({'frame':int(frame),'seconds':float(times[frame]),'pass':pass_name,
            'newly_colored_points':int(np.isfinite(best).sum())-before})
        print('RGB_PROGRESS',len(captures),frame,int(np.isfinite(best).sum()),len(points),flush=True)
    for frame in frame_ids:capture(int(frame),'trajectory_keyframes')
    # Remaining surfaces may be visible only from their original acquisition
    # location. Render those actual recorded locations, with no new route.
    for _ in range(12):
        missing=~np.isfinite(best)
        if missing.sum()<max(100,len(points)*.002):break
        frames,counts=np.unique(ids[missing],return_counts=True)
        order=np.argsort(counts)[::-1]
        candidates=[int(frames[i]) for i in order if int(frames[i]) not in {q['frame'] for q in captures}]
        if not candidates:break
        capture(candidates[0],'original_return_view')
    direct=np.isfinite(best);interpolated=np.zeros(len(points),dtype=bool)
    if np.any(~direct) and np.any(direct):
        from scipy.spatial import cKDTree
        known=np.flatnonzero(direct);unknown=np.flatnonzero(~direct)
        distance,neighbor=cKDTree(xyz[known]).query(xyz[unknown])
        use=distance<.065;selected=unknown[use]
        colors[selected]=colors[known[neighbor[use]]];interpolated[selected]=True
    np.savez_compressed(OUT/'native_rgb_colors.npz',points=points,colors=colors,
        direct_rgb=direct,nearby_rgb=interpolated)
    report={'status':'completed','points':len(points),'direct_rgb_points':int(direct.sum()),
        'nearby_rgb_points':int(interpolated.sum()),'uncolored_neutral_gray_points':int((~direct&~interpolated).sum()),
        'point_positions_unchanged':True,'colors':'Native RGB camera rendering of the actual scene materials and lighting; no height palette.',
        'capture_method':'Six 90-degree RGB/depth camera views at recorded sensor locations, after navigation.',
        'occlusion_depth_tolerance_m':.055,'nearby_rgb_max_distance_m':.065,
        'geometry_pose_source':'Unchanged SLAM Toolbox positions from cloud_samples.npz',
        'appearance_registration_pose_source':'Recorded simulator sensor poses used only to replay RGB views and associate rays with rendered surfaces.',
        'not_synchronized_rgb_recording':True,'frames':captures}
    (OUT/'rgb_registration_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('RGB_COMPLETE',report['direct_rgb_points'],report['nearby_rgb_points'],report['uncolored_neutral_gray_points'],flush=True)
except BaseException:
    traceback.print_exc();(OUT/'rgb_registration_error.txt').write_text(traceback.format_exc(),encoding='utf-8')
finally:app.close()
