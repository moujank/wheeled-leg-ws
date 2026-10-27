"""Record native PhysX ray directions for the exact navigation lidar layout."""
from pathlib import Path
import json,traceback
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','fast_shutdown':True,
    'extra_args':['--/plugins/carb.tasking.plugin/threadCount=4']})
OUT=Path(__file__).resolve().parent/'scene_audit/lidar_calibration'
OUT.mkdir(parents=True,exist_ok=True)
try:
    import numpy as np
    import omni.usd,omni.kit.commands
    from pxr import UsdGeom,UsdPhysics,Gf
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.Xform.Define(stage,'/World')
    # A closed box ensures every calibration ray has a real return.
    for i,(centre,scale) in enumerate([((3,0,0),(.1,6,6)),((-3,0,0),(.1,6,6)),
        ((0,3,0),(6,.1,6)),((0,-3,0),(6,.1,6)),((0,0,3),(6,6,.1)),((0,0,-3),(6,6,.1))]):
        cube=UsdGeom.Cube.Define(stage,f'/World/Wall{i}');cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre));cube.AddScaleOp().Set(Gf.Vec3f(*scale))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    enable_extension('isaacsim.sensors.physx')
    from isaacsim.sensors.physx import _range_sensor
    ok,lidar=omni.kit.commands.execute('RangeSensorCreateLidar',path='/World/Lidar',parent=None,
        min_range=.15,max_range=8.,draw_points=False,draw_lines=False,horizontal_fov=360,
        vertical_fov=140,horizontal_resolution=.5,vertical_resolution=1.,rotation_rate=0.,high_lod=True,yaw_offset=0.)
    assert ok
    world=World(stage_units_in_meters=1.,physics_dt=1/30,rendering_dt=1/30)
    world.reset()
    for _ in range(30):world.step(render=True)
    sensor=_range_sensor.acquire_lidar_sensor_interface()
    angles=np.array(sensor.get_azimuth_data('/World/Lidar'),copy=True).reshape(-1)
    raw=np.array(sensor.get_depth_data('/World/Lidar'),copy=True).reshape(len(angles),-1).T
    cloud=np.array(sensor.get_point_cloud_data('/World/Lidar'),copy=True).reshape(len(angles),-1,3).transpose(1,0,2)
    norm=np.linalg.norm(cloud,axis=2)
    assert np.all((norm>.15)&(norm<7.99)),(norm.min(),norm.max())
    directions=cloud/norm[...,None]
    error=float(np.max(np.abs(norm-raw*8./65535.)))
    assert error<.001,error
    np.savez_compressed(OUT/'native_rays.npz',directions=directions,angles=angles,
        zenith=np.array(sensor.get_zenith_data('/World/Lidar'),copy=True),
        calibration_points=cloud,calibration_raw_depth=raw)
    (OUT/'calibration.json').write_text(json.dumps({'status':'validated','shape':list(directions.shape),
        'max_depth_quantization_error_m':error,'method':'Unit directions measured from native PhysX point-cloud API in a closed calibration box',
        'horizontal_fov_degrees':360,'vertical_fov_degrees':140,'horizontal_resolution_degrees':.5,
        'vertical_resolution_degrees':1.,'range_m':[.15,8.]},indent=2),encoding='utf-8')
    print('CALIBRATION_OK',error,flush=True)
except BaseException:
    traceback.print_exc();(OUT/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
finally:app.close()
