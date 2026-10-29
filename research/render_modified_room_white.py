"""Render the modified USD room on pure white using native Isaac RGB/depth."""
from pathlib import Path
import argparse,json,traceback
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--transparent-topdown',action='store_true')
args=parser.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':720,'renderer':'RayTracedLighting',
    'anti_aliasing':0,'fast_shutdown':True,
    'extra_args':['--/plugins/carb.tasking.plugin/threadCount=4','--/rtx/hydra/enableMultithreading=false']})
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'modules/wheeled_navigation/artifacts/clean_visuals'
OUT.mkdir(exist_ok=True)
try:
    import numpy as np
    import omni.usd
    from pxr import UsdGeom,Gf
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.api import World
    from isaacsim.sensors.camera import Camera
    from PIL import Image
    source=ROOT/'research/scene_audit/code_as_room_open_passages_no_table/scene.usda'
    open_stage(str(source));stage=omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    # Only the near outer wall is cut away for the architectural view.
    # Both interior partitions stay visible; the source scene is never saved.
    for prim in stage.GetPrimAtPath('/World').GetChildren():
        if prim.GetName().startswith('South_Boundary_Wall') or prim.GetName()=='ClosedExteriorDoor':
            UsdGeom.Imageable(prim).MakeInvisible()
    cam=UsdGeom.Camera.Define(stage,'/World/WhiteBackgroundPreview')
    cam.CreateFocalLengthAttr(46.);cam.CreateHorizontalApertureAttr(36.)
    cam.CreateVerticalApertureAttr(27.);cam.CreateClippingRangeAttr(Gf.Vec2f(.1,100.))
    world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/60)
    world.reset();camera=Camera(str(cam.GetPath()),resolution=(2400,1800));camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    if args.transparent_topdown:
        # Orthographic projection makes both wall directions exactly parallel
        # to the image edges, with no perspective convergence or in-plane roll.
        cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        cam.GetHorizontalApertureAttr().Set(110.)
        cam.GetVerticalApertureAttr().Set(82.5)
    xf=UsdGeom.Xformable(stage.GetPrimAtPath('/World/WhiteBackgroundPreview'))
    xf.ClearXformOpOrder();op=xf.AddTransformOp()
    outputs=[]
    views=[('scene_white_background.png',(3,-4.5,18),(0,0,.3),(0,0,1)),
           ('scene_white_topdown.png',(0,0,17),(0,0,0),(0,1,0))]
    if args.transparent_topdown:
        views=[('scene_topdown_transparent.png',(0,0,17),(0,0,0),(0,1,0))]
    for index,(name,eye,target,up) in enumerate(views):
        op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(*up)).GetInverse())
        for _ in range(45 if index==0 else 15):world.step(render=True)
        rgb=np.array(camera.get_rgb(),copy=True);depth=np.asarray(camera.get_depth())
        assert rgb.shape==(1800,2400,3),rgb.shape
        foreground=np.isfinite(depth)&(depth>.1)&(depth<100.)
        if args.transparent_topdown:
            rgba=np.concatenate([rgb,(foreground.astype(np.uint8)*255)[...,None]],axis=2)
            rgba[~foreground,:3]=0
            image=Image.fromarray(rgba)
            box=image.getchannel('A').getbbox();assert box is not None
            assert box[0]>0 and box[1]>0 and box[2]<2400 and box[3]<1800,box
            image=image.crop(box)
            padded=Image.new('RGBA',(image.width+48,image.height+48),(0,0,0,0))
            padded.paste(image,(24,24));padded.save(OUT/name)
            resolution=list(padded.size)
        else:
            rgb[~foreground]=255
            Image.fromarray(rgb).save(OUT/name)
            assert np.all(rgb[0,0]==255)
            resolution=[2400,1800]
        outputs.append({'file':str(OUT/name),'resolution':resolution,
                        'background':'Transparent alpha from native depth' if args.transparent_topdown else 'Pure white RGB(255,255,255); native depth separates environment from scene',
                        'foreground_fraction':float(foreground.mean())})
    report_name='scene_topdown_transparent.json' if args.transparent_topdown else 'scene_white_render.json'
    (OUT/report_name).write_text(json.dumps({'status':'rendered','source':str(source),
        'projection':'orthographic' if args.transparent_topdown else 'perspective',
        'native_isaac_render':True,'interior_partitions_visible':True,'near_exterior_wall_cutaway':True,
        'source_scene_modified':False,'outputs':outputs},indent=2),encoding='utf-8')
    print('WHITE_RENDER_COMPLETE',flush=True)
except BaseException:
    traceback.print_exc();(OUT/'white_render_error.txt').write_text(traceback.format_exc(),encoding='utf-8')
finally:app.close()
