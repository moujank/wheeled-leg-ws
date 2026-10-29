"""Author a separate USD room variant using Isaac's native USD scene APIs."""
from pathlib import Path
import argparse,hashlib,json,traceback
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--remove-lounge-table',action='store_true')
args=parser.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':720,'renderer':'RayTracedLighting',
    'anti_aliasing':0,'fast_shutdown':True,
    'extra_args':['--/plugins/carb.tasking.plugin/threadCount=4','--/rtx/hydra/enableMultithreading=false']})
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'research/scene_audit'/('code_as_room_open_passages_no_table' if args.remove_lounge_table else 'code_as_room_open_passages')
OUT.mkdir(parents=True,exist_ok=True)

def main():
    import numpy as np
    from pxr import Usd,UsdGeom,UsdPhysics,Gf
    import omni.usd
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.api import World
    from isaacsim.sensors.camera import Camera
    from PIL import Image
    source=ROOT/'research/scene_audit/code_as_room/scene.usda'
    visual=source.parent/'visual_scene.usdc'
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,visual)}
    dest=OUT/'scene.usda'
    import os
    layer=Usd.Stage.CreateInMemory();layer.GetRootLayer().subLayerPaths=[os.path.relpath(source,OUT).replace('\\','/')]
    UsdGeom.SetStageUpAxis(layer,'Z');UsdGeom.SetStageMetersPerUnit(layer,1.)
    layer.GetRootLayer().Export(str(dest));open_stage(str(dest))
    stage=omni.usd.get_context().get_stage()
    for _ in range(8):app.update()
    def bounds(prim,cache):
        r=cache.ComputeWorldBound(prim).ComputeAlignedRange()
        return np.array(r.GetMin()),np.array(r.GetMax())
    bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
    xf_cache=UsdGeom.XformCache()
    roots=[]
    for prim in stage.GetPrimAtPath('/World').GetChildren():
        if not any(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(prim)):continue
        lo,hi=bounds(prim,bbox)
        roots.append((prim,lo,hi,xf_cache.GetLocalToWorldTransform(prim),
                      xf_cache.GetLocalToWorldTransform(prim.GetParent())))
    edits=[];extension=.65;removed=[];removed_colliders=0
    if args.remove_lounge_table:
        for name in ('obj_003_Small_Side_Table','obj_004_Tabletop_Book','obj_004_Tabletop_Book__top__Tabletop_Book'):
            prim=stage.GetPrimAtPath('/World/'+name)
            assert prim.IsValid(),name
            count=sum(p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Boundable) for p in Usd.PrimRange(prim))
            removed_colliders+=count
            removed.append({'prim':str(prim.GetPath()),'removed_colliders':count})
            prim.SetActive(False)
    for prim,lo,hi,old_world,parent_world in roots:
        if not prim.IsActive():continue
        name=prim.GetName();newlo,newhi=float(lo[1]),float(hi[1]);reason=None
        if name=='Floor':
            newlo-=extension;newhi+=extension;reason='extend floor at both ends'
        elif name=='North_Boundary_Wall':
            newlo+=extension;newhi+=extension;reason='north exterior wall follows floor'
        elif name.startswith('South_Boundary_Wall'):
            newlo-=extension;newhi-=extension;reason='south exterior wall follows floor'
        elif 'Partition' in name or name=='East_Boundary_Wall' or name.startswith('West_Boundary_Wall'):
            if lo[1]<-2.99:newlo-=extension
            if hi[1]>2.99:newhi+=extension
            if newlo!=float(lo[1]) or newhi!=float(hi[1]):reason='extend wall; keep doorway endpoints fixed'
        elif lo[1]>=1.8:
            newlo+=extension;newhi+=extension;reason='move north-end furniture and attached decor together'
        elif hi[1]<=-1.5:
            newlo-=extension;newhi-=extension;reason='move south-end furniture and attached decor together'
        if reason is None:continue
        scale=(newhi-newlo)/float(hi[1]-lo[1]);offset=newlo-scale*float(lo[1])
        affine=Gf.Matrix4d(1.);affine.SetScale(Gf.Vec3d(1.,scale,1.))
        affine.SetTranslateOnly(Gf.Vec3d(0.,offset,0.))
        new_local=old_world*affine*parent_world.GetInverse()
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(new_local)
        edits.append({'prim':str(prim.GetPath()),'reason':reason,'old_y':[float(lo[1]),float(hi[1])],
                      'new_y':[newlo,newhi],'scale_y':scale,'translation_y':offset})
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    # Keep the indoor demo enclosed with a real closed door at the existing
    # exterior opening; the kinematic robot must not explore off the floor.
    door=UsdGeom.Cube.Define(stage,'/World/ClosedExteriorDoor');door.CreateSizeAttr(1.)
    door.AddTranslateOp().Set(Gf.Vec3d(3.10,-3.725,1.15))
    door.AddScaleOp().Set(Gf.Vec3f(.90,.15,2.30))
    door.CreateDisplayColorAttr([Gf.Vec3f(.28,.17,.09)])
    UsdPhysics.CollisionAPI.Apply(door.GetPrim())
    stage.GetRootLayer().Save()
    bbox.Clear();colliders=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Boundable) and prim.HasAPI(UsdPhysics.CollisionAPI):
            lo,hi=bounds(prim,bbox)
            colliders.append({'path':str(prim.GetPath()),'min':lo.tolist(),'max':hi.tolist()})
    (OUT/'scene_colliders.json').write_text(json.dumps(colliders,indent=2),encoding='utf-8')
    def group_bounds(name):return bounds(stage.GetPrimAtPath('/World/'+name),bbox)
    bed=group_bounds('obj_008_Queen_King_Bed');throw=group_bounds('Bed_Throw')
    bench=group_bounds('obj_017_Foot_Bed_Bench')
    left=group_bounds('obj_010_Left_Nightstand');right=group_bounds('obj_011_Right_Nightstand')
    floor=group_bounds('Floor')
    head=min(left[0][1],right[0][1])-bed[1][1]
    foot=min(bed[0][1],throw[0][1])-bench[1][1]
    assert head>1.2 and foot>1.2,(head,foot)
    assert len(colliders)==223-removed_colliders,len(colliders)
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in hashes.items())
    metadata={'status':'prepared','scene_usd':str(dest),'source_scene':str(source),
        'source_hashes_unchanged':hashes,'method':'Native Isaac USD world transforms in an additive layer',
        'floor_bounds':[float(floor[0][0]),float(floor[0][1]),float(floor[1][0]),float(floor[1][1])],
        'floor_area_m2':float(np.prod((floor[1]-floor[0])[:2])),
        'head_gap_m':float(head),'foot_gap_m':float(foot),'static_mesh_colliders':222-removed_colliders,
        'total_static_colliders':len(colliders),
        'robot_scale_changed':False,'bed_scale_changed':False,'both_interior_partitions_retained':True,
        'original_colliders_retained':222-removed_colliders,'removed_objects':removed,
        'removed_mesh_colliders':removed_colliders,'added_colliders':['/World/ClosedExteriorDoor'],
        'edits':edits}
    (OUT/'scene_metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    # Preview-only session edits: show the correct robot, and cut away only
    # the near exterior wall. Both interior partitions remain visible.
    stage.SetEditTarget(stage.GetSessionLayer())
    robot=UsdGeom.Xform.Define(stage,'/World/PreviewRobot')
    robot.AddTranslateOp().Set(Gf.Vec3d(.9,-1.4,0.))
    model=UsdGeom.Xform.Define(stage,'/World/PreviewRobot/Model')
    model.GetPrim().GetReferences().AddReference((source.parent/'wheeled_arm_v2_bent_visual.usdc').as_posix())
    for prim in stage.GetPrimAtPath('/World').GetChildren():
        if prim.GetName().startswith('South_Boundary_Wall'):UsdGeom.Imageable(prim).MakeInvisible()
    cam=UsdGeom.Camera.Define(stage,'/World/OpenPassagesPreview')
    cam.CreateFocalLengthAttr(46.);cam.CreateHorizontalApertureAttr(36.);cam.CreateVerticalApertureAttr(27.)
    cam.CreateClippingRangeAttr(Gf.Vec2f(.1,100.))
    world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/60,physics_prim_path='/World/PhysicsScene')
    world.reset();camera=Camera(str(cam.GetPath()),resolution=(1440,1080));camera.initialize()
    xf=UsdGeom.Xformable(stage.GetPrimAtPath('/World/OpenPassagesPreview'));xf.ClearXformOpOrder()
    op=xf.AddTransformOp()
    op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(3,-4.5,18),Gf.Vec3d(0,0,.3),Gf.Vec3d(0,0,1)).GetInverse())
    for _ in range(50):world.step(render=True)
    rgb=np.asarray(camera.get_rgb())
    if rgb.shape!=(1080,1440,3):raise RuntimeError(f'Preview camera not ready: {rgb.shape}')
    Image.fromarray(rgb).save(OUT/'isaac_open_passages.png')
    op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(0,0,17),Gf.Vec3d(0,0,0),Gf.Vec3d(0,1,0)).GetInverse())
    for _ in range(25):world.step(render=True)
    Image.fromarray(np.asarray(camera.get_rgb())).save(OUT/'isaac_topdown.png')
    metadata['status']='prepared_and_rendered'
    (OUT/'scene_metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    print('OPEN_PASSAGES '+json.dumps(metadata),flush=True)

try:main()
except BaseException as exc:
    traceback.print_exc()
    (OUT/'prepare_error.json').write_text(json.dumps({'error':str(exc),'traceback':traceback.format_exc()}),encoding='utf-8')
finally:app.close()
