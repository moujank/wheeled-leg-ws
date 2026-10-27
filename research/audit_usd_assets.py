"""Inspect local USDs using Isaac's bundled USD libs, without a GPU/Kit process."""
import argparse
import json
import os
import sys
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('--isaac-root',type=Path,default=Path(os.environ.get('ISAAC_PATH','D:/isaacsim')))
p.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'scene_audit')
p.add_argument('scenes',nargs='+',type=Path)
a=p.parse_args()
handles=[]
for folder in [a.isaac_root/'kit',a.isaac_root/'kit/lib',*a.isaac_root.glob('extscache/omni.usd.libs-*/bin')]:
    if folder.exists():
        handles.append(os.add_dll_directory(str(folder)))
sys.path.insert(0,str(next(a.isaac_root.glob('extscache/omni.usd.libs-*'))))
from pxr import Usd,UsdGeom,UsdPhysics,UsdUtils

a.output.mkdir(parents=True,exist_ok=True)
reports=[]
for source in a.scenes:
    stage=Usd.Stage.Open(str(source.resolve()))
    if not stage:
        raise RuntimeError(f'Could not open {source}')
    prims=list(stage.Traverse())
    _,assets,unresolved=UsdUtils.ComputeAllDependencies(str(source.resolve()))
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.guide],useExtentsHint=True)
    collisions=[]
    for prim in prims:
        if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False:
            box=cache.ComputeWorldBound(prim).ComputeAlignedRange()
            collisions.append({'path':str(prim.GetPath()),'min':list(box.GetMin()),'max':list(box.GetMax())})
    name=source.parent.parent.name if source.parent.name=='asset' else source.parent.name
    wrapper=a.output/f'{name}_navigation.usda'
    if wrapper.exists():
        layer=Usd.Stage.Open(str(wrapper))
    else:
        layer=Usd.Stage.CreateNew(str(wrapper))
    layer.GetRootLayer().subLayerPaths=[str(source.resolve()).replace('\\','/')]
    layer.GetRootLayer().customLayerData={'sourceScene':str(source.resolve()),'purpose':'Navigation scene candidate; no robot/policy validation yet'}
    UsdGeom.SetStageMetersPerUnit(layer,UsdGeom.GetStageMetersPerUnit(stage))
    UsdGeom.SetStageUpAxis(layer,UsdGeom.GetStageUpAxis(stage))
    default=stage.GetDefaultPrim()
    if default:
        layer.SetDefaultPrim(layer.GetPrimAtPath(default.GetPath()))
    layer.GetRootLayer().Save()
    report={'source':str(source.resolve()),'wrapper':str(wrapper.resolve()),
            'meshes':sum(pr.IsA(UsdGeom.Mesh) for pr in prims),'enabled_colliders':len(collisions),
            'rigid_bodies':sum(pr.HasAPI(UsdPhysics.RigidBodyAPI) for pr in prims),
            'physics_scenes':sum(pr.IsA(UsdPhysics.Scene) for pr in prims),
            'meters_per_unit':UsdGeom.GetStageMetersPerUnit(stage),'up_axis':str(UsdGeom.GetStageUpAxis(stage)),
            'unresolved_assets':[str(x) for x in unresolved],'source_modified':False,
            'validation':'USD composition/dependencies only; not a fresh physics rollout'}
    (a.output/f'{name}_colliders.json').write_text(json.dumps(collisions,indent=2),encoding='utf-8')
    reports.append(report)
    print(json.dumps(report),flush=True)
(a.output/'usd_report.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
