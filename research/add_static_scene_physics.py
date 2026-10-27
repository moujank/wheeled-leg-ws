"""Add a separate physics overlay to a converted static room, preserving source visuals."""
import json
import os
import sys
from pathlib import Path

root=Path(__file__).resolve().parent/'scene_audit/code_as_room'
isaac=Path(os.environ.get('ISAAC_PATH','D:/isaacsim'))
handles=[]
for folder in [isaac/'kit',isaac/'kit/lib',*isaac.glob('extscache/omni.usd.libs-*/bin')]:
    if folder.exists(): handles.append(os.add_dll_directory(str(folder)))
sys.path.insert(0,str(next(isaac.glob('extscache/omni.usd.libs-*'))))
from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,Gf,UsdUtils

visual=root/'visual_scene.usdc'
target=root/'scene.usda'
stage=Usd.Stage.Open(str(target)) if target.exists() else Usd.Stage.CreateNew(str(target))
stage.GetRootLayer().subLayerPaths=['visual_scene.usdc']
stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
UsdGeom.SetStageMetersPerUnit(stage,1.)
UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
physics=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));physics.CreateGravityMagnitudeAttr(9.81)
light=UsdLux.DomeLight.Define(stage,'/World/NavigationLighting')
light.CreateIntensityAttr(1000.)
mesh_paths=[]
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
        mesh_paths.append(str(prim.GetPath()))
stage.GetRootLayer().customLayerData={'source':'Local Code-as-Room GLB output',
    'physicsScope':'Static triangle-mesh colliders; not robot navigation verified'}
stage.GetRootLayer().Save()
_,_,missing=UsdUtils.ComputeAllDependencies(str(target))
report={'scene':str(target),'static_triangle_mesh_colliders':len(mesh_paths),
        'unresolved_assets':[str(x) for x in missing],
        'scope':'USD and collision authoring only; physics/sensor tests are separate'}
(root/'physics_overlay.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
