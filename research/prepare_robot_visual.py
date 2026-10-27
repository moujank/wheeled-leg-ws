"""Create a physics/sensor-free visual copy, without editing the robot source."""
import os,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
isaac=Path(os.environ.get('ISAAC_PATH','D:/isaacsim'));handles=[]
for d in [isaac/'kit',isaac/'kit/lib',*isaac.glob('extscache/omni.usd.libs-*/bin')]:
    if d.exists():handles.append(os.add_dll_directory(str(d)))
sys.path.insert(0,str(next(isaac.glob('extscache/omni.usd.libs-*'))))
from pxr import Usd,UsdGeom,UsdPhysics,Sdf
source=root/'modules/wheeled_description/wheeled_leg_v2/urdf/wheeled_leg_v2.usd'
stage=Usd.Stage.Open(str(source))
for _ in range(5):
    inst=[p for p in stage.Traverse() if p.IsInstance()]
    if not inst:break
    for p in inst:p.SetInstanceable(False)
stage=Usd.Stage.Open(stage.Flatten())
remove=[]
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Joint) or 'sensor' in p.GetTypeName().lower() or p.GetName().lower()=='sensors':
        remove.append(p.GetPath());continue
    kept=[schema for schema in p.GetAppliedSchemas() if not any(s in schema.lower() for s in ['physics','physx','sensor'])]
    p.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(kept))
    for attr in list(p.GetAttributes()):
        if attr.GetName().lower().startswith(('physics:','physx','sensor:')):p.RemoveProperty(attr.GetName())
for path in reversed(remove):stage.RemovePrim(path)
dest=root/'research/scene_audit/code_as_room/robot_visual.usdc'
dest.parent.mkdir(parents=True,exist_ok=True)
stage.Export(str(dest))
print('VISUAL_ONLY',dest,'meshes',sum(p.IsA(UsdGeom.Mesh) for p in stage.Traverse()),'schemas',[str(p.GetPath()) for p in stage.Traverse() if any('phys' in s.lower() for s in p.GetAppliedSchemas())])
