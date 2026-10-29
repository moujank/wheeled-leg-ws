"""Bake the URDF's exact visual meshes into the training nominal bent stance.

No policy/physics is required. Joint angles are read from the existing training
asset configuration with ast, and URDF forward kinematics places every link.
The original URDF, STL meshes and USDs are never edited.
"""
import ast
import fnmatch
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
ISAAC=Path(os.environ.get('ISAAC_PATH','D:/isaacsim'));handles=[]
for folder in [ISAAC/'kit',ISAAC/'kit/lib',*ISAAC.glob('extscache/omni.usd.libs-*/bin')]:
    if folder.exists():handles.append(os.add_dll_directory(str(folder)))
sys.path.insert(0,str(next(ISAAC.glob('extscache/omni.usd.libs-*'))))
from pxr import Usd,UsdGeom,UsdShade,Gf,Sdf

DESCRIPTION=ROOT/'modules/wheeled_description/wheeled_arm_v2'
URDF=DESCRIPTION/'urdf/wheeled_arm_v2.urdf'
CFG=ROOT/'modules/wheeled_train/wheeled_leg/source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/asset/wheeled_arm_v2.py'
OUT=ROOT/'research/scene_audit/code_as_room/wheeled_arm_v2_bent_visual.usdc'
OUT.parent.mkdir(parents=True,exist_ok=True)

def configured_joints():
    tree=ast.parse(CFG.read_text(encoding='utf-8'))
    folded={}
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='ARM_FOLDED_JOINT_POS' for t in node.targets):
            folded=ast.literal_eval(node.value)
    for node in ast.walk(tree):
        if isinstance(node,ast.keyword) and node.arg=='joint_pos':
            q={}
            for key,value in zip(node.value.keys,node.value.values):
                if key is None:
                    if not isinstance(value,ast.Name) or value.id!='ARM_FOLDED_JOINT_POS':raise ValueError('Unexpected joint configuration expression')
                    q.update(folded)
                else:q[ast.literal_eval(key)]=ast.literal_eval(value)
            return q
    raise ValueError('Nominal joint pose not found')

def origin(node):
    result=np.eye(4)
    if node is not None:
        result[:3,3]=np.fromstring(node.attrib.get('xyz','0 0 0'),sep=' ')
        result[:3,:3]=Rotation.from_euler('xyz',np.fromstring(node.attrib.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return result

def stl(path):
    with path.open('rb') as f:
        f.read(80);n=int(np.frombuffer(f.read(4),dtype='<u4')[0])
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        rows=np.frombuffer(f.read(),dtype=dtype,count=n)
    vertices,indices=np.unique(rows['vertices'].reshape(-1,3),axis=0,return_inverse=True)
    return vertices,indices.reshape(-1,3),rows['normal']

urdf=ET.parse(URDF).getroot();patterns=configured_joints()
joint_values={}
transforms={'base_link':np.eye(4)}
pending=list(urdf.findall('joint'))
while pending:
    progressed=False
    for joint in pending[:]:
        parent=joint.find('parent').attrib['link'];child=joint.find('child').attrib['link']
        if parent not in transforms:continue
        name=joint.attrib['name'];angle=0.
        for pattern,value in patterns.items():
            if fnmatch.fnmatch(name,pattern.replace('.*','*')):angle=float(value)
        joint_values[name]=angle
        axis=np.fromstring(joint.find('axis').attrib['xyz'],sep=' ')
        rotation=np.eye(4);rotation[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*angle).as_matrix()
        transforms[child]=transforms[parent]@origin(joint.find('origin'))@rotation
        pending.remove(joint);progressed=True
    if not progressed:raise ValueError('URDF is not a connected tree')

stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1)
robot=UsdGeom.Xform.Define(stage,'/Robot');stage.SetDefaultPrim(robot.GetPrim())
all_points=[];wheel_points=[];link_centres={};link_points={}
for link in urdf.findall('link'):
    name=link.attrib['name'];group=UsdGeom.Xform.Define(stage,'/Robot/'+name)
    # Points are placed in the base coordinate frame, keeping the hierarchy
    # simple for a pure visual overlay. Pose remains fully traceable to the URDF.
    visual=link.find('visual')
    file=Path(visual.find('geometry/mesh').attrib['filename']).name
    vertices,faces,normals=stl(DESCRIPTION/'meshes'/file)
    transform=transforms[name]@origin(visual.find('origin'))
    transformed=vertices@transform[:3,:3].T+transform[:3,3]
    link_points[name]=transformed;all_points.append(transformed)
    link_centres[name]=transforms[name][:3,3].tolist()
    if name in ['HubL','HubR']:wheel_points.append(transformed)
    mesh=UsdGeom.Mesh.Define(stage,str(group.GetPath())+'/Visual')
    mesh.CreatePointsAttr(transformed.astype(np.float32).tolist())
    mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
    mesh.CreateNormalsAttr((normals@transform[:3,:3].T).astype(np.float32).tolist());mesh.SetNormalsInterpolation('uniform')
    mesh.CreateSubdivisionSchemeAttr('none')
    rgba=np.fromstring(visual.find('material/color').attrib['rgba'],sep=' ')
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*map(float,rgba[:3]))])
    mesh.CreateExtentAttr([Gf.Vec3f(*map(float,transformed.min(axis=0))),Gf.Vec3f(*map(float,transformed.max(axis=0)))])

points=np.concatenate(all_points);wheels=np.concatenate(wheel_points)
ground_offset=-float(wheels[:,2].min())+.006
# Position the nominal base using wheel contact, not the whole robot's AABB.
robot.AddTranslateOp().Set(Gf.Vec3d(0,0,ground_offset))
stage.GetRootLayer().customLayerData={'sourceURDF':str(URDF),'poseSource':str(CFG),
    'jointValues':json.dumps(joint_values),'purpose':'Bent-leg static visual; no physics or policy'}
stage.Export(str(OUT))
for side in ['L','R']:
    hip=np.array(link_centres['Hip'+side]);knee=np.array(link_centres['Knee'+side]);hub=np.array(link_centres['Hub'+side])
    a,b=hip-knee,hub-knee
    angle=np.degrees(np.arccos(np.clip(a.dot(b)/np.linalg.norm(a)/np.linalg.norm(b),-1,1)))
    print('KNEE_INTERNAL_ANGLE',side,float(angle))
report=dict(source_urdf=str(URDF),pose_source=str(CFG),output=str(OUT),joint_angles_rad=joint_values,
    base_height_m=ground_offset,bounds_min=(points.min(axis=0)+[0,0,ground_offset]).tolist(),
    bounds_max=(points.max(axis=0)+[0,0,ground_offset]).tolist(),wheel_min_z_m=float(wheels[:,2].min()+ground_offset),
    link_centres_base_frame=link_centres,meshes=len(link_points),source_modified=False)
OUT.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
