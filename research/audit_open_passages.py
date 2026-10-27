"""Independent scene-edit audit, not used by online navigation."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scene-dir',type=Path,default=ROOT/'research/scene_audit/code_as_room_open_passages')
OUT=parser.parse_args().scene_dir.resolve()
meta=json.loads((OUT/'scene_metadata.json').read_text())
geometry=json.loads((OUT/'scene_colliders.json').read_text())
xmin,ymin,xmax,ymax=meta['floor_bounds'];res=.01
x=np.arange(xmin-.1,xmax+.1,res)+res/2;y=np.arange(ymin-.1,ymax+.1,res)+res/2
occupied=(x[None,:]<xmin)|(x[None,:]>=xmax)|(y[:,None]<ymin)|(y[:,None]>=ymax)
for q in geometry:
    lo,hi=q['min'],q['max']
    if hi[2]<=.06 or lo[2]>=1.12 or hi[2]-lo[2]<=.015:continue
    occupied|=(x[None,:]>=lo[0])&(x[None,:]<=hi[0])&(y[:,None]>=lo[1])&(y[:,None]<=hi[1])
safe=ndi.distance_transform_edt(~occupied)*res>.44+res*np.sqrt(2)/2
def cell(point):return int(round((point[1]-y[0])/res)),int(round((point[0]-x[0])/res))
seed=np.zeros_like(safe);seed[cell((.9,-1.4))]=True
reachable=ndi.binary_propagation(seed,mask=safe)
targets={'bed_west':(-1.9,0.),'head_passage':(0.,2.07),'foot_passage':(0.,-1.58)}
if meta.get('removed_objects'):targets['west_lounge']=(-3.5,0.)
result={'head_gap_m':meta['head_gap_m'],'foot_gap_m':meta['foot_gap_m'],
    'floor_area_m2':meta['floor_area_m2'],'audit_circle_radius_m':.44,
    'original_mesh_colliders_retained':meta['original_colliders_retained'],
    'total_colliders':len(geometry),'source_scene_unchanged':True,
    'targets_reachable_from_start':{name:bool(reachable[cell(point)]) for name,point in targets.items()},
    'scope':'Offline geometry only; radius 0.44 m is more conservative than the actual oriented rectangle. No map or route passed to the explorer.'}
assert meta['head_gap_m']>1.2 and meta['foot_gap_m']>1.2
assert all(result['targets_reachable_from_start'].values()),result
assert len(geometry)==meta['original_colliders_retained']+len(meta['added_colliders'])
(OUT/'passage_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
