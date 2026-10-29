"""Offline source-scene audit: compare circle vs actual rectangular footprint.

This evaluator is never imported by SLAM, exploration, planners or simulation.
The fixed-orientation rectangle test is a connectivity diagnostic, not a valid
nonholonomic route or a substitute for the online planner.
"""
import json
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
root=Path(__file__).resolve().parents[1]
g=json.loads((root/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json').read_text())
res=.01;xmin=-4.6;ymin=-3.1;w=920;h=620
x=xmin+(np.arange(w)+.5)*res;y=ymin+(np.arange(h)+.5)*res
occupied=(abs(x)[None,:]>=4.5)|(abs(y)[:,None]>=3.)
for q in g:
    lo,hi=q['min'],q['max']
    if hi[2]<=.06 or lo[2]>=1.12 or hi[2]-lo[2]<=.015:continue
    occupied|=(x[None]>=lo[0])&(x[None]<=hi[0])&(y[:,None]>=lo[1])&(y[:,None]<=hi[1])
circle=ndi.distance_transform_edt(~occupied)*res>.38+res*.707
# Symmetric XY box slightly larger than actual [-.232,.268] x [-.264,.264].
rectangle=~ndi.maximum_filter(occupied,size=(57,57),mode='constant',cval=1)
start=(int((-1.4-ymin)/res),int((.9-xmin)/res))
seed=np.zeros_like(occupied);seed[start]=True
results={}
for name,safe in [('circle_radius_038',circle),('rectangle_axis_aligned_057',rectangle)]:
    component=ndi.binary_propagation(seed,mask=safe)
    results[name]={'reachable_centre_area_m2':float(component.sum()*res**2),
                   'bedroom_west_reachable':bool(component[int((1.0-ymin)/res),int((-1.9-xmin)/res)]),
                   'dressing_room_reachable':bool(component[int((1.0-ymin)/res),int((3.0-xmin)/res)]),
                   'lounge_reachable':bool(component[int((0-ymin)/res),int((-3.5-xmin)/res)])}
    np.save(root/f'research/{name}_component.npy',component)
(root/'research/room_reachability_audit.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
