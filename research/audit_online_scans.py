"""Offline geometry audit, never imported by the online decision processes."""
import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
out=Path(sys.argv[1])
d=np.load(out/'sensor_history.npz')
g=json.loads((root/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json').read_text())
g=[q for q in g if q['max'][2]>.06 and q['min'][2]<1.15 and q['max'][2]-q['min'][2]>.015]
lo=np.array([q['min'][:2] for q in g]);hi=np.array([q['max'][:2] for q in g])
for i in np.linspace(0,len(d['truth'])-1,6).astype(int):
    p=d['truth'][i];rr=d['ranges'][i];angles=d['angles']
    good=(rr>.15)&(rr<7.99)
    sensor=p[:2]+.16*np.array([np.cos(p[2]),np.sin(p[2])])
    pts=sensor+rr[good,None]*np.c_[np.cos(angles[good]+p[2]),np.sin(angles[good]+p[2])]
    distances=np.linalg.norm(np.maximum(np.maximum(lo[None]-pts[:,None],pts[:,None]-hi[None]),0),axis=2).min(axis=1)
    root_clear=np.linalg.norm(np.maximum(np.maximum(lo-p[:2],p[:2]-hi),0),axis=1)
    print(i,p,'endpoint_aabb_error',np.quantile(distances,[.5,.9,1]),'root_clear',root_clear.min(),g[int(root_clear.argmin())]['path'])
