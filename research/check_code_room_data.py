import sys,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'modules/wheeled_navigation'))
from scan_nav.core import OccupancyMapper
from scan_nav.code_room import load_geometry,clearance,path_clearance
out=root/'modules/wheeled_navigation/artifacts/code_as_room_demo'
d=np.load(out/'scans.npz');r=d['ranges'];raw=d['raw_depth_uint16']
print('POSES',d['poses'].shape,d['poses'][::30]);print('RANGES',r.min(),r.max(),'hit fraction',np.mean(r<19.99),'raw',raw.min(),raw.max())
print('FIRST',r[0,::20]);print('LAST',r[-1,::20])
mapper=OccupancyMapper.load(out)
print('MAP','seen',mapper.observed.sum(),'free',mapper.free().sum(),'occupied',mapper.occupied().sum())
g=load_geometry(root/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json')
errors=[]
for pose,rr in zip(d['poses'],r):
    hit=(rr<19.99)&(rr>.15)
    ex=pose[0]+rr[hit]*np.cos(pose[2]+d['angles'][hit]);ey=pose[1]+rr[hit]*np.sin(pose[2]+d['angles'][hit])
    errors.extend(max(0,clearance(x,y,g)) for x,y in zip(ex[::6],ey[::6]))
print('ENDPOINT',np.percentile(errors,[50,95,99]),max(errors))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(14,9))
rgb=np.full((*mapper.observed.shape,3),.65);rgb[mapper.free()]=1;rgb[mapper.occupied()]=0
s=mapper.spec
ax.imshow(rgb,origin='lower',extent=[s.xmin,s.xmin+s.width*s.resolution,s.ymin,s.ymin+s.height*s.resolution])
ax.plot(*d['poses'][:,:2].T,color='blue',lw=1)
for radius in [.25,.22,.20,.18,.15]:
    for start,goal in [((.9,-1.4),(3.1,1.5)),((3.4,-1.25),(3.1,1.5)),((.9,-1.4),(.92,1.6))]:
        try:
            path=mapper.plan(start,goal,radius)
            print('PLAN',radius,start,goal,len(path),'gap',path_clearance(path,g))
            if radius==.20:ax.plot(*np.array(path).T,color='green')
        except Exception as e:print('FAIL',radius,start,goal,str(e))
fig.savefig(out/'debug_map.png')
