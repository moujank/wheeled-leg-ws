import argparse,json,math,sys
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules/wheeled_navigation'))
from scan_nav.footprint import ROBOT_POLYGON,PADDING
for name in ('global','local'):
    f=a.run/'ros'/f'{name}_costmap.npz'
    if not f.exists():continue
    q=np.load(f);grid=q['data'];res=float(q['resolution']);origin=q['origin'];pose=q['pose']
    cs,sn=np.cos(pose[2]),np.sin(pose[2]);rot=np.array([[cs,-sn],[sn,cs]])
    pp=np.array(ROBOT_POLYGON);mid=(pp.min(0)+pp.max(0))/2;half=(pp.max(0)-pp.min(0))/2+PADDING
    centre=pose[:2]+rot@mid;rr,cc=np.indices(grid.shape)
    positions=np.c_[origin[0]+(cc.ravel()+.5)*res,origin[1]+(rr.ravel()+.5)*res]
    delta=positions-centre
    hit=np.all(abs(delta)<=abs(rot)@half+res/2,axis=1)
    hit&=np.all(abs(delta@rot)<=half+res/2*abs(rot).sum(0),axis=1)
    cells=grid.ravel()[hit];bad=hit&(grid.ravel()==100)
    print(json.dumps({'name':name,'frame':str(q['frame']),'pose':pose.tolist(),
        'overlap_cell_values':dict(zip(*[v.tolist() for v in np.unique(cells,return_counts=True)])),
        'lethal_cell_world_positions':positions[bad].tolist()},indent=2))
