"""Offline explanation of the circled bed passages; never a planner input."""
import json,sys,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT/'modules/wheeled_navigation'
sys.path.insert(0,str(NAV))
from scan_nav.core import OccupancyMapper
from scan_nav.footprint import pose_is_free
g=json.loads((ROOT/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json').read_text())
g=[q for q in g if q['max'][2]>.06 and q['min'][2]<1.12 and q['max'][2]-q['min'][2]>.015]
groups={}
for q in g:
    name=q['path'].split('/')[2]
    if any(t in name for t in ('Queen_King','Bed_Bedding','Bed_Throw','Foot_Bed_Bench','Nightstand')):
        if name not in groups:groups[name]={'min':q['min'],'max':q['max']}
        else:
            groups[name]['min']=np.minimum(groups[name]['min'],q['min']).tolist()
            groups[name]['max']=np.maximum(groups[name]['max'],q['max']).tolist()
print('SOURCE_OBJECT_BOUNDS',json.dumps(groups,indent=2))
lo=np.array([q['min'][:2] for q in g]);hi=np.array([q['max'][:2] for q in g])
mid=(lo+hi)/2;half=(hi-lo)/2
def source_free(x,y,yaw=0):
    cs,sn=math.cos(yaw),math.sin(yaw);rot=np.array([[cs,-sn],[sn,cs]])
    center=np.array([x,y])+rot@np.array([(.268151-.231726)/2,(.264315-.264291)/2])
    rh=np.array([(.268151+.231726)/2+.01,(.264315+.264291)/2+.01])
    delta=mid-center
    axes=np.c_[abs(delta)-(half+abs(rot)@rh),abs(delta@rot)-(rh+half@abs(rot))]
    return not np.any(np.all(axes<=0,axis=1))

maps={}
for run in ('autonomy_ros_polygon_v5','autonomy_ros_polygon_v12'):
    maps[run+'_slam']=OccupancyMapper.load(NAV/'artifacts'/run/'ros/slam_map')
    maps[run+'_measured']=OccupancyMapper.load(NAV/'artifacts'/run/'measured_baseline')

report={}
for name,ymin,ymax in [('foot',-1.5,-1.05),('head',1.45,2.15)]:
    rows=[]
    for x in (.9,.6,.3,0.,-.4,-.8,-1.2,-1.7):
        values=np.arange(ymin,ymax+.001,.005)
        good=[float(y) for y in values if source_free(x,y)]
        row={'x':x,'source_free_y_range':None if not good else [round(min(good),3),round(max(good),3)]}
        for key,mapper in maps.items():
            valid=[float(y) for y in values if pose_is_free(mapper,(x,y),0.)]
            row[key]=None if not valid else [round(min(valid),3),round(max(valid),3)]
        rows.append(row)
    report[name]=rows
print('ALIGNED_BODY_VALID_CENTRE_Y',json.dumps(report,indent=2))

for run in ('autonomy_ros_polygon_v5','autonomy_ros_polygon_v12'):
    events=json.loads((NAV/'artifacts'/run/'ros/events.json').read_text())
    selected=[e for e in events if e.get('goal',[10])[0]<.7 and e['event'] in ('goal_sent','goal_result','heading_path_rejected')]
    print('LEFT_GOAL_EVENTS',run,json.dumps(selected,indent=2))
out={'source_bounds':groups,'aligned_body_valid_centre_y':report,
     'note':'Fixed yaw zero, actual full rectangle plus 1 cm padding. Final maps only; not the live map at each failed goal time.'}
# A counterexample for geometry diagnosis only: sample a differential-drive
# manoeuvre that turns in the open approach area, crosses the foot passage,
# and turns in the west aisle. NEVER feed this source-derived path to autonomy.
poses=[]
def rotate(x,y,aa,bb):
    poses.extend((x,y,float(t)) for t in np.linspace(aa,bb,round(abs(bb-aa)/.01)+1))
def line(aa,bb,yaw):
    poses.extend((float(x),float(y),yaw) for x,y in np.linspace(aa,bb,round(math.dist(aa,bb)/.005)+1))
rotate(.9,-1.4,0,math.pi/2)
line((.9,-1.4),(.9,-1.245),math.pi/2)
rotate(.9,-1.245,math.pi/2,math.pi)
line((.9,-1.245),(-1.9,-1.245),math.pi)
rotate(-1.9,-1.245,math.pi,math.pi/2)
line((-1.9,-1.245),(-1.9,1.),math.pi/2)
audit={'sampled_poses':len(poses),'source_collision_samples':sum(not source_free(*p) for p in poses),
       'model':'Full body rectangle + 1 cm padding vs body-height-filtered mesh AABBs; turns in place allowed.',
       'scope':'Offline geometry counterexample only, not autonomously planned or executed in Isaac.'}
out['source_only_geometry_counterexample']=audit
print('SOURCE_ONLY_GEOMETRY_COUNTEREXAMPLE',json.dumps(audit,indent=2))
(ROOT/'research/left_passage_diagnosis.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
