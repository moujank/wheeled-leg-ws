"""Post-run evidence and figures. Ground truth is read ONLY by this evaluator."""
from __future__ import annotations
import argparse,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scan_nav.core import OccupancyMapper
from scan_nav.footprint import ROBOT_POLYGON


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);a=p.parse_args()
    out=a.run.resolve();motion=json.loads((out/'motion.json').read_text())
    status=json.loads((out/'status.json').read_text())
    times=np.array([r['seconds'] for r in motion]);truth=np.array([r['truth'] for r in motion])
    odom=np.array([r['odom'] for r in motion]);commands=np.array([r['command'] for r in motion])
    baseline=OccupancyMapper.load(out/'measured_baseline')
    ros_result=json.loads((out/'ros/result.json').read_text()) if (out/'ros/result.json').exists() else None
    events=json.loads((out/('ros/events.json' if ros_result else 'events.json')).read_text())
    report={'execution':status,'ros':ros_result,'maximum_step_m':float(np.max(np.linalg.norm(np.diff(truth[:,:2],axis=0),axis=1))),
            'max_abs_vx':float(np.max(abs(commands[:,0]))),'max_abs_wz':float(np.max(abs(commands[:,1]))),
            'ground_truth_used_by_this_offline_evaluator_only':True}
    if status['scene']=='code-as-room':
        workspace=ROOT.parents[1]
        geometry_file=Path(status.get('scene_geometry_file',workspace/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json'))
        geometry=json.loads(geometry_file.read_text())
        geometry=[q for q in geometry if q['max'][2]>.06 and q['min'][2]<1.12 and q['max'][2]-q['min'][2]>.015]
        lo=np.array([q['min'][:2] for q in geometry]);hi=np.array([q['max'][:2] for q in geometry])
        delta=np.maximum(np.maximum(lo[None]-truth[:,None,:2],truth[:,None,:2]-hi[None]),0)
        gap=np.linalg.norm(delta,axis=2).min(axis=1)
        report['independent_aabb_clearance_m']=float(gap.min())
        report['circle_aabb_overlap_samples']=int((gap<.38).sum())
        report['audit_note']='Conservative XY circle vs full-height scene mesh AABBs; not a rigid-body contact simulation.'
        s=baseline.spec
        xx=s.xmin+(np.arange(s.width)+.5)*s.resolution;yy=s.ymin+(np.arange(s.height)+.5)*s.resolution
        floor_bounds=status.get('floor_bounds',[-4.5,-3.,4.5,3.])
        xmin,ymin,xmax,ymax=floor_bounds
        floor=(xx[None,:]>=xmin)&(xx[None,:]<xmax)&(yy[:,None]>=ymin)&(yy[:,None]<ymax)
        floor_area=(xmax-xmin)*(ymax-ymin)
        report['observed_floor_projection_m2']=float((baseline.observed&floor).sum()*s.resolution**2)
        report['observed_floor_projection_fraction']=report['observed_floor_projection_m2']/floor_area
        report['floor_bounds']=floor_bounds;report['floor_area_m2']=floor_area
        report['coverage_definition']='Observed XY cells within the actual room floor bounds, on the same 5 cm truth-aligned evaluation grid; not complete 3D surface reconstruction.'
        report['evaluated_scene_geometry']=str(geometry_file)
        if 'polygon' in status.get('navigation_footprint',{}):
            polygon=np.asarray(ROBOT_POLYGON);mid=(polygon.min(axis=0)+polygon.max(axis=0))/2
            half=(polygon.max(axis=0)-polygon.min(axis=0))/2
            gmid=(lo+hi)/2;ghalf=(hi-lo)/2
            overlaps=[];separations=[]
            for point in truth:
                cs,sn=math.cos(point[2]),math.sin(point[2]);rot=np.array([[cs,-sn],[sn,cs]])
                centre=point[:2]+rot@mid;delta=gmid-centre
                world_axes=np.abs(delta)-(ghalf+np.abs(rot)@half)
                local_axes=np.abs(delta@rot)-(half+ghalf@np.abs(rot))
                axes=np.c_[world_axes,local_axes]
                overlaps.append(bool(np.any(np.all(axes<=0,axis=1))))
                separations.append(float(np.max(axes,axis=1).min()))
            report['polygon_aabb_overlap_samples']=sum(overlaps)
            report['minimum_polygon_separating_axis_gap_m']=min(separations)
            report['audit_note']='Rectangle/AABB SAT uses the full unscaled robot envelope at every executed yaw. Circumcircle overlaps alone are not collisions for this non-circular robot. This is not a rigid-body contact simulation.'
    estimated=None;slam=None
    if ros_result and (out/'ros/estimates.json').exists():
        est=np.asarray(json.loads((out/'ros/estimates.json').read_text()))
        gt=np.c_[np.interp(est[:,0],times,truth[:,0]),np.interp(est[:,0],times,truth[:,1]),
                  np.interp(est[:,0],times,np.unwrap(truth[:,2]))]
        theta=gt[0,2]-est[0,3]
        rotation=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        estimated=est[:,1:3]@rotation.T
        estimated+=gt[0,:2]-estimated[0]
        noisy=np.c_[np.interp(est[:,0],times,odom[:,0]),np.interp(est[:,0],times,odom[:,1])]
        # Initial-frame alignment only; no trajectory-fitting that can hide drift.
        noisy+=gt[0,:2]-noisy[0]
        slam_error=np.linalg.norm(estimated-gt[:,:2],axis=1)
        odom_error=np.linalg.norm(noisy-gt[:,:2],axis=1)
        report['pose_evaluation']={'samples':len(est),'slam_position_rmse_m':float(np.sqrt(np.mean(slam_error**2))),
            'odometry_position_rmse_m':float(np.sqrt(np.mean(odom_error**2))),
            'slam_final_position_error_m':float(slam_error[-1]),'odometry_final_position_error_m':float(odom_error[-1]),
            'alignment':'One initial SE(2) transform only; live TF sampling includes communication latency.',
            'loop_closure_observed':'Not inferred from enabling the parameter; requires separate pose-graph evidence.'}
        slam=OccupancyMapper.load(out/'ros/slam_map')
    (out/'evaluation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    plt.rcParams.update({'font.family':'DejaVu Sans','figure.facecolor':'#f3f6fa',
                         'axes.facecolor':'white','axes.spines.top':False,'axes.spines.right':False,
                         'axes.labelcolor':'#314359','text.color':'#17334a','font.size':10})
    fig=plt.figure(figsize=(16,10),layout='constrained')
    gs=fig.add_gridspec(2,3,height_ratios=[1.55,1.])
    def map_panel(ax,m,title,track):
        pixels=np.full(m.observed.shape,.50);pixels[m.free()]=1.;pixels[m.occupied()]=0.
        s=m.spec;extent=[s.xmin,s.xmin+s.width*s.resolution,s.ymin,s.ymin+s.height*s.resolution]
        ax.imshow(pixels,origin='lower',extent=extent,cmap=ListedColormap(['#182d40','#a8b7c6','#edf3f7']),vmin=0,vmax=1)
        ax.plot(track[:,0],track[:,1],color='#10a889',lw=2,label='Executed trajectory')
        ax.scatter(track[0,0],track[0,1],s=65,color='#2874d0',marker='o',zorder=5,label='Start')
        ax.scatter(track[-1,0],track[-1,1],s=70,color='#dc6650',marker='s',zorder=5,label='End')
        goals=[e['goal'] for e in events if e['event'] in ('frontier_goal','goal_sent') and e.get('goal')]
        if goals:ax.scatter(*np.array(goals).T,s=25,c='#eda83c',marker='x',label='Online goals')
        if status['scene']=='code-as-room':ax.set_xlim(xmin-.2,xmax+.2);ax.set_ylim(ymin-.2,ymax+.2)
        else:ax.set_xlim(-5.2,5.2);ax.set_ylim(-4.2,4.2)
        ax.set_aspect('equal');ax.set_title(title,fontweight='bold');ax.set_xlabel('x / m');ax.set_ylabel('y / m')
    ax=fig.add_subplot(gs[0,:2]);map_panel(ax,slam or baseline,
        'SLAM Toolbox live map + executed path' if slam else 'Measured map + online exploration',
        estimated if estimated is not None else truth[:,:2]);ax.legend(loc='upper left',fontsize=8)
    ax2=fig.add_subplot(gs[0,2]);ax2.axis('off')
    lines=['ONLINE EXPLORATION','',f"Scene: {status['scene']}",f"Runtime: {status['mode']}",
        f"Final state: {status['phase']}",f"Distance: {status['distance_m']:.2f} m",
        f"Simulation: {status['sim_seconds']:.1f} s",
        (f"Full polygon + {status['navigation_footprint'].get('padding',0)*100:g} cm padding" if 'polygon' in status.get('navigation_footprint',{}) else 'Robot circle radius: 0.38 m'),'',
        'No preselected itinerary','No source geometry in planner','Velocity-driven kinematic motion',
        'Checkpoint / balancing: not used','']
    if ros_result:lines += [f"Nav2 goals: {ros_result['goals_sent']}",f"Nav2 successes: {ros_result['goals_reached']}",
                          f"SLAM map updates: {ros_result['map_updates']}",f"Nav2 path messages: {ros_result['path_messages']}"]
    if 'polygon_aabb_overlap_samples' in report:
        lines += ['',f"Polygon/AABB overlaps: {report['polygon_aabb_overlap_samples']}",
                  f"Observed XY area: {report['observed_floor_projection_m2']:.2f} m²",
                  f"Observed / room floor: {report['observed_floor_projection_fraction']:.1%}"]
    elif 'independent_aabb_clearance_m' in report:
        lines += ['',f"Independent clearance: {report['independent_aabb_clearance_m']:.3f} m",
                  f"Circle/AABB overlaps: {report['circle_aabb_overlap_samples']}"]
    ax2.text(.03,.98,'\n'.join(lines),va='top',fontsize=11,linespacing=1.55,
             bbox={'boxstyle':'round,pad=1','facecolor':'white','edgecolor':'#dce5ed'})
    ax3=fig.add_subplot(gs[1,0])
    if status['scene']=='code-as-room' and (out/'map_progress.npz').exists():
        progress=np.load(out/'map_progress.npz')
        known=(progress['observed']&floor[None]).sum(axis=(1,2))*baseline.spec.resolution**2
        area_times=np.r_[progress['times'],times[-1]]-times[0]
        known=np.r_[known,report['observed_floor_projection_m2']]
        area_title='Observed XY area within the room floor'
    else:
        known=np.array([r['known_cells'] for r in motion])*baseline.spec.resolution**2
        area_times=times-times[0];area_title='New observations during motion'
    ax3.plot(area_times,known,color='#10a889',lw=2);ax3.fill_between(area_times,known,alpha=.1,color='#10a889')
    ax3.set(title=area_title,xlabel='Simulation time / s',ylabel='Observed area / m²')
    ax4=fig.add_subplot(gs[1,1]);ax4.plot(times-times[0],commands[:,0],color='#2874d0',label='vx / m/s')
    ax4.plot(times-times[0],commands[:,1],color='#ee9f32',alpha=.8,label='wz / rad/s')
    ax4.set(title='Executed velocity commands',xlabel='Simulation time / s');ax4.legend(fontsize=8)
    ax5=fig.add_subplot(gs[1,2])
    if estimated is not None:
        ax5.plot(est[:,0]-times[0],odom_error,color='#ee9f32',label='Biased odometry')
        ax5.plot(est[:,0]-times[0],slam_error,color='#2874d0',label='SLAM estimate')
        ax5.set(title='Position error against simulator truth',xlabel='Simulation time / s',ylabel='Error / m');ax5.legend(fontsize=8)
    else:
        distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(truth[:,:2],axis=0),axis=1))]
        ax5.plot(times-times[0],distance,color='#2874d0');ax5.set(title='Actual travelled distance',xlabel='Simulation time / s',ylabel='Distance / m')
    fig.suptitle('Unknown room → measured map → frontier goals → live navigation',fontsize=19,fontweight='bold')
    fig.savefig(out/'autonomy_evidence.png',dpi=160);plt.close(fig)
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
