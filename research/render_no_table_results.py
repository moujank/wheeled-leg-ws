"""Plot saved Isaac/ROS evidence for the table-removal experiment."""
from pathlib import Path
import json,sys,math,argparse,io
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle,Polygon,Patch
from matplotlib.lines import Line2D
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT/'modules/wheeled_navigation'
sys.path.insert(0,str(NAV))
from scan_nav.core import OccupancyMapper
from scan_nav.footprint import ROBOT_POLYGON
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--run',default='autonomy_ros_no_table_v1')
a=p.parse_args()
OUT=NAV/'artifacts'/a.run
FONT='C:/Windows/Fonts/msyh.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=FONT).get_name(),
    'axes.unicode_minus':False,'figure.facecolor':'#f1f5f8','axes.facecolor':'#acbbc9',
    'text.color':'#19364c','axes.labelcolor':'#526b7d','font.size':11})
COLORS=['#acbbc9','#f5f8f9','#233f54'];CMAP=ListedColormap(COLORS)

def load(path):
    report=json.loads((path/'evaluation.json').read_text())
    motion=json.loads((path/'motion.json').read_text())
    return {'path':path,'report':report,'status':report['execution'],
        'truth':np.asarray([q['truth'] for q in motion]),'times':np.asarray([q['seconds'] for q in motion]),
        'slam':OccupancyMapper.load(path/'ros/slam_map'),
        'measured':OccupancyMapper.load(path/'measured_baseline')}

controller=json.loads((OUT/'run_config.json').read_text())['arguments']['controller']
old=load(NAV/'artifacts'/('autonomy_ros_open_passages_v2' if controller=='rpp' else 'autonomy_ros_open_passages_v1'))
new=load(OUT)
def map_panel(ax,d,footprints=False):
    m=d['slam'];s=m.spec
    data=np.zeros(m.observed.shape,dtype=np.uint8);data[m.observed]=1;data[m.occupied()]=2
    ax.imshow(data,origin='lower',extent=[s.xmin,s.xmin+s.width*s.resolution,s.ymin,s.ymin+s.height*s.resolution],
              cmap=CMAP,vmin=0,vmax=2,interpolation='nearest')
    # These wall outlines are read only for offline illustration.
    for x,y,h in [(-2.625,-3.65,3.05),(-2.625,.6,3.05),(1.475,-3.65,1.45),(1.475,-1.1,4.75)]:
        ax.add_patch(Rectangle((x,y),.15,h,fill=False,ec='#bb955e',ls='--',lw=.8))
    q=d['truth'];ax.plot(q[:,0],q[:,1],c='white',lw=3.8)
    ax.plot(q[:,0],q[:,1],c='#099a82',lw=1.9)
    ax.scatter(*q[0,:2],c='#287bd1',s=46,zorder=10,edgecolors='white')
    ax.scatter(*q[-1,:2],facecolors='none',edgecolors='#e0a240',marker='s',s=85,lw=1.8,zorder=11)
    if footprints:
        walked=0.;last=-1.
        for i,pose in enumerate(q):
            if i:walked+=np.linalg.norm(q[i,:2]-q[i-1,:2])
            if walked-last<2.:continue
            last=walked;cs,sn=np.cos(pose[2]),np.sin(pose[2])
            poly=np.asarray(ROBOT_POLYGON)@np.array([[cs,sn],[-sn,cs]])+pose[:2]
            ax.add_patch(Polygon(poly,fill=False,ec='#098973',lw=.7,alpha=.7))
    ax.set(xlim=(-4.5,4.5),ylim=(-3.65,3.65),xlabel='x / 米',ylabel='y / 米')
    ax.set_aspect('equal')
    for spine in ax.spines.values():spine.set_visible(False)

q=new['truth'];lounge=(q[:,0]<-2.625)&(np.abs(q[:,1])<3.65)
fully_inside=[]
for pose in q:
    cs,sn=np.cos(pose[2]),np.sin(pose[2])
    poly=np.asarray(ROBOT_POLYGON)@np.array([[cs,sn],[-sn,cs]])+pose[:2]
    fully_inside.append(bool(poly[:,0].max()<-2.625))
summary={'run':a.run,'comparison_run':old['path'].name,
    'entered_west_lounge':bool(lounge.any()),
    'entry_definition':'Base centre crosses the west face of the left partition at x=-2.625 m.',
    'full_body_cleared_west_face':any(fully_inside),
    'first_lounge_entry_seconds':float(new['times'][np.flatnonzero(lounge)[0]]) if lounge.any() else None,
    'leftmost_executed_x_m':float(q[:,0].min()),
    'observed_floor_projection_m2':new['report']['observed_floor_projection_m2'],
    'observed_floor_projection_fraction':new['report']['observed_floor_projection_fraction'],
    'net_observed_area_change_m2':new['report']['observed_floor_projection_m2']-old['report']['observed_floor_projection_m2'],
    'controller':controller,
    'comparison_limit':'Same floor and controller configuration; separate online runs, not deterministic paired trials.',
    'static_preview':'Isaac native rendering of the modified scene; not a frame captured during autonomous motion.'}
summary['regional_observed_projection_m2']={}
for label,xlo,xhi in [('west_lounge',-4.5,-2.625),('bedroom',-2.625,1.475),('east_room',1.475,4.5)]:
    values={}
    for key,d in [('before',old),('after',new)]:
        m=d['measured'];s=m.spec
        xx=s.xmin+(np.arange(s.width)+.5)*s.resolution
        yy=s.ymin+(np.arange(s.height)+.5)*s.resolution
        mask=(xx[None,:]>=xlo)&(xx[None,:]<xhi)&(np.abs(yy[:,None])<3.65)
        values[key]=float((mask&m.observed).sum()*s.resolution**2)
    summary['regional_observed_projection_m2'][label]=values
(OUT/'scene_change_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
fig=plt.figure(figsize=(18,11.5))
fig.text(.04,.95,'移除入口小桌后，机器人实际探索到了哪里？',fontsize=25,fontweight='bold')
fig.text(.04,.91,'Code-as-Room  ·  从空白地图开始  ·  实时雷达 → SLAM Toolbox → 在线目标 → Nav2 速度指令',fontsize=12,color='#597184')
for i,(d,title) in enumerate([(old,'移除前：床头 / 床尾已加宽'),(new,'移除后：左侧入口小桌及物品已移除')]):
    left=.04+i*.325;r=d['report']
    fig.text(left,.857,title,fontsize=12.5,fontweight='bold')
    fig.text(left,.802,f'{r["observed_floor_projection_fraction"]:.1%}',fontsize=29,color='#098973',fontweight='bold')
    fig.text(left+.12,.81,f'{r["observed_floor_projection_m2"]:.2f} / {r["floor_area_m2"]:.2f} m²',fontsize=10.5)
    ax=fig.add_axes([left,.345,.30,.42]);map_panel(ax,d,footprints=bool(i))
    fig.text(left,.305,f'行驶 {d["status"]["distance_m"]:.2f} 米  ·  仿真 {d["status"]["sim_seconds"]:.1f} 秒',fontsize=10.5)
    fig.text(left,.276,'已返回起点' if d['status']['phase']=='complete' else '结束状态：'+d['status']['phase'],fontsize=10.5,color='#597184')
scene=plt.imread(ROOT/'research/scene_audit/code_as_room_open_passages_no_table/isaac_topdown.png')
ax=fig.add_axes([.715,.59,.25,.25]);ax.imshow(scene);ax.axis('off')
fig.text(.715,.567,'新场景 · Isaac Sim 原生静态预览',fontsize=10.5,color='#597184')
entry='底座中心已越过左侧入口' if summary['entered_west_lounge'] else '本轮尚未实际驶入左侧休息区'
if summary['full_body_cleared_west_face']:entry='完整身体已穿过左侧入口'
fig.text(.715,.51,entry,fontsize=12.5,fontweight='bold',color='#098973')
stats=[f'床头净间隙  1.21 米',f'床尾净间隙  1.27 米',
    f'导航目标  {new["status"]["goals_reached"]} / {new["status"]["goals_selected"]} 成功',
    f'完整轮廓 / 障碍 AABB 重叠  {new["report"]["polygon_aabb_overlap_samples"]} 个采样',
    f'观测面积变化  {summary["net_observed_area_change_m2"]:+.2f} 平方米']
fig.text(.715,.469,'\n'.join(stats),fontsize=11,linespacing=1.9,va='top')
legend=[Patch(fc=COLORS[1],label='已观测区域'),Patch(fc=COLORS[0],label='未知区域'),Patch(fc=COLORS[2],label='测量到的障碍'),
    Line2D([0],[0],c='#099a82',lw=2,label='实际执行轨迹'),Line2D([0],[0],c='#bb955e',ls='--',label='隔墙参考（离线）')]
fig.legend(handles=legend,loc='center',bbox_to_anchor=(.5,.212),ncol=5,frameon=False)
fig.text(.04,.15,'底图来自实际 SLAM 输出；轨迹与矩形身体轮廓仅在事后叠加，未将场景几何交给探索器。',fontsize=11,color='#597184')
fig.text(.04,.111,'覆盖率统计房间 65.7 平方米范围内的已观测 XY 投影，包含测到的障碍格，不等于三维重建完成率。',fontsize=10.5,color='#597184')
fig.text(.04,.072,'运动采用导航线速度 / 角速度积分，未调用 RL checkpoint；轮廓检查不等于动力学接触验证。',fontsize=10.5,color='#597184')
fig.text(.04,.035,f'两轮使用同一 {controller.upper()} 跟随器与场地范围；在线通信与目标选择具有时序差异，面积变化不能视为严格单因素结论。',fontsize=10.5,color='#597184')
fig.savefig(OUT/'no_table_comparison.png',dpi=170);plt.close(fig)

# The animation replays saved, truth-aligned measured-grid snapshots, not
# a synthesized SLAM animation or a predetermined trajectory.
with np.load(OUT/'map_progress.npz') as h:
    ts=h['times'];obs=h['observed'];odds=h['log_odds']
m=new['measured'];s=m.spec
if len(ts) and ts[-1]<new['times'][-1]:
    ts=np.r_[ts,new['times'][-1]]
    obs=np.concatenate([obs,m.observed[None]])
    odds=np.concatenate([odds,m.log_odds[None]])
xx=s.xmin+(np.arange(s.width)+.5)*s.resolution
yy=s.ymin+(np.arange(s.height)+.5)*s.resolution
floor=(abs(xx)[None,:]<4.5)&(abs(yy)[:,None]<3.65)
frames=[]
for k,t in enumerate(ts):
    fig,ax=plt.subplots(figsize=(9,8.5),dpi=110)
    data=np.zeros(obs[k].shape,dtype=np.uint8);data[obs[k]]=1;data[obs[k]&(odds[k]>.65)]=2
    ax.imshow(data,origin='lower',extent=[s.xmin,s.xmin+s.width*s.resolution,s.ymin,s.ymin+s.height*s.resolution],cmap=CMAP,vmin=0,vmax=2)
    selected=new['times']<=t;track=q[selected]
    if len(track):
        ax.plot(track[:,0],track[:,1],c='#099a82',lw=2)
        pose=track[-1];cs,sn=np.cos(pose[2]),np.sin(pose[2])
        ax.add_patch(Polygon(np.asarray(ROBOT_POLYGON)@np.array([[cs,sn],[-sn,cs]])+pose[:2],fc='#18b89e',ec='white',lw=1.3))
    area=(obs[k]&floor).sum()*s.resolution**2
    ax.set(xlim=(-4.5,4.5),ylim=(-3.65,3.65),xlabel='x / 米',ylabel='y / 米')
    ax.set_aspect('equal');ax.set_title(f'移除小桌后的实际扫描回放\nt = {t:.0f} 秒   |   房间内观测投影 {area:.2f} m²',fontsize=15,pad=15)
    fig.text(.5,.025,'已保存的真实扫描评价栅格 + 实际轨迹；机器人运动采用速度积分',ha='center',fontsize=9,color='#597184')
    fig.tight_layout(rect=(0,.04,1,1));buffer=io.BytesIO();fig.savefig(buffer,format='png');plt.close(fig)
    buffer.seek(0);frames.append(Image.open(buffer).convert('RGB').copy())
if frames:
    frames[0].save(OUT/'mapping_replay.gif',save_all=True,append_images=frames[1:],duration=[450]*(len(frames)-1)+[2000],loop=0)
print(json.dumps(summary,ensure_ascii=False,indent=2))
