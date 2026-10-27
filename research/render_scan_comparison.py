"""Render actual completed exploration runs; never synthesize scans or routes.

Published SLAM maps are shown on a common canvas. Statistics use the same
truth-aligned 5 cm evaluation grid. Scene reference walls are offline annotations.
"""
from pathlib import Path
import json,math,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle,Polygon,Circle,Patch
from matplotlib.lines import Line2D

ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT/'modules/wheeled_navigation'
sys.path.insert(0,str(NAV))
from scan_nav.core import OccupancyMapper
from scan_nav.footprint import ROBOT_POLYGON
OUT=NAV/'artifacts/autonomy_scan_comparison'
OUT.mkdir(parents=True,exist_ok=True)
font='C:/Windows/Fonts/msyh.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
    'axes.unicode_minus':False,'figure.facecolor':'#f0f4f8','axes.facecolor':'#a4b4c5',
    'text.color':'#19364c','axes.labelcolor':'#476073','xtick.color':'#62788a',
    'ytick.color':'#62788a','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
COLORS=['#a4b4c5','#f3f7f8','#24445b']
CMAP=ListedColormap(COLORS)
RUNS=[
    ('autonomy_ros_code_room_v2','原来的圆形轮廓','#d76c58','中间通道反复往返\n未进入右侧房间'),
    ('autonomy_ros_polygon_v5','矩形轮廓 · 覆盖较好的一轮','#0d9b80','已进入右侧区域并返回\n左侧仍未充分探索'),
    ('autonomy_ros_polygon_v12','矩形轮廓 · 后续调参的一轮','#5371bd','完成部分目标并返回\n覆盖面积未超过前一轮')]

def load_run(item):
    name,title,color,note=item;path=NAV/'artifacts'/name
    report=json.loads((path/'evaluation.json').read_text(encoding='utf-8'))
    status=json.loads((path/'status.json').read_text(encoding='utf-8'))
    motion=json.loads((path/'motion.json').read_text(encoding='utf-8'))
    truth=np.array([m['truth'] for m in motion]);times=np.array([m['seconds'] for m in motion])
    measured=OccupancyMapper.load(path/'measured_baseline')
    s=measured.spec
    xx=s.xmin+(np.arange(s.width)+.5)*s.resolution
    yy=s.ymin+(np.arange(s.height)+.5)*s.resolution
    floor=(abs(xx)[None,:]<4.5)&(abs(yy)[:,None]<3.)
    area=float((floor&measured.observed).sum()*s.resolution**2)
    assert abs(area-report['observed_floor_projection_m2'])<1e-6
    return dict(name=name,title=title,color=color,note=note,path=path,report=report,status=status,
                truth=truth,times=times,measured=measured,floor=floor,area=area,
                slam=OccupancyMapper.load(path/'ros/slam_map'))

data=[load_run(r) for r in RUNS]

def canvas(mapper,res=.025):
    # Outside a published map's extent MUST remain unknown, not white/free.
    x=-4.5+(np.arange(round(9/res))+.5)*res
    y=-3+(np.arange(round(6/res))+.5)*res
    c=np.floor((x-mapper.spec.xmin)/mapper.spec.resolution).astype(int)
    r=np.floor((y-mapper.spec.ymin)/mapper.spec.resolution).astype(int)
    inside=(r[:,None]>=0)&(r[:,None]<mapper.spec.height)&(c[None,:]>=0)&(c[None,:]<mapper.spec.width)
    rr=np.clip(r,0,mapper.spec.height-1);cc=np.clip(c,0,mapper.spec.width-1)
    obs=mapper.observed[np.ix_(rr,cc)]&inside
    occ=mapper.occupied()[np.ix_(rr,cc)]&inside
    image=np.zeros(inside.shape,dtype=np.uint8);image[obs]=1;image[occ]=2
    return image

def references(ax):
    ax.add_patch(Rectangle((-4.5,-3),9,6,fill=False,ec='#526a7b',lw=1,zorder=4))
    for x,y,h in [(-2.625,-3,2.4),(-2.625,.6,2.4),(1.475,-3,.8),(1.475,-1.1,4.1)]:
        ax.add_patch(Rectangle((x,y),.15,h,fill=False,ec='#b18749',lw=.9,ls=(0,(3,3)),zorder=4,alpha=.9))

def setup_map(ax,mapper,labels=True):
    ax.imshow(canvas(mapper),origin='lower',extent=(-4.5,4.5,-3,3),cmap=CMAP,vmin=0,vmax=2,interpolation='nearest')
    references(ax)
    ax.set(xlim=(-4.58,4.58),ylim=(-3.08,3.08),xticks=[-4,-2,0,2,4],yticks=[-2,0,2])
    ax.set_aspect('equal');ax.tick_params(labelsize=9,length=2)
    for spine in ax.spines.values():spine.set_visible(False)
    if labels:
        for x,text in [(-3.55,'左侧休息区'),(-.5,'中间卧室'),(3.05,'右侧衣帽间')]:
            ax.text(x,2.77,text,ha='center',va='center',fontsize=9,color='#3b5365',zorder=8,
                bbox=dict(boxstyle='round,pad=.23',fc='white',ec='none',alpha=.82))

def track(ax,d,footprints=False):
    q=d['truth'];ax.plot(q[:,0],q[:,1],color='white',lw=3.5,alpha=.9,zorder=5)
    ax.plot(q[:,0],q[:,1],color=d['color'],lw=1.9,zorder=6)
    ax.scatter(q[0,0],q[0,1],s=38,c='#2878c5',edgecolors='white',lw=.8,zorder=10)
    ax.scatter(q[-1,0],q[-1,1],s=65,marker='s',facecolors='none',edgecolors='#e6a53e',lw=1.8,zorder=11)
    if footprints:
        last=-2.;distance=0.
        for i,pose in enumerate(q):
            if i:distance+=math.dist(q[i-1,:2],pose[:2])
            if distance-last<1.6:continue
            last=distance;cs,sn=np.cos(pose[2]),np.sin(pose[2]);rot=np.array([[cs,-sn],[sn,cs]])
            poly=np.asarray(ROBOT_POLYGON)@rot.T+pose[:2]
            ax.add_patch(Polygon(poly,fill=False,ec=d['color'],lw=.9,alpha=.65,zorder=7))

def legend(fig,y):
    handles=[Patch(fc=COLORS[1],ec='#c6d1db',label='已观测区域'),Patch(fc=COLORS[0],label='未知 / 未输出地图'),
        Patch(fc=COLORS[2],label='测量到的障碍'),Line2D([0],[0],c='#0d9b80',lw=2,label='实际执行轨迹'),
        Line2D([0],[0],c='#b18749',ls='--',label='隔墙参考轮廓（离线叠加）')]
    fig.legend(handles=handles,loc='center',bbox_to_anchor=(.5,y),ncol=5,frameon=False,fontsize=10.5,
               handlelength=1.7,columnspacing=1.6)

fig=plt.figure(figsize=(18,10.2))
fig.text(.045,.947,'改成矩形轮廓之后，实际扫描到了哪里？',fontsize=24,fontweight='bold')
fig.text(.045,.902,'Code-as-Room 同一场景  /  实际 SLAM 地图与执行轨迹  /  三轮结果按同一范围展示',fontsize=12,color='#597184')
for i,d in enumerate(data):
    left=.045+i*.318
    fig.text(left,.841,f'0{i+1}  {d["title"]}',fontsize=13,fontweight='bold',color=d['color'])
    fig.text(left,.776,f'{d["area"]/54:.1%}',fontsize=30,fontweight='bold',color=d['color'])
    fig.text(left+.115,.788,f'{d["area"]:.2f} / 54.00 m²',fontsize=11,color='#597184')
    ax=fig.add_axes([left,.365,.29,.365]);setup_map(ax,d['slam']);track(ax,d)
    fig.text(left,.319,d['note'],fontsize=11,linespacing=1.6,va='top')
    fig.text(left,.237,f'{d["status"]["sim_seconds"]:.1f} 秒仿真  ·  实际行驶 {d["status"]["distance_m"]:.2f} 米',fontsize=10.5,color='#597184')
    fig.text(left,.209,d['name'],fontsize=8.5,color='#7a8f9f')
legend(fig,.17)
delta=data[1]['area']-data[0]['area']
fig.text(.045,.112,f'矩形轮廓已让机器人进入右侧；整间房观测面积净增 {delta:.2f} m²，约 {delta/54*100:.1f} 个百分点。',
         fontsize=13,fontweight='bold',color='#267c6b')
fig.text(.045,.066,'覆盖统计：同一张 5 厘米评价栅格，限于 9 × 6 米房间内的已观测 XY 投影；不表示完整三维表面重建。',fontsize=10,color='#597184')
fig.text(.045,.035,'各轮的扫描采样、控制参数和目标选择也有调整，不能把差异全部归因于轮廓；运动为速度积分，未接入 RL 策略。',fontsize=10,color='#597184')
fig.savefig(OUT/'scan_comparison.png',dpi=180);plt.close(fig)

best=data[1]
fig=plt.figure(figsize=(16,10.5))
fig.text(.045,.946,'矩形轮廓探索 · 扫描结果详图',fontsize=24,fontweight='bold')
fig.text(.045,.903,'覆盖较好的一轮：真实 SLAM Toolbox 输出地图 + 实际运动轨迹 + 雷达回波示例',fontsize=12,color='#597184')
ax=fig.add_axes([.045,.235,.635,.62]);setup_map(ax,best['slam']);track(ax,best,footprints=True)
ax.set_xlabel('x / 米',fontsize=10);ax.set_ylabel('y / 米',fontsize=10)
with np.load(best['path']/'sensor_history.npz') as history:
    poses=history['truth'];angles=history['angles'];times=history['times'];ranges=history['ranges']
    eligible=np.where(poses[:,0]>1.8)[0]
    index=eligible[np.argmax(poses[eligible,1])]
    pose=poses[index];scan=ranges[index]
    mount=pose[:2]+.16*np.array([math.cos(pose[2]),math.sin(pose[2])])
    for k in range(0,len(angles),24):
        if not .15<=scan[k]<7.99:continue
        end=mount+scan[k]*np.array([math.cos(angles[k]+pose[2]),math.sin(angles[k]+pose[2])])
        ax.plot([mount[0],end[0]],[mount[1],end[1]],color='#17aaa0',alpha=.25,lw=.8,zorder=3)
        ax.scatter(*end,s=3,c='#17aaa0',alpha=.6,zorder=4)
    ax.scatter(*mount,s=45,marker='*',c='#147b75',edgecolors='white',lw=.7,zorder=12)
    scan_time=float(times[index])
ax.text(-3.55,.55,'左侧仍有大块未知\n机器人尚未进入',ha='center',va='center',fontsize=11,color='#38536a',
        bbox=dict(boxstyle='round,pad=.5',fc='white',ec='none',alpha=.9))
ax.annotate('已实际进入右侧',xy=(2.48,.72),xytext=(2.65,1.8),fontsize=10,color='#0c7865',
            arrowprops=dict(arrowstyle='->',lw=1.2,color='#0c7865'),ha='center',zorder=15,
            bbox=dict(boxstyle='round,pad=.3',fc='white',ec='none',alpha=.94))
fig.text(.045,.185,f'细矩形沿路线显示机器人的真实占地朝向；扇状细线为 t = {scan_time:.1f} 秒的一帧真实回波投影（稀疏显示）。',
         fontsize=10,color='#597184')

scene=plt.imread(NAV/'artifacts/code_as_room_video/native_lit_background.png')
axscene=fig.add_axes([.715,.637,.25,.224]);axscene.imshow(scene);axscene.axis('off')
fig.text(.715,.613,'同一 Code-as-Room 场景参考',fontsize=10.5,color='#597184')
fig.text(.715,.558,f'{best["area"]/54:.1%}',fontsize=30,fontweight='bold',color='#0d9b80')
fig.text(.715,.523,f'房间内观测投影 {best["area"]:.2f} m² / 54.00 m²',fontsize=11)
stats=[f'实际行驶  {best["status"]["distance_m"]:.2f} 米',f'仿真时长  {best["status"]["sim_seconds"]:.1f} 秒',
       f'导航目标  11 次 / 成功 5 次（含返回）','完整轮廓与场景 AABB 重叠  0 个采样']
fig.text(.715,.481,'\n'.join(stats),fontsize=11,va='top',linespacing=1.8)
axbody=fig.add_axes([.73,.205,.21,.18]);axbody.set_aspect('equal');axbody.axis('off')
axbody.add_patch(Circle((0,0),.38,fill=False,ec='#a8b7c6',ls='--',lw=1.5))
axbody.add_patch(Polygon(ROBOT_POLYGON,fc='#a6ded1',ec='#0d9b80',lw=1.7))
axbody.arrow(-.1,0,.2,0,width=.007,head_width=.06,head_length=.04,color='#08765f')
axbody.set(xlim=(-.55,.55),ylim=(-.43,.47))
fig.text(.715,.194,'矩形约 0.50 × 0.53 米 + 1 厘米余量',fontsize=10.5)
fig.text(.715,.165,'虚线圆为原先直径 0.76 米的轮廓',fontsize=10,color='#597184')
legend(fig,.118)
fig.text(.045,.075,'地图之外保留未知；两道内隔墙的虚线仅供读图定位，不曾作为探索器的已知场景输入。',fontsize=10,color='#597184')
fig.text(.045,.045,'扫描面积采用统一坐标下的 5 厘米评价图统计；底图为实际 SLAM 输出。回波和轨迹仅在事后按仿真坐标对齐展示。',fontsize=10,color='#597184')
fig.text(.045,.017,'该轮完成部分探索并返回，不表示全场景探索完成。轮廓/AABB 检查不等于真实刚体接触验证。',fontsize=10,color='#597184')
fig.savefig(OUT/'rectangle_scan_detail.png',dpi=180);plt.close(fig)

# Record gains and losses explicitly; a changed route can reveal one region
# while observing less of another. Net area alone does not show that distinction.
old,new=data[0]['measured'],data[1]['measured']
assert vars(old.spec)==vars(new.spec)
floor=data[0]['floor'];cell=old.spec.resolution**2
change={'newly_observed_vs_circle_m2':float((new.observed&~old.observed&floor).sum()*cell),
        'observed_only_in_circle_run_m2':float((old.observed&~new.observed&floor).sum()*cell),
        'net_observed_area_change_m2':delta}
summary={'runs':[{'id':d['name'],'area_m2':d['area'],'fraction':d['area']/54,
    'seconds':d['status']['sim_seconds'],'distance_m':d['status']['distance_m']} for d in data],
    'change':change,'scan_example_seconds':scan_time,
    'map_source':'Actual saved SLAM Toolbox occupancy grids, common display canvas; outside-map area is unknown.',
    'statistics_source':'Actual measured scans on the same truth-aligned 5 cm evaluation grid, restricted to the 9 x 6 m room.',
    'not_a_single_variable_ablation':True,'new_simulation_run_performed_for_this_figure':False,
    'scene_reference_image':'Existing Isaac rendering of the same static scene, not a new autonomous-run screenshot.'}
(OUT/'comparison.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
(OUT/'README.zh.md').write_text('''# 矩形轮廓扫描结果对照

这组图直接读取已完成的真实仿真实验数据，没有生成新的轨迹或虚构扫描。

- `scan_comparison.png`：原圆形、矩形覆盖较好的一轮、矩形后续调参的一轮。
- `rectangle_scan_detail.png`：矩形 v5 的实际 SLAM 输出、实际轨迹、沿途身体轮廓和一帧真实回波。
- `comparison.json`：图中指标和新观测/少观测区域面积。

地图显示来自 SLAM Toolbox，采用同一房间坐标范围；发布地图范围外仍画为未知。
面积在同一 5 厘米、仿真坐标对齐的评价栅格上计算，限于 9×6 米房间。
隔墙虚线和场景照片只作为离线读图参考，未作为规划器输入。
场景照片来自此前同一静态房间的 Isaac 渲染，不能解释为本轮机器人运动截图。

三个实验的时长、扫描采样和控制参数不同，不是只改变轮廓的单因素对照。
这些实验没有调用 RL checkpoint，运动执行仍为线速度/角速度积分。

复现：从工作区根目录运行 `D:/isaacsim/python.bat research/render_scan_comparison.py`。
''',encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
print(OUT)
