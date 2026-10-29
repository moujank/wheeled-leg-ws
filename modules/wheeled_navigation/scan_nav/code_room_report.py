"""Evidence-based presentation of the Code-as-Room Isaac scan experiment.

All scene pictures are saved Isaac viewport captures. Map graphics are drawn
from recorded sensor integration; no floorplan is painted into the map.
"""
from pathlib import Path
import base64
import html
import json
import math
import numpy as np
from PIL import Image,ImageDraw,ImageFont,ImageOps

BG='#eef2f4';INK='#132c3d';MUTED='#5a6d7b';TEAL='#00a783';BLUE='#387de4';GOLD='#df902c'
FONT=Path('C:/Windows/Fonts/msyh.ttc')
BOLD=Path('C:/Windows/Fonts/msyhbd.ttc')

def font(size,bold=False):
    return ImageFont.truetype(str(BOLD if bold and BOLD.exists() else FONT),size)

def text(draw,xy,value,size=30,fill=INK,bold=False):
    draw.text(xy,value,font=font(size,bold),fill=fill)

def card(im,box):
    ImageDraw.Draw(im).rounded_rectangle(box,radius=24,fill='white')

def fit(im,path,box):
    with Image.open(path) as source:
        if Path(path).name=='isaac_cutaway.png':source=source.crop((320,140,1650,1130))
        picture=ImageOps.contain(source.convert('RGB'),(box[2]-box[0],box[3]-box[1]),Image.Resampling.LANCZOS)
        im.paste(picture,(box[0]+(box[2]-box[0]-picture.width)//2,box[1]+(box[3]-box[1]-picture.height)//2))

def write_report(out):
    from .core import OccupancyMapper
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from matplotlib.lines import Line2D
    fp=FontProperties(fname=str(FONT))
    plt.rcParams.update({'axes.spines.top':False,'axes.spines.right':False,
                         'axes.labelcolor':MUTED,'text.color':INK,'xtick.color':MUTED,'ytick.color':MUTED})
    out=Path(out)
    m=json.loads((out/'metrics.json').read_text(encoding='utf-8'))
    mapper=OccupancyMapper.load(out)
    data=np.load(out/'scans.npz');poses=data['poses'];angles=data['angles']
    progress=np.load(out/'map_progress.npz')
    spec=mapper.spec
    extent=[spec.xmin,spec.xmin+spec.width*spec.resolution,spec.ymin,spec.ymin+spec.height*spec.resolution]
    def raster(odds,seen):
        rgb=np.full((*seen.shape,3),[207,216,223],dtype=np.uint8)
        rgb[seen&(odds<-.3)]=[248,249,245]
        rgb[seen&(odds>.65)]=[29,48,63]
        rgb[seen&(odds>=-.3)&(odds<=.65)]=[153,171,183]
        return rgb
    def map_plot(filename,idx=None,odds=None,seen=None,show_plans=True):
        fig,ax=plt.subplots(figsize=(12,8.1),facecolor='white')
        odds=mapper.log_odds if odds is None else odds
        seen=mapper.observed if seen is None else seen
        ax.imshow(raster(odds,seen),origin='lower',extent=extent,interpolation='nearest')
        n=len(poses) if idx is None else idx+1
        ax.plot(poses[:n,0],poses[:n,1],color=BLUE,lw=1.6,alpha=.75,dashes=(3,3))
        if show_plans:
            colors=[TEAL,GOLD,'#9869ba']
            for route,color in zip(m['planning_cases'],colors):
                xy=np.array(route['path']);ax.plot(*xy.T,color=color,lw=3.3,alpha=.95)
            for label,point,color in [('S',m['planning_cases'][0]['start'],BLUE),('G',m['planning_cases'][0]['goal'],GOLD)]:
                ax.scatter(*point,s=180,c=color,edgecolors='white',linewidths=2,zorder=5)
                ax.annotate(label,point,xytext=(11,7),textcoords='offset points',fontsize=13,weight='bold',color=color)
        if idx is not None:
            pose=poses[idx];r=data['ranges'][idx];hit=np.isfinite(r)&(r<19.99)&(r>.15)
            aa=angles[hit]+pose[2];r=r[hit]
            for angle,dist in zip(aa[::12],r[::12]):
                ax.plot([pose[0],pose[0]+dist*math.cos(angle)],[pose[1],pose[1]+dist*math.sin(angle)],color=TEAL,lw=.6,alpha=.35)
            ax.scatter(*pose[:2],s=80,c=TEAL,edgecolors='white',zorder=8)
        for label,x in [('休闲区',-3.5),('卧室',-.5),('衣帽间',2.9)]:
            ax.text(x,2.95,label,fontproperties=fp,fontsize=12,ha='center',va='top',color=INK,
                    bbox=dict(boxstyle='round,pad=.25',facecolor='white',alpha=.90,edgecolor='none'))
        ax.plot([-4.2,-3.2],[-2.95,-2.95],color=INK,lw=4);ax.text(-3.7,-2.78,'1 m',ha='center',fontsize=11)
        ax.annotate('N',xy=(4.55,2.7),xytext=(4.55,2.05),ha='center',arrowprops=dict(arrowstyle='->',color=INK,lw=1.5))
        ax.set(xlim=(-4.8,4.8),ylim=(-3.25,3.25),aspect='equal',xlabel='x / m',ylabel='y / m')
        ax.spines[['top','right','left','bottom']].set_visible(False)
        if show_plans:
            items=[Line2D([0],[0],color=BLUE,lw=2,dashes=(3,3),label='预设采样路线'),
                   *[Line2D([0],[0],color=c,lw=3,label=f"{i+1}. {r['name']}") for i,(r,c) in enumerate(zip(m['planning_cases'],colors))]]
            ax.legend(handles=items,prop=fp,loc='upper center',bbox_to_anchor=(.5,-.08),ncol=2,frameon=False)
        fig.subplots_adjust(left=.055,right=.985,top=.97,bottom=.14 if show_plans else .07)
        fig.savefig(out/filename,dpi=180,facecolor='white');plt.close(fig)
    map_plot('measured_map_and_paths.png')
    frames=[]
    for j,(idx,odds,seen) in enumerate(zip(progress['indices'],progress['log_odds'],progress['observed'])):
        filename=f'mapping_stage_{j+1}.png'
        map_plot(filename,int(idx),odds,seen,show_plans=False)
        image=Image.open(out/filename).convert('RGB');image.thumbnail((960,648))
        frame=Image.new('RGB',(1000,735),'white');frame.paste(image,((1000-image.width)//2,55))
        d=ImageDraw.Draw(frame);text(d,(28,16),f'真实扫描累积  {idx+1:03d} / {len(poses)} 帧',27,bold=True)
        text(d,(28,691),'深色：障碍物    浅色：观测到的空闲区域    灰蓝：未知',21,fill=MUTED)
        frames.append(frame)
    frames[0].save(out/'mapping_progress.gif',save_all=True,append_images=frames[1:],duration=[1000]*4+[2500],loop=0)

    # A large, readable overview, pairing actual rendered geometry with data.
    im=Image.new('RGB',(3000,2350),BG);d=ImageDraw.Draw(im)
    d.rectangle((0,0,3000,280),fill=INK)
    text(d,(90,48),'CODE-AS-ROOM  /  ISAAC SIM',32,fill='#65d7bd',bold=True)
    text(d,(90,102),'从三维房间到雷达地图与目标点路径',72,fill='white',bold=True)
    text(d,(94,207),'真实 PhysX 测距 · 预设位姿采样 · 仿真真值定位 · 仅在扫描地图上规划',31,fill='#c3d4df')
    card(im,(60,320,1900,1600));card(im,(1940,320,2940,1600))
    text(d,(100,350),'01  三维房间与路线预览',44,bold=True)
    text(d,(102,413),'Isaac Sim 原生视口截图；前墙和部分隔墙仅在截图时隐藏',26,fill=MUTED)
    fit(im,out/'isaac_cutaway.png',(85,465,1875,1535))
    text(d,(1980,350),'02  扫描地图与 A* 路径',38,bold=True)
    fit(im,out/'measured_map_and_paths.png',(1960,442,2920,1155))
    text(d,(1990,1180),'地图只接收扫描距离和传感器位姿',29,bold=True)
    text(d,(1990,1232),'未知区域保持未知，规划时禁止穿越。',27,fill=MUTED)
    text(d,(1990,1294),'休闲区入口受边桌影响，未完成进入验证。',25,fill=MUTED)
    text(d,(1990,1360),'蓝虚线  预设采样    绿线  跨房间路径',25,fill=BLUE)
    text(d,(1990,1405),'橙线  绕岛台路径    紫线  卧室东侧路径',25,fill=MUTED)
    text(d,(1990,1490),'S  起点      G  目标点',29,bold=True)
    card(im,(60,1640,1610,2230));card(im,(1650,1640,2940,2230))
    text(d,(100,1672),'03  原始房间俯视',37,bold=True)
    fit(im,out/'isaac_topdown.png',(80,1730,1000,2207))
    text(d,(1035,1755),'54 m²',53,bold=True)
    text(d,(1038,1820),'原场景地面面积',26,fill=MUTED)
    text(d,(1035,1887),'222',53,bold=True)
    text(d,(1038,1952),'房间静态网格碰撞体',26,fill=MUTED)
    text(d,(1035,2020),'3 个功能区',40,bold=True)
    text(d,(1038,2080),'休闲区 / 卧室 / 衣帽间',25,fill=MUTED)
    text(d,(1690,1672),'04  本次运行记录',37,bold=True)
    stats=[(f"{m['scan_count']} 帧",'逐点采样'),(f"{m['raw_ray_count']:,}",'原始测距射线'),
           (f"{m['resolution_m']*100:g} cm",'栅格分辨率'),(f"{m['free_area_m2']:.1f} m²",'观测空闲区域')]
    for i,(value,label) in enumerate(stats):
        x=1700+(i%2)*575;y=1758+(i//2)*157
        text(d,(x,y),value,50,fill=TEAL,bold=True);text(d,(x,y+67),label,28,fill=MUTED)
    text(d,(1700,2095),'3 条目标路径通过；4 类无效目标被拒绝',29,bold=True)
    text(d,(1700,2150),'多高度扫描：0.18 / 0.38 / 0.62 / 0.90 m',26,fill=MUTED)
    text(d,(80,2272),'演示条件：脚本直接设置位姿；使用理想定位。规划采用 0.25 m 虚拟圆盘，尚未验证整机外形、动力学与 RL 行走。',28,fill=MUTED)
    im.save(out/'code_as_room_preview.png')

    # Causal schematic with explicit information boundaries and no invented UI.
    im=Image.new('RGB',(2800,1880),BG);d=ImageDraw.Draw(im)
    text(d,(80,50),'建图与路径规划：这次实际运行了什么',65,bold=True)
    text(d,(84,145),'读图顺序：房间产生测距 → 用真值位姿累积扫描 → 从观测地图寻找可通行路径',31,fill=MUTED)
    boxes=[(70,255,680,855),(755,255,1365,855),(1440,255,2050,855),(2125,255,2735,855)]
    titles=['01  房间与位姿采样','02  原生雷达测距','03  扫描累积成图','04  地图上的 A*']
    bodies=[['加载 Code-as-Room 的 USD','保留房间与家具的碰撞网格','按预设路线直接写入根位姿','每次修改后等待 3 个物理步'],
            ['4 个高度 × 360 个方向','PhysX 射线与碰撞面求交','同方向取最近的有效回波','保存原始 uint16 距离数组'],
            ['使用仿真真值 x、y、yaw','射线经过处累积为空闲','回波终点累积为障碍物','没有被扫描的区域保持未知'],
            ['输入：扫描得到的占据地图','将未知与障碍物向外膨胀','按虚拟圆盘半径 0.25 m 规划','输出绕障碍物的目标点路径']]
    for i,(b,title,body) in enumerate(zip(boxes,titles,bodies)):
        card(im,b);d.rounded_rectangle((b[0]+24,b[1]+24,b[0]+112,b[1]+112),radius=22,fill=TEAL if i>0 else BLUE)
        text(d,(b[0]+47,b[1]+36),str(i+1),45,fill='white',bold=True)
        text(d,(b[0]+28,b[1]+145),title,36,bold=True)
        for j,s in enumerate(body):text(d,(b[0]+28,b[1]+230+j*62),s,28,fill=MUTED)
        if i<3:
            x=b[2]+12;y=550
            d.line((x,y,x+47,y),fill=TEAL,width=7)
            d.polygon([(x+57,y),(x+37,y-14),(x+37,y+14)],fill=TEAL)
    card(im,(70,915,1370,1615));card(im,(1425,915,2735,1615))
    text(d,(110,950),'多高度扫描为什么有帮助',39,bold=True)
    text(d,(112,1010),'床、桌面和椅子的高度不同；单一平面容易漏掉低矮家具。',27,fill=MUTED)
    # Side-view diagram, labelled as schematic rather than a rendered sensor.
    floor_y=1450
    d.line((130,floor_y,1295,floor_y),fill=INK,width=5)
    d.rounded_rectangle((225,1170,370,1390),radius=20,fill='#71899a')
    d.ellipse((217,1375,278,1450),fill=INK);d.ellipse((321,1375,382,1450),fill=INK)
    d.rectangle((645,1300,890,1447),fill='#cbbbaa')
    d.rectangle((1040,1185,1220,1210),fill='#cbbbaa')
    d.rectangle((1050,1210,1070,1447),fill='#cbbbaa');d.rectangle((1190,1210,1210,1447),fill='#cbbbaa')
    for z,y,xend in [(.18,1400,645),(.38,1350,645),(.62,1270,1050),(.90,1192,1040)]:
        d.line((375,y,xend,y),fill=TEAL,width=4)
        d.ellipse((xend-7,y-7,xend+7,y+7),fill=GOLD)
        text(d,(420,y-33),f'{z:.2f} m',22,fill=TEAL)
    text(d,(200,1480),'虚拟扫描载体',26);text(d,(720,1480),'低矮家具',26);text(d,(1070,1480),'桌面与桌腿',26)
    text(d,(112,1552),'示意图：本次使用四个虚拟平面，并非真实硬件的三维点云模型。',24,fill=MUTED)
    text(d,(1465,950),'信息从哪里来，验证到了哪里',39,bold=True)
    notes=[('场景几何','用于设计采样路线、生成真实雷达回波和事后检查。'),
           ('最终地图','由扫描距离与真值位姿生成；不直接填入房间平面图。'),
           ('目标规划','只读取扫描地图；未知区、床体、衣柜和房间外部不可通行。'),
           ('路径回放','仍然由脚本设置模型位姿；到达目标不代表行走策略可用。'),
           ('整机限制','采样盘半径 0.22 m、规划盘半径 0.25 m，均为演示参数。'),
           ('后续接入','拿到 checkpoint 后，再替换为高层速度指令与真实关节控制。')]
    for i,(label,value) in enumerate(notes):
        y=1035+i*86
        text(d,(1475,y),label,27,fill=TEAL,bold=True)
        text(d,(1475,y+37),value,25,fill=MUTED)
    d.rounded_rectangle((70,1670,2735,1805),radius=24,fill=INK)
    text(d,(110,1695),'已验证',34,fill='#65d7bd',bold=True)
    text(d,(310,1699),'扫描随位姿变化、地图从空白累积、3 条路径绕障碍物、4 类无效目标被拒绝。',33,fill='white')
    text(d,(110,1753),'使用理想定位；本次没有运行回环检测、SLAM Toolbox、Nav2 或机器人 RL 策略。',27,fill='#c3d4df')
    im.save(out/'mapping_method_diagram.png')

    def embed(name):
        mime='image/gif' if name.endswith('.gif') else 'image/png'
        return 'data:'+mime+';base64,'+base64.b64encode((out/name).read_bytes()).decode('ascii')
    cards=[]
    for title,name in [('总览','code_as_room_preview.png'),('方法与边界','mapping_method_diagram.png'),
                       ('建图过程','mapping_progress.gif'),('原始俯视截图','isaac_topdown.png'),
                       ('原始完整房间截图','isaac_full_room.png'),('扫描地图与三条路径','measured_map_and_paths.png')]:
        cards.append(f'<section><h2>{title}</h2><a href="{embed(name)}" target="_blank"><img src="{embed(name)}" alt="{title}"></a></section>')
    rows=''.join(f"<tr><td>{html.escape(r['name'])}</td><td>{r['length_m']:.2f} m</td><td>{r['min_clearance_m']:.3f} m</td></tr>" for r in m['planning_cases'])
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Code-as-Room · Isaac 雷达建图验证</title>
    <style>body{margin:0;background:#eef2f4;color:#132c3d;font:18px/1.8 "Microsoft YaHei",sans-serif}main{max-width:1400px;margin:auto;padding:40px}h1{font-size:42px}section{background:white;border-radius:20px;padding:24px;margin:28px 0}img{width:100%;height:auto}table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:12px;border-bottom:1px solid #ddd}small{color:#5a6d7b}</style><main>
    <h1>Code-as-Room 房间：扫描、建图与目标点规划</h1><p>本报告的三维图来自 Isaac Sim 视口；地图来自保存的 PhysX 测距。机器人采用脚本设置位姿，定位使用仿真真值。</p>'''+''.join(cards)+f'''<section><h2>路径独立检查</h2><table><tr><th>目标任务</th><th>路径长度</th><th>中心到场景 AABB 最小距离</th></tr>{rows}</table><p>所有路线从扫描地图规划，随后用原场景家具的包围盒独立检查。圆盘半径是演示参数，没有认证完整机器人外形。</p></section><section><h2>复查数据</h2><p>同目录保存 metrics.json、scans.npz、grid.npz、map.pgm、map.yaml、map_progress.npz、scripted_replay.json 和 navigation_scene.usda。scans.npz 保留原始 uint16 测距、角度、四个扫描高度及每帧位姿，可离线重新建图。</p><p>休闲区门口有家具挤占通道；本次没有移动源场景家具，也没有证明机器人可以进入该区域。四个扫描平面之间仍可能漏掉障碍物。</p></section></main></html>'''
    (out/'report.html').write_text(page,encoding='utf-8')

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('output',type=Path)
    write_report(p.parse_args().output)
