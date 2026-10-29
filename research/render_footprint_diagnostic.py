"""Offline geometry explanation only; not imported by any navigation process."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle,Rectangle,Patch

root=Path(__file__).resolve().parents[1]
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams.update({'font.family':font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name(),
    'axes.unicode_minus':False,'figure.facecolor':'#f1f5f9','axes.facecolor':'white',
    'text.color':'#19364c','axes.labelcolor':'#19364c','font.size':11,
    'axes.spines.top':False,'axes.spines.right':False})
fig=plt.figure(figsize=(15,10.5))
gs=fig.add_gridspec(2,2,height_ratios=[1,1.55],left=.065,right=.975,top=.86,bottom=.115,hspace=.39,wspace=.2)
for column in range(2):
    ax=fig.add_subplot(gs[0,column])
    for y in (.31,-.58):ax.add_patch(Rectangle((-.9,y),1.8,.27,color='#8493a3'))
    ax.axhline(.31,color='#566b7e',lw=1);ax.axhline(-.31,color='#566b7e',lw=1)
    if column==0:
        ax.add_patch(Circle((0,0),.38,fc='#df6e5c',ec='#a84437',alpha=.65,lw=2))
        ax.set_title('原来的圆形表示：直径 0.76 米',fontweight='bold',pad=14)
        ax.text(0,-.76,'圆形与上下障碍重叠 → 规划器拒绝',ha='center',color='#a84437')
    else:
        ax.add_patch(Rectangle((-.231726,-.264291),.499877,.528606,fc='#42bfa5',ec='#168d75',alpha=.8,lw=2))
        ax.add_patch(Rectangle((-.241726,-.274291),.519877,.548606,fill=False,ec='#168d75',ls='--',lw=1.4))
        ax.arrow(-.12,0,.25,0,width=.008,head_width=.065,head_length=.05,color='#155b50')
        ax.set_title('完整模型俯视包络：约 0.50 × 0.53 米',fontweight='bold',pad=14)
        ax.text(0,-.76,'朝向合适时有通过空间；虚线为额外 1 厘米余量',ha='center',color='#168d75')
    ax.annotate('',xy=(.7,.31),xytext=(.7,-.31),arrowprops={'arrowstyle':'<->','color':'#19364c','lw':1.5})
    ax.text(.73,0,'0.62 米',rotation=90,va='center')
    ax.set(xlim=(-.9,.9),ylim=(-.86,.6));ax.set_aspect('equal');ax.axis('off')

geometry=json.loads((root/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json').read_text())
for column,(name,title) in enumerate([
    ('circle_radius_038','原来的圆形：底座中心只能留在中部通道'),
    ('rectangle_axis_aligned_057','固定朝向方形诊断：多处通道重新连通')]):
    ax=fig.add_subplot(gs[1,column]);ax.set_facecolor('#edf2f6')
    data=np.load(root/f'research/{name}_component.npy')
    image=np.zeros((*data.shape,4));image[data]=[.12,.66,.55,.70]
    ax.imshow(image,origin='lower',extent=(-4.6,4.6,-3.1,3.1))
    for obj in geometry:
        lo,hi=obj['min'],obj['max']
        if hi[2]<=.06 or lo[2]>=1.12 or hi[2]-lo[2]<=.015:continue
        ax.add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],color='#536b7f',alpha=.85,lw=0))
    ax.scatter(.9,-1.4,marker='*',s=130,color='#f0b44e',edgecolor='#6e561e',zorder=5)
    ax.set_title(title,fontweight='bold',pad=12);ax.set(xlim=(-4.6,4.6),ylim=(-3.1,3.1),xlabel='x / 米',ylabel='y / 米')
    ax.set_aspect('equal')
    area=float(data.sum())*.01**2
    ax.text(.03,.97,f'连通中心区域约 {area:.2f} 平方米',transform=ax.transAxes,va='top',fontsize=11,
        bbox={'boxstyle':'round,pad=.5','fc':'white','ec':'none','alpha':.95})
fig.suptitle('为什么机器人可能被限制在一条通道里',fontsize=23,fontweight='bold',y=.97)
fig.text(.5,.915,'真实尺寸没有变；改变的是规划器如何表示机器人，以及如何检查它的朝向。',ha='center',fontsize=13)
fig.text(.065,.063,'下排是读取源场景后的离线几何诊断。绿色表示底座中心的连通区域，不代表已经扫描的面积，也不是执行轨迹。',fontsize=10.5)
fig.text(.065,.033,'右下采用略大于实际包络的 0.57 × 0.57 米方形，固定朝向；转弯能否完成仍需在线规划与执行验证。',fontsize=10.5)
fig.savefig(root/'research/footprint_diagnostic.png',dpi=170)
print(root/'research/footprint_diagnostic.png')
