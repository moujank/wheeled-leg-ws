# Code-as-Room 雷达探索动画

动画在 Isaac Sim 4.5 中实际渲染三维场景和机器人，表现房间从暗处随着扫描逐渐显现的过程。视频版为 1920 × 1080、24 fps、40 秒，包含 2 秒开场、34 秒预设路线运动和 4 秒结尾停留；GIF 是 960 × 540、8 fps 的便携预览。右侧为扫描覆盖示意和路线进度，三维画面原生渲染区域为 1536 × 1080。

## 机器人与姿态

使用 `modules/wheeled_description/wheeled_arm_v2` 的 URDF 和原始 STL。此前使用 `wheeled_leg_v2` 默认展开姿态的样片已被替换。

姿态直接读取训练资产配置 `modules/wheeled_train/wheeled_leg/source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/asset/wheeled_arm_v2.py`：

- HipL / HipR：+0.65 / -0.65 rad。
- KneeL / KneeR：+1.30 / -1.30 rad。
- Arm_Y1：-0.65 rad；Arm_Y2：1.20 rad；其余臂关节为 0。

`research/prepare_wheeled_arm_visual.py` 解析 URDF 关节树，计算正向运动学，把原始视觉网格放到对应姿态，再生成 `wheeled_arm_v2_bent_visual.usdc`。它没有缩放机器人，没有修改原始模型。轮子底部放在 z=0.006 m，模型保持弯腿、收臂状态。机器人姿态近景保存在输出目录的 `robot_pose_closeup.png`。

## 运动与扫描

机器人沿之前的采样路线走一圈，回到起点 `(0.9, -1.4)`。程序生成世界坐标下的平面速度和角速度 `(vx, vy, wz)`，按帧积分到根节点的位置、朝向中。起步和停止具有速度过渡，角速度限制为 2.1 rad/s。关节保持名义姿态；这是允许侧向平移的展示运动，没有调用 RL 策略、轮足平衡控制器或动力学。

为了实际扫到床面、桌面和高低不同的家具，动画使用独立的虚拟三维 PhysX 雷达：水平 360°、水平角间隔 1°、垂直视场 80°、垂直角间隔 5°。Isaac 实际返回 17 个垂直通道。雷达安装点相对机器人根节点为 `(0.16, 0, 0.84)` m，青色外壳与扫描射线从同一位置出发。显示扇区宽 48°，每 1.8 秒转一圈。

每一帧设置机器人根位姿后等待三个物理步，读取原始 uint16 深度、局部三维命中点和命中 USD prim 路径。原始房间的墙体碰撞保持完整。按用户要求，两道内部隔墙及各自的门洞段完整保留在画面中；镜头提高到更接近俯视的位置，只对前侧外墙隐藏渲染外观。还会用命中点到对应场景包围盒的距离核对坐标变换，拒绝明显不一致的结果。

## 暗区怎样变亮

亮度是根据扫描数据制作的观测可视化效果，不是物理灯光照明，也不是三维重建结果：

1. 向下与水平射线在地面上的可见投影累积成区域。相邻一度射线之间通过扇面插值连接，边缘略作平滑，避免远处出现条纹。
2. 当回波命中一个家具网格部件时，对应部件渐渐恢复正常亮度。采用部件级显现，不能解释为该部件的每一个背面、顶面都被扫描。
3. 首次观测时间被永久记录，已显现的区域不会重新变暗。地面约 0.36 秒、家具约 0.5 秒完成渐变。
4. 始终被遮挡的区域继续保持暗色。不会在结尾统一点亮全部房间。

投影覆盖比例与之前“最近回波融合的二维占据地图覆盖比例”含义不同，不应直接比较或作为 SLAM 精度。之前的建图文件、训练文件和 MuJoCo 代码均未改动。

## 复现

在 `E:\project\robot_project` 运行：

```powershell
.\wheeled-leg-ws\modules\wheeled_sim_bridge\render_code_room_video.ps1
```

快速样片使用 `-Preview`。样片为 18 秒、8 fps，用于检查整个路线与显示效果。

视频编码器安装在工作区 `research/video_deps/`，未安装到 Isaac 的全局环境。新环境需要先执行：

```powershell
D:\isaacsim\python.bat -m pip install --no-deps --target research/video_deps imageio-ffmpeg==0.6.0
```

输出位于 `modules/wheeled_navigation/artifacts/code_as_room_video/`：

| 文件 | 用途 |
| --- | --- |
| `code_as_room_lidar_reveal.mp4` | H.264 MP4 正式视频 |
| `code_as_room_lidar_reveal.gif` | GIF 预览 |
| `preview.html` | 本地播放页 |
| `storyboard.png` | 四个时刻的画面 |
| `robot_pose_closeup.png` | 弯腿、收臂的机器人近景 |
| `animation_scans.npz` | 原始测距、角度、三维命中点、传感器位置、首次观测时间与覆盖进度 |
| `motion_commands.json` | 每帧位姿、速度指令与路线进度 |
| `hit_parts.json` | 场景部件编号及每帧命中列表 |
| `camera_surface_projection.npz` | 用于三维画面亮度合成的深度反投影和部件索引 |
| `render_status.json` | 雷达参数、坐标检查、渲染指标和文件路径 |
| `media_validation.json` | 编码后逐帧解码、亮度变化、运动积分和显现单调性的检查结果 |

`reveal_scene.usda` 只保存了用于渲染的场景叠加层；逐帧暗区合成发生在输出视频中，单独打开 USD 不会播放这种亮度变化。
