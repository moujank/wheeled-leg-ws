# 床头、床尾通道加宽后的探索实验

本实验将原 Code-as-Room 场景中的床头、床尾通道加宽，再从空白地图重新运行真实 SLAM Toolbox、Nav2 和在线探索。机器人仍使用 `wheeled_arm_v2` 的弯腿、收臂资产，以及完整矩形轮廓。移动由导航速度积分执行，未接入 RL checkpoint 或平衡动力学。

## 场景修改

使用 Isaac Sim 原生 USD API 创建独立叠加层：工作区 `research/scene_audit/code_as_room_open_passages/scene.usda`。原场景和原视觉资产的 SHA-256 在修改前后保持一致。

- 床两端的家具与对应装饰分别向外移动 0.65 米；床和机器人保持原尺寸。
- 地板从 9 × 6 米延长为 9 × 7.3 米；南北外墙及内部隔墙的端部相应延长，内门洞端点保留。
- 床头净间隙约 1.2075 米，床尾约 1.2700 米。
- 保留原有 222 个静态网格碰撞体；原外门洞新增一扇关闭门板及其碰撞体，使简化运动演示保持在屋内。
- 预览只隐藏近侧外墙的外观，两道内部隔墙保留；导航仿真加载的是完整碰撞场景。

`research/scene_audit/code_as_room_open_passages/scene_metadata.json` 记录每个物体的变换、地板范围与源文件哈希。`passage_audit.json` 是事后几何检查：更保守的 0.44 米半径圆也能从起点连通床左侧、床头和床尾通道。这个几何检查的地图与路线不会交给探索器。

## 重新生成场景与预览

从工作区根目录运行，每次只启动一个 GPU 仿真进程：

```powershell
D:/isaacsim/python.bat research/prepare_code_room_open_passages.py
D:/isaacsim/python.bat research/audit_open_passages.py
```

输出 `isaac_open_passages.png`、`isaac_topdown.png` 和 USD 场景。

## 从空白地图运行

```powershell
D:/Miniconda/python.exe modules/wheeled_navigation/scripts/run_autonomy.py --stage ros2 --controller dwb --scene-usd research/scene_audit/code_as_room_open_passages/scene.usda --seconds 540 --output modules/wheeled_navigation/artifacts/my_open_passages_run
```

每次选择新的输出目录。`--scene-usd` 仅传给仿真端；独立 ROS 探索进程仍只接收扫描、带误差里程计、TF、地图和导航状态，不读取场景布局，也不加载旧地图或旧路线。

导航参数沿用当前 DWB / Smac Hybrid A* 配置。本轮首先检验场景加宽后的结果，没有为了让机器人去左边而插入固定目标序列。

## 评价

```powershell
D:/isaacsim/python.bat modules/wheeled_navigation/scripts/analyze_autonomy.py modules/wheeled_navigation/artifacts/my_open_passages_run
```

仿真端把实际加载场景的碰撞体边界保存为本轮 `scene_colliders.json`，只供离线评价。评价器根据本轮真实地板范围计算覆盖率，并检查实际轨迹上的完整机器人包络。新场景面积为 65.7 平方米，不能继续沿用旧场景的 54 平方米分母。

指标含义与原自主探索实验一致：已观测 XY 栅格投影面积不是完整三维重建率；`phase=complete` 表示本轮结束后返回成功，不表示已经覆盖所有房间。

实际输出位于 `artifacts/autonomy_ros_open_passages_v1/` 和 `artifacts/autonomy_ros_open_passages_v2/`。DWB 第一轮观测 39.83 平方米，返航失败；RPP 第二轮观测 42.00 平方米，占 65.7 平方米房间的 63.9%，并成功返回。两轮均实际走到床左侧，离线完整轮廓与场景障碍 AABB 检查均没有重叠。返回成功只表示该轮结束时返航成功。

## 进一步移除左侧入口小桌

左侧休息区门口的小桌子及其上下两组书本原本占据了入口。独立场景 `research/scene_audit/code_as_room_open_passages_no_table/scene.usda` 在上述加宽修改上，将三个物体的 USD prim 设为 inactive，使外观、传感器可见几何和碰撞体一起退出场景。共移除 14 个网格碰撞体，剩余 208 个原始网格碰撞体与 1 个关闭外门的立方体碰撞体。原文件保持完整。

```powershell
D:/isaacsim/python.bat research/prepare_code_room_open_passages.py --remove-lounge-table
D:/isaacsim/python.bat research/audit_open_passages.py --scene-dir research/scene_audit/code_as_room_open_passages_no_table
D:/Miniconda/python.exe modules/wheeled_navigation/scripts/run_autonomy.py --stage ros2 --controller rpp --domain-id 84 --scene-usd research/scene_audit/code_as_room_open_passages_no_table/scene.usda --seconds 540 --output modules/wheeled_navigation/artifacts/autonomy_ros_no_table_v1
D:/isaacsim/python.bat modules/wheeled_navigation/scripts/analyze_autonomy.py modules/wheeled_navigation/artifacts/autonomy_ros_no_table_v1
D:/isaacsim/python.bat research/render_no_table_results.py --run autonomy_ros_no_table_v1
```

重跑请选择新的输出目录，并避免与仍在运行的 ROS domain 共用同一个编号。几何预检查的 0.44 米半径圆现在也能从起点进入左侧休息区。探索器不读取这个预检查结果，仍需用实时扫描自行找到入口。

`no_table_comparison.png` 对照移除前后两次 RPP 探索的真实地图和轨迹；`mapping_replay.gif` 回放本轮保存的扫描评价栅格与实际运动轨迹。动图中的栅格在事后用仿真位姿对齐，实际导航仍使用 SLAM Toolbox 地图；不能把评价栅格解释为导航输入。原生渲染图是修改后场景的静态预览，不能解释为运动过程中的相机帧。

移除小桌后完成的两轮均保存了实际地图，但均未成功返航，不能当作全屋探索已经完成：

| 本轮 | 跟随器 | 已观测 XY 投影 | 占房间面积 | 实际行驶 | 结束状态 |
| --- | --- | --- | --- | --- | --- |
| `autonomy_ros_no_table_v1` | RPP | 43.2575 m² | 65.84% | 11.68 m | `return_failed` |
| `autonomy_ros_no_table_v2` | DWB | 44.1475 m² | 67.20% | 10.81 m | `return_failed` |

DWB 这一轮在约 60.2 秒时底座中心越过了左隔墙西侧面，最左 x 约 -2.807 米，但完整身体包络尚未全部越过隔墙西侧面，不能说已经完成穿门和深入休息区。左侧区域的已观测投影从使用相同 DWB 的移除前一轮约 4.03 平方米增加到约 7.95 平方米。两轮实际轨迹的完整矩形轮廓与场景障碍 AABB 均无重叠。导航日志仍有局部跟随碰撞预测、目标执行失败与返航失败；几何通路已打开不代表导航软件已经可靠通过。

## 单独查看建图与三维点云

`ros/slam_map/map.pgm` 与 `map.yaml` 是实际 SLAM Toolbox 二维占据栅格输出，`reconstruction/slam_map_only.png` 单独显示地图，不包含轨迹或源模型。

三维点云来自同一轮 `sensor_history.npz` 保存的原生三维雷达深度；使用 `ros/estimates.json` 中 SLAM Toolbox 的 x/y/yaw 位姿做时间插值，再按固定安装位置 `[0.16, 0, 0.84]` 米拼接。转换程序不读取仿真真值位姿或源场景网格。机器人贴地平动，因此这里假设 z、横滚和俯仰固定。这是三维扫描累积，尚未运行完整六自由度三维 SLAM。

```powershell
D:/isaacsim/python.bat research/capture_lidar_ray_calibration.py
D:/isaacsim/python.bat research/export_slam_reconstruction.py modules/wheeled_navigation/artifacts/autonomy_ros_no_table_v2
```

射线标定从相同 360×140 度、0.5×1 度分辨率的原生 PhysX 点云 API 获取单位方向。转换后重建的平面投影与实际用于导航的已保存扫描在抽查的 8 帧中最大误差约 0.123 毫米，与深度量化步长相当。若改传感器角度或分辨率，必须重新标定并通过数据形状和投影检查。

DWB 本轮使用 194 帧扫描，约 252.9 万个有效回波经 3.5 厘米体素去重后得到 248,330 个点。输出 `reconstruction/reconstruction_overview.png`、`slam_pose_cloud.ply`、`pointcloud_viewer.html` 和 `reconstruction_report.json`。PLY 包含 XYZ 和同场景原生 RGB 渲染配准的颜色，已经替换此前的高度伪彩色。HTML 为无外部依赖的本地点云查看器，支持拖动旋转、滚轮缩放与俯视。内置浏览器拒绝 file 协议，未完成其交互预览检查，可由用户在本地浏览器打开；静态 PNG 已实际查看。PNG 为便于看见室内，仅隐藏近侧外墙的高处点；PLY 与 HTML 保留全部回波，未观测表面不补全。

### 原场景 RGB 配色

激光原始数据不含 RGB。`research/colorize_lidar_from_isaac_rgb.py` 在同一静态场景中按记录的传感器位置补渲染 RGB 与深度图，通过投影和遮挡检查，把图像颜色配准到已有回波。这里为外观配准读取了记录的仿真位姿，仅用于相机重放及回波与 RGB 的对应；点云的 XYZ 仍保持先前的 SLAM 拼接结果，没有替换成真值位置。这些 RGB 图像是建图结束后补渲染的，不能描述为探索过程中同步采集的相机数据。

```powershell
D:/isaacsim/python.bat research/colorize_lidar_from_isaac_rgb.py modules/wheeled_navigation/artifacts/autonomy_ros_no_table_v2
D:/isaacsim/python.bat research/export_slam_reconstruction.py modules/wheeled_navigation/artifacts/autonomy_ros_no_table_v2 --rgb modules/wheeled_navigation/artifacts/autonomy_ros_no_table_v2/reconstruction/native_rgb_colors.npz
```

本轮渲染 27 组记录位置的六向相机视图。248,033 个点直接取得深度一致的 RGB 像素颜色，129 个点使用距离不超过 6.5 厘米的已配色邻点，168 个无法可靠配色的点保留中性灰。颜色包含原场景材质和原生照明的效果，不是高度着色。`rgb_registration_report.json` 保存配色比例、视点与方法；`rgb_pointcloud.png` 是原色点云单独预览。导出器自动使用已存在的 `native_rgb_colors.npz`；没有 RGB 数据时仅输出中性灰，不再默认生成高度伪彩色。
