# Code-as-Room 房间中的雷达建图与目标点规划

这份演示用于在暂时没有最新 RL checkpoint、激光雷达和摄像头时，检查丰富室内场景中的扫描建图与目标点规划。它按预设路线直接修改机器人根节点的位置和朝向，在每个采样位姿等待三个物理步，再读取 Isaac Sim 的 PhysX 雷达数据。机器人外观来自工程原有 USD，房间来自本地 Code-as-Room 示例。

本次没有运行 RL 策略，没有施加速度或关节力矩，也没有训练机器人。定位使用仿真真值，因此这里验证的是“已知位姿下的扫描建图 + 地图路径搜索”，没有运行位姿估计、回环检测、SLAM Toolbox 或 Nav2。

## 场景与数据来源

源文件：`E:\project\Code-as-Room-main\example\run_20260930_153912_example1\stage12_render\scene.glb`。

转换文件位于工作区 `research/scene_audit/code_as_room/`：

- `visual_scene.usdc` 与 `textures/` 保留原房间外观。
- `scene.usda` 叠加 222 个静态三角网格碰撞体与照明。
- `robot_visual.usdc` 是去除关节、刚体、碰撞与传感器配置的机器人外观副本。由 `research/prepare_robot_visual.py` 生成，原机器人文件不变。避免将原关节配置载入纯位姿采样场景时触发 Isaac 原生崩溃。
- `conversion.json` 和 `physics_overlay.json` 记录源文件与转换信息。

房间面积约 9 × 6 米，包括休闲区、卧室、衣帽间，以及床、床尾凳、沙发、边桌、衣柜、岛台和植物。休闲区门口的边桌挤占通道；这次没有移动家具，机器人未进入休闲区。通过门口射线看到的区域仍然可以出现在地图中，家具后方没有看到的部分保持未知。

## 建图与规划的数据流

1. 根据家具的包围盒设计一条预设采样路线。这一步允许知道场景布局，不是自主探索。采样路线设计的离散网格为 5 厘米，使用 0.22 米演示圆盘避开家具。
2. 在预设路线每隔约 0.12 米设置根位姿，挂在该根节点下的四个虚拟雷达随之改变位置和朝向。每个位姿等待三个物理步，读取四个高度的真实 PhysX 距离数组。
3. 扫描高度为 0.18、0.38、0.62 和 0.90 米，每层 360 个方向。同一个方向选择四层最近的有效回波，得到偏保守的二维距离投影。这个配置是四层虚拟扫描，不是某款真实三维雷达的硬件仿真；层间仍可能漏掉障碍物。
4. 一个空白的建图器只接收距离、角度和仿真真值位姿。射线经过处降低占据概率，命中终点提高占据概率。最终地图分辨率为 2.5 厘米。5 厘米地图在膨胀后不能正确分辨其中一处窄通道，因此提高了分辨率，没有人工补画空闲栅格。
5. A* 只读取这张扫描地图。将未知区域与障碍物一起膨胀，按半径 0.25 米的虚拟圆盘寻找路径，禁止斜穿障碍物角落。完成搜索后，再用原房间的家具包围盒独立检查路径中心的最小距离。
6. 第一条目标路径还会在 Isaac 中通过根位姿设置逐点回放，保存 `scripted_replay.json`。这只是路线回放，不表示真实控制器能够完成行走。

0.22 米与 0.25 米是演示圆盘参数，没有覆盖整机所有外形、机械臂、转身扫过的空间与动力学。机器人原始外观比圆盘复杂；不能据此断言实际机器人可以穿过所有窄通道。

## 在本机复现

目前使用本机 `D:\isaacsim` 的 Isaac Sim 4.5.0。整个流程不需要额外下载网络模型权重。

在工作区根目录运行：

```powershell
.\wheeled-leg-ws\modules\wheeled_sim_bridge\run_code_room_demo.ps1
```

带窗口运行：

```powershell
.\wheeled-leg-ws\modules\wheeled_sim_bridge\run_code_room_demo.ps1 -Gui
```

脚本会先检查本地房间文件，必要时生成机器人外观副本，运行 Isaac，最后离线重建地图并核对扫描和路径。请只运行一个 Isaac GPU 进程。

也可以单独运行扫描程序，传入其他已准备好的路径：

```powershell
D:\isaacsim\python.bat .\wheeled-leg-ws\modules\wheeled_sim_bridge\scripts\run_code_room_mapping.py --scene E:\project\robot_project\research\scene_audit\code_as_room\scene.usda --geometry E:\project\robot_project\research\scene_audit\code_as_room_geometry\code_as_room_colliders.json
```

这个房间的预设采样点与几何审计函数放在 `scan_nav/code_room.py`。它们依赖此房间布局；更换房间时，需要重新设计采样路线与楼面边界。现有代码不是任意 GLB 一键自主探索工具。

## 结果与预览

输出位于 `modules/wheeled_navigation/artifacts/code_as_room_demo/`。

| 文件 | 内容 |
| --- | --- |
| `code_as_room_preview.png` | 3000 × 2350 的总览，包含真实 Isaac 视口、扫描地图、路径和运行统计 |
| `mapping_method_diagram.png` | 2800 × 1880 的方法图，说明数据来源、扫描高度、定位条件与验证边界 |
| `mapping_progress.gif` | 五个阶段的实际地图累积过程 |
| `isaac_full_room.png` | 完整墙面的 Isaac 场景截图 |
| `isaac_cutaway.png` | 为看清家具，仅在截图时隐藏前墙和部分隔墙的视口图 |
| `isaac_topdown.png` | 完整房间的俯视视口图 |
| `isaac_robot_closeup.png` | 机器人与家具的近景 |
| `measured_map_and_paths.png` | 三条目标路径与完整采样路线 |
| `report.html` | 自包含图片报告，不需要联网 |
| `scans.npz` | 原始 uint16 距离、融合距离、扫描角度、高度、位姿与仿真时间 |
| `grid.npz`、`map.pgm`、`map.yaml` | 数值地图与 ROS 兼容地图文件 |
| `metrics.json` | 三条路径、几何检查、无效目标测试与运行指标 |
| `recording_validation.json` | 离线精确重建与路径复现的检查结果 |
| `navigation_scene.usda` | 带机器人和路线的本地 USD 叠加场景，引用原房间文件 |

截图的墙面隐藏发生在采样完成之后，只写入 session layer，地图不会受这种展示处理影响。输出的场景文件保留完整墙体。图中的蓝虚线是预设采样路线，绿线是跨房间规划路线，橙线是绕岛台路线，紫线是卧室东侧路线。

离线检查无需启动 Isaac 图形引擎：

```powershell
D:\isaacsim\python.bat .\wheeled-leg-ws\modules\wheeled_navigation\scripts\validate_code_room_recording.py .\wheeled-leg-ws\modules\wheeled_navigation\artifacts\code_as_room_demo --geometry .\research\scene_audit\code_as_room_geometry\code_as_room_colliders.json
```

离线程序从原始扫描重新计算融合距离，再从空白地图积分，要求 log-odds 数组与观测掩码逐元素相等；随后重新规划全部目标，并复查障碍目标与几何间距。这些检查用于证实结果确实来自保存的扫描数据。
