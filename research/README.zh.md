# 导航仿真与结果导出的复现入口

所有命令从 **Git 仓库根目录 `wheeled-leg-ws`** 执行。历史实验曾使用仓库外的研究目录；当前代码已整理为仓库内 `research/`，不要求克隆目录仍叫 `wheeled-leg-ws`。完整结果与限制见[工作总结](../docs/NAVIGATION_PROGRESS.zh.md)。

本机验证环境为 Windows、Isaac Sim 4.5.0、RTX 4060 Laptop 8 GB。ROS 端为 Pixi 隔离的 ROS 2 Humble、SLAM Toolbox 与 Nav2。Isaac Python 与 ROS Python 必须分开。当前不依赖 Isaac Lab 训练接口，不需要神经网络权重。

## 环境与外部资产

准备 Isaac Sim 和 Pixi。默认 Isaac 路径为 `D:/isaacsim`；其他安装路径可设置 `ISAAC_PATH`，启动器可用 `--isaac-python` 或 `ISAACSIM_PYTHON` 指定其 Python 启动器。Pixi 可通过 PATH、`PIXI_EXE` 或 `--pixi` 指定，不提交运行时二进制。

```powershell
$env:ISAAC_PATH='D:/isaacsim'
$env:ISAACSIM_PYTHON='D:/isaacsim/python.bat'
pixi install --manifest-path research/ros2_runtime/pixi.toml
```

依赖清单与锁文件已纳入版本管理，实际环境目录 `.pixi/` 忽略。CPU 分析或旧视频导出可在独立环境安装 `research/requirements-analysis.txt`；不要把 ROS 的 Python 包路径注入 Isaac Python。

首先从仓库现有 URDF 生成正确的弯腿、收臂可视资产：

```powershell
D:/isaacsim/python.bat research/prepare_wheeled_arm_visual.py
```

生成的 USD 在 `research/scene_audit/code_as_room/`，不纳入 Git。它是静态外观，不能当作 RL 或动力学执行器。

陌生环境原生验证场景由代码直接创建，不需要第三方房间。Code-as-Room 演示则需要自行准备已经获得使用许可的家具场景 GLB。本仓库不重新分发第三方房间网格与纹理，也不包含生成场景时用到的服务凭据。使用本机 Blender 把 GLB 转为 USD：

```powershell
blender --background --python research/convert_code_as_room.py -- --input C:/path/to/scene.glb --output research/scene_audit/code_as_room/visual_scene.usdc
D:/isaacsim/python.bat research/add_static_scene_physics.py
D:/isaacsim/python.bat research/audit_usd_assets.py --output research/scene_audit/code_as_room_geometry research/scene_audit/code_as_room/scene.usda
```

加宽脚本针对本次 Code-as-Room 示例的物体命名与布局，不是任意 GLB 的通用房间改造器；新场景需要先核对 prim 名称、尺寸与单位。历史源场景来自本机 `Code-as-Room` 的 `stage12_render/scene.glb`，上游工程许可不代表其中每个外部家具资产都可直接转授权。转换脚本只读取输入，不改写源 GLB。

## 修改后的房间

```powershell
D:/isaacsim/python.bat research/prepare_code_room_open_passages.py --remove-lounge-table
D:/isaacsim/python.bat research/audit_open_passages.py --scene-dir research/scene_audit/code_as_room_open_passages_no_table
```

床头与床尾间隙分别约 1.21 与 1.27 米，左侧入口小桌与书本的视觉及碰撞体一并移除，两道内部隔墙保留。生成的是独立 USD 层；几何审计的地图和路径不交给在线探索器。

## 从空白地图运行

先检查不依赖 ROS 的传感器与探索闭环：

```powershell
python modules/wheeled_navigation/scripts/run_autonomy.py --stage native --scene validation --seconds 240 --dynamic-test --output modules/wheeled_navigation/artifacts/my_native_run
```

运行真实 SLAM Toolbox / Nav2：

```powershell
python modules/wheeled_navigation/scripts/run_autonomy.py --stage ros2 --controller dwb --scene-usd research/scene_audit/code_as_room_open_passages_no_table/scene.usda --seconds 540 --output modules/wheeled_navigation/artifacts/my_ros_run
```

每次使用新输出目录；一次只启动一个 Isaac GPU 进程。探索器不接受预设路线，不读取源场景几何。控制器可用 `--controller rpp` 比较；不同控制器与运行时长的结果不能当作严格单变量对照。

```powershell
D:/isaacsim/python.bat modules/wheeled_navigation/scripts/analyze_autonomy.py modules/wheeled_navigation/artifacts/my_ros_run
```

输出实际 SLAM 地图、执行轨迹、位姿误差与完整轮廓检查。`complete` 仅表示本轮返回成功；地图覆盖与失败情况必须另看报告。原始距离、日志和逐步地图快照通常很大，`artifacts/` 已整体忽略，避免意外上传。

## 原色点云

当前三维雷达布局为 360×140 度、0.5×1 度分辨率。先获取原生射线标定，再从本轮记录生成点云，最后按记录传感器位置补渲染 RGB 配色：

```powershell
D:/isaacsim/python.bat research/capture_lidar_ray_calibration.py
D:/isaacsim/python.bat research/export_slam_reconstruction.py modules/wheeled_navigation/artifacts/my_ros_run
D:/isaacsim/python.bat research/colorize_lidar_from_isaac_rgb.py modules/wheeled_navigation/artifacts/my_ros_run
D:/isaacsim/python.bat research/export_slam_reconstruction.py modules/wheeled_navigation/artifacts/my_ros_run
```

导出器自动读取本轮 `native_rgb_colors.npz`。第一次尚无 RGB 时使用中性灰，不再默认按高度分色。颜色配准使用同场景原生相机 RGB 与深度检查；点位置仍为 SLAM 平面位姿拼接，不能称为完整六自由度三维 SLAM。相机画面是事后重放补渲染，未声称探索时已同步采集 RGB。

## 展示图与历史对照

```powershell
D:/isaacsim/python.bat research/render_modified_room_white.py --transparent-topdown
D:/isaacsim/python.bat research/render_no_table_results.py --run autonomy_ros_no_table_v2
D:/isaacsim/python.bat research/export_clean_visuals.py
```

正交俯视场景边线横平竖直，背景透明。后两个工具用于本次历史实验的固定目录名，并依赖移除前后的完整记录；新运行应修改配置或指定相应运行名，不能用缺失记录生成“对照结果”。点云 PLY 和 HTML 保留全部点，静态预览只切去近侧外墙的高处点以便观察内部。HTML 是本地无外部依赖文件；内置浏览器拒绝 file 协议，交互检查尚未完成，静态 PNG 已查看。

## CPU 检查

```powershell
D:/isaacsim/python.bat -m unittest discover -s modules/wheeled_navigation/tests -v
D:/isaacsim/python.bat -m compileall -q modules/wheeled_navigation modules/wheeled_sim_bridge/scripts research
```

测试覆盖未知区域不可行驶、墙角穿越、足迹朝向、扫描对探索目标的影响、新障碍引发重规划与速度积分边界。上述 CPU 测试不代替 Isaac 传感器仿真或真实 ROS 闭环。
