# wheeled_mujoco

该模块包含 wheeled-leg 工作空间在 MuJoCo 侧的资产、sim2sim 运行器，以及后续 ROS 和任务编排的预留结构。

当前目录结构：

- `asset/`：MuJoCo MJCF、URDF、mesh、纹理和相关机器人资产。
- `minimal_sim2sim/`：当前验证过的最小调试基线。除非明确在调这个基线，否则不要随意改动。
- `scripts/`：运行或检查 sim2sim 任务的命令行入口。
- `source/`：可导入的 MuJoCo 侧源码包，包含任务配置、运行适配器、ROS 接口和任务编排预留。
- `MUJOCO_DEBUG_HANDOFF.md`：当前 MuJoCo sim2sim 调试状态文件。

新的 sim2sim 工作调用顺序：

```text
scripts/run_*.py
  -> source/wheeled_mujoco/sim2sim
  -> source/wheeled_mujoco/tasks
  -> asset/ 和 wheeled_train logs
```

现有的 `minimal_sim2sim/run_base_locked.py` 仍然是 `base_locked` 启动和 reset 稳定性的行为参考。
