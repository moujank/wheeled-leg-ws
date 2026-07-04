# wheeled_mujoco/source

该目录是 MuJoCo 侧 sim2sim 代码的可导入源码工作区。

职责划分：

- `wheeled_mujoco/sim2sim`：可运行的 sim2sim 适配器和 CLI。
- `wheeled_mujoco/tasks`：任务级 MuJoCo 资产/环境配置，命名与训练任务文件保持一致。
- `wheeled_mujoco/ros`：后续机器人控制仿真的 ROS 节点和接口预留。
- `wheeled_mujoco/orchestration`：后续任务切换和多任务耦合执行工具预留。

这个源码树之外的现有文件保持原样。当前稳定的 `minimal_sim2sim/run_base_locked.py` 仍然是行为基线。
