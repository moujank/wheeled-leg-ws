# wheeled_mujoco/source

该目录是 MuJoCo 侧 Python 代码的可导入源码树。

## 子包

- `wheeled_mujoco/tasks`：与训练侧命名和日志布局对齐的任务元数据
- `wheeled_mujoco/sim2sim`：runner 适配层与 CLI 参数转发层
- `wheeled_mujoco/ros`：预留的 ROS 接口骨架
- `wheeled_mujoco/orchestration`：预留的任务切换与协调辅助层
- `wheeled_mujoco/paths.py`：共享的仓库相对路径工具

## 设计边界

- 可复用 Python 逻辑应放在这里
- 一次性执行入口应放在 `modules/wheeled_mujoco/scripts`
- 低风险的 base-locked 行为基线仍位于 `modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py`

当前这个包刻意保持精简。它的职责是包装已经验证过的基线，而不是直接取代它。
