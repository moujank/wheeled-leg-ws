# wheeled_mujoco.orchestration

该目录预留给任务选择和多任务协调逻辑。

## 当前文件

- `task_switcher.py`：小型进程内注册表与任务过渡描述辅助代码

## 当前状态

- 这里还没有实现策略混合或任务仲裁运行时
- 目前也不会直接改写 MuJoCo 控制输出

## 预期职责

单个 runner 只执行一个任务 rollout。编排层后续应负责决定：

- 何时切换任务
- 如何表示任务过渡
- 更高层控制器是否需要串联 locomotion、recovery、obstacle、grasp 等行为

这个边界应始终位于单个 runner 之上。
