# 项目待办

[文档索引](README.md) · [迁移流程](SIM2SIM_DYNAMIC_FLOW.md) · [MuJoCo 文档](../modules/wheeled_mujoco/docs/README.md)

## Isaac Lab 训练侧

- [ ] base-locked 对齐 MuJoCo 参数后重新训练。
- [ ] 继续优化 hard 任务。
- [ ] 明确从 hard 开始的输出文档位置及路径约定；实验产物说明沿用 `logs/rsl_rl/<experiment>/best/notes.md`，跨实验说明放入训练模块 `docs/`。

## MuJoCo 侧

- [ ] 整体学习 `minimal_sim2sim/` 的闭环实现。
- [ ] 按入口、运行时、任务配置与资产职责，进一步细化 sim2sim。

以上待办由原根目录 `todo.md` 整理而来，完成状态仍待后续实际工作确认。
