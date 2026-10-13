# MuJoCo Sim2Sim 调试交接

[文档索引](README.md) · [完整调试历史](archive/MUJOCO_DEBUG_HISTORY.md) · [一致性清单](MUJOCO_CONSISTENCY_CHECKLIST.md) · [项目待办](../../../docs/TODO.md)

文档整理日期：2026-10-12。本次仅整理文档，未运行仿真或重新验证策略。

## 状态与证据

- 运行基线入口为 `minimal_sim2sim/run_base_locked.py`；任务包装器位于 `scripts/`，目录职责见 [架构说明](SIM2SIM_ARCHITECTURE.md)。
- 旧记录曾在 27D 策略及保守保护参数下报告低速、0.5 m/s 命令长跑不倒；记录也明确指出有效速度被降额，不能等同于真实速度跟踪达标。
- 2026-07-08 的 90D 策略记录报告启动/handoff 回归失败，与较早的稳定记录属于不同条件。原始命令、输出和失败样例已完整归档。
- 原交接中的高度 `0.32`、旧观测 `27D` 和动作尺度等说明不作为统一默认值。现有 runner 支持旧 `27D` 观测，以及带机体系线速度的 `30 × history` 观测；`90D` 对应 3 帧。实际输入维度由导出策略决定。
- 策略选择以 [POLICY_INDEX.md](../../wheeled_train/wheeled_leg/logs/rsl_rl/POLICY_INDEX.md) 及对应 `best/notes.md` 为入口。例如 [hard 策略说明](../../wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked_hard/best/notes.md) 已包含 2026-08-08 的选择记录，早期调试证据不自动适用于该 checkpoint。

## 下一次调试需要固定的输入

1. task、experiment、run、checkpoint 和导出策略路径。
2. 该 run 的 `params/env.yaml`，包括观测顺序/历史长度、动作尺度、reset 和 actuator 参数。
3. 活动 MJCF、CLI 覆盖参数以及启动流程。
4. 用上述固定条件复核启动、静止、小速度及目标速度，分别记录“不倒”和“速度跟踪”结果。

具体排查步骤维护于 [一致性清单](MUJOCO_CONSISTENCY_CHECKLIST.md)，资产查询步骤维护于 [ASSET_INDEX.md](ASSET_INDEX.md)，跨模块迁移流程维护于 [SIM2SIM_DYNAMIC_FLOW.md](../../../docs/SIM2SIM_DYNAMIC_FLOW.md)。

## 状态更新规则

后续实际调试完成后，在本文件更新当前状态，在历史文档追加具备日期的证据。每次记录包含：

- 修改的文件、参数或逻辑，以及固定的 task/run/checkpoint/policy/MJCF。
- 是否影响启动；启动命令与结果。
- 短跑结果；非零速度的 20s 长跑结果。
- effective command、实际速度、base_z、pitch、yaw、hub_vel、ctrl_sat 和接触状态。
- 失败前的关键日志、当前结论和下一步。

历史采用过 4.5s 启动、6s 短跑、20s 非零速度长跑以及目标通过后的 viewer/real-time 验证。继续调试时须先确认这些验证条件适用于选定策略，再记录新的结果。
