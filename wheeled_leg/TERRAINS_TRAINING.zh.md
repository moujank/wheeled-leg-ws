# Terrains 低速越障训练与验收

本次继续使用 `Wheeled-Leg-Terrains`，单策略、持续前向 0.25 m/s。先解决困难地形通行，再讨论扩展速度。完整曲线分析、迁移对照和历史文件清单见 [实验报告](reports/terrain_transfer_20261020/REPORT.zh.md)，固定选模结果见 [选模表](reports/terrain_transfer_20261020/selection_results.zh.md)。旧版说明已存为 `reports/terrain_transfer_20261020/previous_training_notes.zh.md`，其中旧的 model_5000 推荐已被本次固定评估替代。

## 初始化与训练阶段

本次选中：`logs/rsl_rl/wheeled_leg_terrains/2026-10-20_09-29-55/model_6000.pt`。它来自从零训练的实验，但后续工作是从该已学会地形运动的模型微调，不需要再从随机权重或平地策略重来。第7/9行重点类型平均通过率36.875%，尚未达到困难地形可靠通行目标。

最终选择记录在 `reports/terrain_transfer_20261020/selection_summary.json` 的 `selected_checkpoint`。下面命令直接读取已完成的筛选结果，校验文件存在；路径不是根据运行时间猜测生成。

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
TERRAIN_PY=/home/moujank/miniconda3/envs/env_isaaclab/bin/python
TERRAIN_INIT=$("$TERRAIN_PY" -c 'import json; from pathlib import Path; d=json.loads(Path("reports/terrain_transfer_20261020/selection_summary.json").read_text()); assert d["complete"]; p=Path(d["selected_checkpoint"]); assert p.is_file(); print(p)')
"$TERRAIN_PY" scripts/rsl_rl/train.py \
  --task Wheeled-Leg-Terrains --headless --num_envs 4096 \
  --max_iterations 2000 --seed 42 --terrain_phase focus \
  --run_name low_speed_focus --init_checkpoint "$TERRAIN_INIT" \
  env.scene.terrain.visual_material=null env.scene.sky_light.spawn.texture_file=null
```

两个 null 仅关闭远程视觉材质，避免离线训练依赖网络，不改变地形碰撞几何。需要完整画面时可去掉。

使用 `--init_checkpoint` 启动新的实验、优化器与课程，不与 `--resume` 混用。网络、动作噪声和观测归一化一起加载；成熟统计保留原 count，避免把已稳定控制器的统计计数重置成 10000。学习率为固定 1e-4。新日志目录保存 `initialization.json`，记录源路径、SHA256、count 和阶段。正常续跑同一实验才考虑 `--resume`。

三阶段只改变 24 列地形采样比例，不改变机器人、82/89 维输入、动作接口或地形最大几何难度。各阶段从第 0 行开始，连续两次满足通行要求后升级。

| 阶段 | 平地 | 坡 | 反坡 | 上台阶 | 下台阶 | 粗糙 | 波浪 | 离散障碍 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| foundation | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 |
| focus | 2 | 1 | 1 | 3 | 5 | 6 | 1 | 5 |
| retention | 3 | 2 | 2 | 3 | 4 | 4 | 2 | 4 |

本次推荐先 focus 2000 轮，每 500 轮固定评估。随后从 focus 中实测较好的 checkpoint 开始 retention 1000–2000 轮，恢复易地形覆盖；将上述命令改成 `--terrain_phase retention`、新的 run_name 和该实际 checkpoint。不要默认使用最后一轮。foundation 用于从零起步或低行通行验证失败的回退；没有必要先放弃已学到的地形技能、重新训练平地。

## 固定条件评估与可视化

将真实模型绝对路径作为参数传给探针；每个环境只计第一个回合，避免快速摔倒环境重复计数。

```bash
TERRAIN_CANDIDATE="$TERRAIN_INIT"
"$TERRAIN_PY" scripts/rsl_rl/probe_terrain_transfer.py \
  --headless --checkpoint "$TERRAIN_CANDIDATE" --scene terrain \
  --level 7 --speed 0.25 --num_envs 192 --seed 42 --terrain-seed 43 \
  --preserve-count --output reports/terrain_transfer_20261020/candidate_row7_seed42.json
```

分别评估行 0、4、7、9。正式验收使用 reset seed 42/43/44，并交叉使用 geometry seed 43/44/45；192 环境只是筛查，最终每个重点类型、种子与行应累计至少 100 个首回合。更大样本优先使用 `--num_envs 768`（当前 focus 分配下，下台阶160、粗糙192、离散160），并保留原始 JSON。重复使用相同输出路径会覆盖旧结果，正式评估应包含模型、行、两种 seed。

报告按 `first_episode_by_type` 的 successes/episodes、falls/episodes，检查 completed 与 episodes 是否相等。`traversal_counts_all_completed` 含首回合后的重复尝试，不作为公平选模的分母。选模重点看第7/9行下台阶、粗糙和离散障碍的等权平均，同时检查易地形能力和物理失败率。

建议验收目标：容易行每种类型通过率≥95%；困难行重点类型≥80%、物理失败率≤10%，并给出多种子置信区间。当前候选尚未达成。通行条件衡量位移、时长、跟踪和姿态，不能保证接触随机地图中每个障碍。对固定障碍尺寸的能力还需构造实际路径与障碍接触验证。

界面检查可使用：

```bash
"$TERRAIN_PY" scripts/rsl_rl/play.py \
  --task Wheeled-Leg-Terrains-Play --num_envs 48 \
  --checkpoint "$TERRAIN_CANDIDATE" --seed 42 \
  --terrain_level 7 --terrain_speed 0.25 --max_steps 25000
```

Play 使用名义模型、关闭部分随机化，适合观察动作；上述 probe 保留训练随机化，适合固定条件选模，两者结果不能直接混在一个表里。

## 奖励曲线的使用

`Train/mean_reward` 为最近已结束回合的累计奖励，回合提前通过边界也会降低它。不要只按总奖励峰值选模。同时查看有效移动误差、停滞比例、接触/倾斜失败率，以及各类型固定难度成功率。`Episode_Reward/*` 是回合奖励积分除以固定20秒，提前结束会一起下降。

训练升级阈值为至少前进2.6m、达到积分命令距离65%、至少8秒有效回合、姿态/横向偏移合格、无物理失败，跟踪误差阈值0.15m/s；固定评估采用更严格的0.10m/s。不要把地形均值或 time_out 比例当成成功率。`base_contact` 还包含 Hip/Knee 触地。

若 focus 后粗糙地形仍几乎零成功，先检查轮接触顺序、抬轮高度、腿目标/扭矩/速度饱和、扫描是否能分辨障碍，以及摔倒前状态；每次只做一个可验证修改，避免同时改网络、奖励、噪声和课程而失去因果依据。

## 实际迁移计划

1. 策略与归一化作为同一部署产物保存，核对关节顺序/符号、单位、默认偏置、动作裁剪和125Hz控制周期。执行器和闭环延迟按实测校准。
2. 高度扫描的54维顺序、坐标系、间距、范围和缺测处理必须与训练一致。真实系统没有等价感知时，先做感知适配或教师学生训练，不能直接把扫描置零。
3. 先做另一仿真器对照，使用实测质量/惯量、摩擦、延迟、噪声与电机能力做随机化，再在保护条件下做低速平地、单障碍、连续地形逐级验证，记录通过率、姿态、电流/扭矩和打滑。
4. 现有轮速动作最终±6rad/s，轮半径约0.0935m，对应理想滚速约0.56m/s；2m/s目标需先重新验证执行能力。没有证据要求拆分两个网络，当前先保持同一策略的低速越障能力。

本次没有写入 v1 配置。检查期间检测到 v1 文件被外部修改，修改前平地对照与修改后基线应区分，详见实验报告。

## 本次验证范围

项目回归测试70项通过；12组固定难度真实GPU评估共2304首回合已完成；所选model_6000完成64环境、2次PPO更新的短程验证，82/89输入、学习率、focus采样、归一化历史和源文件校验记录正确。见 `reports/terrain_transfer_20261020/ppo_smoke_verification.json`。正式2000轮训练尚未执行，不能宣称改动已经提高越障成功率。
