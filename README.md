# wheeled_train

该目录用于保存 Isaac Lab 训练相关内容，是当前项目的训练研发主模块。
当前实际训练代码位于 `modules/wheeled_train/wheeled_leg`，其中已经包含：

- 自定义 Wheeled 任务注册与环境配置
- `rsl_rl` / `skrl` / `rl_games` 训练与 play 脚本
- `logs/rsl_rl` 训练日志、checkpoint 和 `best/` 最优策略索引

下面只整理当前仓库里已经存在并可直接对照使用的 `rsl_rl` 训练与 play 命令。

## 目录结构

```text
modules/wheeled_train/
├── README.md
└── wheeled_leg/
    ├── source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/
    │   └── __init__.py                         # 任务注册入口
    ├── scripts/rsl_rl/train.py                # RSL-RL 训练脚本
    ├── scripts/rsl_rl/play.py                 # RSL-RL 回放脚本
    ├── scripts/rsl_rl/update_best_policy.py   # 维护 best/model.pt 软链接
    └── logs/rsl_rl/                           # 训练日志与 checkpoint
```

## 已注册任务

当前 `wheeled_leg` 里注册了以下任务对：

| train task | play task | 建议 experiment_name |
| --- | --- | --- |
| `Wheeled-Base-Locked-v0` | `Wheeled-Base-Locked-Play-v0` | `wheeled_base_locked` |
| `Wheeled-Base-Locked-Hard-v0` | `Wheeled-Base-Locked-Hard-Play-v0` | `wheeled_base_locked_hard` |
| `Wheeled-Arm-Free-Locomotion-v0` | `Wheeled-Arm-Free-Locomotion-Play-v0` | `wheeled_arm_free_locomotion` |
| `Wheeled-Arm-Free-Recovery-v0` | `Wheeled-Arm-Free-Recovery-Play-v0` | `wheeled_arm_free_recovery` |
| `Wheeled-Arm-Free-Obstacle-v0` | `Wheeled-Arm-Free-Obstacle-Play-v0` | `wheeled_arm_free_obstacle` |
| `Wheeled-Grasp-Pose-Locomotion-v0` | `Wheeled-Grasp-Pose-Locomotion-Play-v0` | `wheeled_grasp_pose_locomotion` |
| `Wheeled-Cartesian-Grasp-v0` | `Wheeled-Cartesian-Grasp-Play-v0` | `wheeled_cartesian_grasp` |
| `Wheeled-Loaded-Grasp-Obstacle-v0` | `Wheeled-Loaded-Grasp-Obstacle-Play-v0` | `wheeled_loaded_grasp_obstacle` |

## 进入目录与激活环境

以下命令默认在仓库根目录 `/home/moujank/project/wheeled-leg-ws` 下执行。

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
```

如果这个扩展还没安装到当前 Python/conda 环境里，先执行：

```bash
python -m pip install -e source/wheeled_leg
```

## RSL-RL 训练命令

### 训练脚本实际支持的 argparse 参数

`scripts/rsl_rl/train.py` 和 `scripts/rsl_rl/cli_args.py` 中显式定义了这些参数：

```text
--video
--video_length
--video_interval
--num_envs
--task
--agent
--seed
--max_iterations
--distributed
--experiment_name
--run_name
--resume
--load_run
--checkpoint
--logger
--log_project_name
```

另外脚本还会通过 `AppLauncher.add_app_launcher_args(parser)` 注入 Isaac Lab 通用启动参数。
当前工作区没有安装 `isaaclab`，这里无法直接展开其 `--help` 输出；在你的 Isaac Lab 环境里可用下面命令查看完整帮助：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/train.py --help
```

### RSL-RL train 完整命令模板

下面这个模板把脚本里显式定义的全部 argparse 参数都列出来了。布尔开关只有在需要打开时才保留；不需要时直接删掉该参数。

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/train.py \
  --task=<TRAIN_TASK> \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=<EXPERIMENT_NAME> \
  --run_name=<RUN_NAME> \
  --num_envs=<NUM_ENVS> \
  --seed=<SEED> \
  --max_iterations=<MAX_ITERATIONS> \
  --checkpoint=<CHECKPOINT_FILE> \
  --load_run=<LOAD_RUN_DIR> \
  --logger=<tensorboard|wandb|neptune> \
  --log_project_name=<LOG_PROJECT_NAME> \
  --video \
  --video_length=200 \
  --video_interval=2000 \
  --resume \
  --distributed
```

说明：

- `--task`：训练任务，例如 `Wheeled-Base-Locked-Hard-v0`
- `--agent`：默认就是 `rsl_rl_cfg_entry_point`；`Wheeled-Base-Locked-Hard-v0` 虽然注册时会自动切换到底层 hard 配置，但 CLI 这里仍然沿用默认值
- `--experiment_name`：建议显式指定为日志目录名，便于和 `logs/rsl_rl/<experiment>/best/` 对齐
- `--run_name`：会追加到时间戳目录名后面
- `--checkpoint`、`--load_run`、`--resume`：仅断点续训时需要
- `--video`、`--video_length`、`--video_interval`：仅训练中录视频时需要
- `--distributed`：仅多卡/多节点训练时需要

### 可直接使用的训练命令示例

#### 1. Base Locked

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/train.py \
  --task=Wheeled-Base-Locked-v0 \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_base_locked \
  --run_name=baseline \
  --num_envs=4096 \
  --seed=42 \
  --max_iterations=6000
```

#### 2. Base Locked Hard

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/train.py \
  --task=Wheeled-Base-Locked-Hard-v0 \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_base_locked_hard \
  --run_name=baseline \
  --num_envs=4096 \
  --seed=42 \
  --max_iterations=4000
```

#### 3. 其他任务

把 `<TRAIN_TASK>` 和 `<EXPERIMENT_NAME>` 替换成上面任务表中的对应值即可：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/train.py \
  --task=<TRAIN_TASK> \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=<EXPERIMENT_NAME> \
  --run_name=baseline \
  --num_envs=<NUM_ENVS> \
  --seed=42 \
  --max_iterations=<MAX_ITERATIONS>
```

## 如何选择最佳策略

当前 `logs/rsl_rl` 的设计是：

```text
logs/rsl_rl/<experiment>/<timestamp_run_name>/model_<iteration>.pt
logs/rsl_rl/<experiment>/best/model.pt
logs/rsl_rl/<experiment>/best/notes.md
```

也就是说：

- 原始训练输出保留在每个时间戳 run 目录里
- 人工选出来的“当前最佳策略”统一通过 `best/model.pt` 软链接暴露
- 选择理由、来源 run、来源 checkpoint 记录在 `best/notes.md`

### 当前日志目录里的实际状态

当前仓库里只有下面两个 experiment 已经真正建立了 `best/model.pt`：

```text
logs/rsl_rl/wheeled_base_locked/best/model.pt
  -> ../../cartpole_direct/2026-06-30_12-40-10/model_5350.pt

logs/rsl_rl/wheeled_base_locked_hard/best/model.pt
  -> ../2026-06-30_16-10-31/model_3000.pt
```

其余 experiment 当前只有 `best/notes.md` 占位说明，还没有 `best/model.pt`。

### 选择最佳策略时看什么

结合当前已有 `notes.md`，以及常规 RSL-RL 训练筛选流程，建议用下面标准：

- `Wheeled-Base-Locked`：优先看高平均奖励、整段 episode 稳定、低 base contact、低速度跟踪误差
- `Wheeled-Base-Locked-Hard`：优先看 rough terrain 上是否稳定、坏接触是否更少、速度跟踪是否更稳
- `Wheeled-Arm-Free-Locomotion`：优先看全身稳定 locomotion、低 base/arm 非期望接触、手臂摆动平滑
- `Wheeled-Arm-Free-Recovery`：优先看从跌倒/扰动状态恢复是否快、最终姿态是否稳、动作抖动是否低
- `Wheeled-Arm-Free-Obstacle`：优先看越障成功率、跌倒率、机身姿态和速度控制
- `Wheeled-Grasp-Pose-Locomotion`：优先看 locomotion 稳定且 arm 能维持 grasp-ready posture
- `Wheeled-Cartesian-Grasp`：优先看末端位置误差、底盘姿态稳定性、手臂动作平滑性
- `Wheeled-Loaded-Grasp-Obstacle`：优先看负载/抓持姿态下的稳定越障和低非期望接触

实际操作上，通常是：

1. 先看 TensorBoard 曲线，筛掉明显未收敛或后期退化的 checkpoint。
2. 在候选 checkpoint 中用 `play.py` 回放。
3. 以实际行为稳定性为最终准则，不只看单一 reward 数值。
4. 确认后，把该 checkpoint 提升为 `best/model.pt`。

## 将某个 checkpoint 设为 best

用 `scripts/rsl_rl/update_best_policy.py` 维护 `best/` 索引：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/update_best_policy.py \
  --experiment=<EXPERIMENT_NAME> \
  --run=<TIMESTAMP_RUN_DIR> \
  --checkpoint=model_<ITERATION>.pt \
  --task=<TRAIN_TASK> \
  --play-task=<PLAY_TASK> \
  --comment="<WHY_THIS_IS_BEST>"
```

示例：把 `wheeled_base_locked_hard` 的 `model_3000.pt` 标成当前最优策略：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/update_best_policy.py \
  --experiment=wheeled_base_locked_hard \
  --run=2026-06-30_16-10-31 \
  --checkpoint=model_3000.pt \
  --task=Wheeled-Base-Locked-Hard-v0 \
  --play-task=Wheeled-Base-Locked-Hard-Play-v0 \
  --comment="Selected after play test: stable on rough terrain, fewer bad contacts, better velocity tracking."
```

执行后会更新：

```text
logs/rsl_rl/wheeled_base_locked_hard/best/model.pt
logs/rsl_rl/wheeled_base_locked_hard/best/notes.md
```

## RSL-RL play 命令

### play 脚本实际支持的 argparse 参数

`scripts/rsl_rl/play.py` 和 `scripts/rsl_rl/cli_args.py` 中显式定义了这些参数：

```text
--video
--video_length
--disable_fabric
--num_envs
--task
--agent
--seed
--use_pretrained_checkpoint
--real-time
--experiment_name
--run_name
--resume
--load_run
--checkpoint
--logger
--log_project_name
```

同样，Isaac Lab 通用启动参数需要在激活环境后查看：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/play.py --help
```

### RSL-RL play 完整命令模板

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/play.py \
  --task=<PLAY_TASK> \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=<EXPERIMENT_NAME> \
  --run_name=<RUN_NAME> \
  --num_envs=<NUM_ENVS> \
  --seed=<SEED> \
  --load_run=<LOAD_RUN_DIR> \
  --checkpoint=<CHECKPOINT_FILE_OR_PATH> \
  --logger=<tensorboard|wandb|neptune> \
  --log_project_name=<LOG_PROJECT_NAME> \
  --video \
  --video_length=200 \
  --disable_fabric \
  --use_pretrained_checkpoint \
  --real-time \
  --resume
```

说明：

- 最常用的是 `--experiment_name` + `--load_run` + `--checkpoint`
- 如果 `--checkpoint` 给的是绝对路径，脚本会直接加载
- 如果 `--checkpoint=model.pt` 且 `--load_run=best`，就会走 `best/model.pt`
- `--use_pretrained_checkpoint` 仅用于加载官方预训练权重，不走本地 `logs/rsl_rl`

### 直接 play 当前 best 策略

#### 1. Base Locked 当前 best

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/play.py \
  --task=Wheeled-Base-Locked-Play-v0 \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_base_locked \
  --load_run=best \
  --checkpoint=model.pt
```

#### 2. Base Locked Hard 当前 best

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/play.py \
  --task=Wheeled-Base-Locked-Hard-Play-v0 \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_base_locked_hard \
  --load_run=best \
  --checkpoint=model.pt
```

### play 指定 run 的指定 checkpoint

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/play.py \
  --task=Wheeled-Base-Locked-Hard-Play-v0 \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_base_locked_hard \
  --load_run=2026-06-30_16-10-31 \
  --checkpoint=model_3000.pt
```

### play 其他任务的 best 策略

如果对应 experiment 已经建立 `best/model.pt`，统一使用下面模式：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
conda activate <your_isaaclab_conda_env>
python scripts/rsl_rl/play.py \
  --task=<PLAY_TASK> \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=<EXPERIMENT_NAME> \
  --load_run=best \
  --checkpoint=model.pt
```

例如：

```bash
python scripts/rsl_rl/play.py \
  --task=Wheeled-Arm-Free-Locomotion-Play-v0 \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_arm_free_locomotion \
  --load_run=best \
  --checkpoint=model.pt
```

前提是 `logs/rsl_rl/wheeled_arm_free_locomotion/best/model.pt` 已经存在；当前仓库里这个文件还没有建立。
