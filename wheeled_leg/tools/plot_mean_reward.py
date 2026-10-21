#!/usr/bin/env python3
"""Post-process RSL-RL TensorBoard logs and plot mean reward curves.

以下示例从 modules/wheeled_train/wheeled_leg 执行；--run-dir 为显式日志路径。

完整运行示例:
    1. 按任务名处理该任务最新一次训练，并生成 mean reward 曲线图:
       python3 tools/plot_mean_reward.py --task wheeled_grasp_pose_locomotion

    2. 按任务名处理，并对曲线做平滑:
       python3 tools/plot_mean_reward.py --task wheeled_grasp_pose_locomotion --smoothing 20

    3. 直接指定某一次训练的日志目录:
       python3 tools/plot_mean_reward.py \
           --run-dir logs/rsl_rl/wheeled_grasp_pose_locomotion/2026-07-04_16-40-52

    4. 查看某次训练里有哪些可用的 TensorBoard 标量标签:
       python3 tools/plot_mean_reward.py --task wheeled_grasp_pose_locomotion --list-tags

参数说明:
    --task
        任务名。对应 logs/rsl_rl 下的任务目录名。
        例如 wheeled_grasp_pose_locomotion。
        提供这个参数后，脚本会自动选择该任务最新一次训练记录。

    --run-dir
        某一次训练的完整日志目录路径。
        当你想精确指定某个 run，而不是自动选最新 run 时使用。

    --log-root
        训练日志根目录。
        默认指向:
        modules/wheeled_train/wheeled_leg/logs/rsl_rl

    --tag
        要读取并绘图的 TensorBoard 标量标签。
        默认优先读取 Train/mean_reward。新版 RSL-RL 通常只记录
        Episode_Reward/*，此时脚本会自动把所有 Episode_Reward 项按训练
        step 相加，生成等价的 Episode_Reward/total 曲线。
        如果你想画别的指标，可以先用 --list-tags 查看可选标签。

    --smoothing
        曲线平滑窗口大小，使用滑动平均。
        设为 1 表示不平滑。
        例如 20 表示使用 20 个点的滑动平均。

    --output-name
        输出图片文件名。
        图片会被保存到对应训练目录下的 plots/ 子目录中。
        默认文件名为 mean_reward.png。

    --list-tags
        只列出当前训练日志中包含的所有标量标签，不生成图片。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing import event_accumulator


TRAIN_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_ROOT = TRAIN_ROOT / "logs" / "rsl_rl"
DEFAULT_TAG = "Train/mean_reward"
DERIVED_TOTAL_TAG = "Episode_Reward/total"


@dataclass(frozen=True)
class ScalarSeries:
    tag: str
    steps: list[int]
    values: list[float]
    wall_times: list[float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot mean reward from a completed training run. "
            "By default, the script finds the latest run under the given task."
        )
    )
    parser.add_argument("--task", type=str, help="Experiment directory name, or a supported wheeled-leg/arm Gym ID.")
    parser.add_argument("--run-dir", type=Path, help="Explicit run directory containing TensorBoard event files.")
    parser.add_argument(
        "--log-root",
        type=Path,
        default=DEFAULT_LOG_ROOT,
        help=f"Root directory that stores RSL-RL logs. Default: {DEFAULT_LOG_ROOT}",
    )
    parser.add_argument("--tag", type=str, default=DEFAULT_TAG, help=f"Scalar tag to plot. Default: {DEFAULT_TAG}")
    parser.add_argument(
        "--smoothing",
        type=int,
        default=1,
        help="Moving-average window for the plotted reward curve. Use 1 to disable smoothing.",
    )
    parser.add_argument(
        "--output-name",
        type=str,
        default="mean_reward.png",
        help="Output image filename stored under <run-dir>/plots.",
    )
    parser.add_argument(
        "--list-tags",
        action="store_true",
        help="List scalar tags from the selected run and exit.",
    )
    parser.add_argument("--start-step", type=int, default=0, help="First training iteration to include.")
    parser.add_argument("--end-step", type=int, help="Last training iteration to include.")
    parser.add_argument("--output", type=Path, help="Explicit image path; overrides --output-name.")
    parser.add_argument("--csv-output", type=Path, help="Export raw and smoothed scalar data to this CSV.")
    args = parser.parse_args()
    if args.smoothing < 1:
        parser.error("smoothing must be at least 1.")
    if args.start_step < 0 or (args.end_step is not None and args.end_step < args.start_step):
        parser.error("Step range must satisfy 0 <= start-step <= end-step.")
    if Path(args.output_name).name != args.output_name or args.output_name in {"", ".", ".."}:
        parser.error("output-name must be a filename; use --output for a custom path.")
    return args


def resolve_run_dir(log_root: Path, task: str | None, run_dir: Path | None) -> Path:
    if run_dir is not None:
        resolved = run_dir.expanduser().resolve()
        if not resolved.is_dir():
            raise FileNotFoundError(f"Run directory does not exist: {resolved}")
        return resolved

    if not task:
        raise ValueError("Either --task or --run-dir must be provided.")

    # Train and PLAY IDs share an experiment; custom experiment names still work.
    task = {
        "Wheeled-Arm-v1": "wheeled_arm_v1",
        "Wheeled-Arm-Play-v1": "wheeled_arm_v1",
        "Wheeled-Leg-Obstacle-Jump": "wheeled_leg_obstacle_jump",
        "Wheeled-Leg-Obstacle-Jump-Play": "wheeled_leg_obstacle_jump",
        "Wheeled-Leg-Jump": "wheeled_leg_jump",
        "Wheeled-Leg-Jump-Play": "wheeled_leg_jump",
    }.get(task, task)
    task_dir = (log_root / task).resolve()
    if not task_dir.is_dir():
        raise FileNotFoundError(f"Task log directory does not exist: {task_dir}")

    run_dirs = sorted(
        [
            path for path in task_dir.iterdir()
            if path.is_dir() and path.name != "best" and any(path.glob("events.out.tfevents.*"))
        ],
        key=lambda path: path.name,
    )
    if not run_dirs:
        raise FileNotFoundError(f"No training runs found under task directory: {task_dir}")
    return run_dirs[-1]


def find_event_file(run_dir: Path) -> Path:
    event_files = sorted(run_dir.glob("events.out.tfevents.*"))
    if not event_files:
        raise FileNotFoundError(f"No TensorBoard event file found in: {run_dir}")
    return event_files[-1]


def load_event_accumulator(event_file: Path) -> event_accumulator.EventAccumulator:
    accumulator = event_accumulator.EventAccumulator(
        str(event_file),
        size_guidance={
            event_accumulator.SCALARS: 0,
            event_accumulator.TENSORS: 0,
        },
    )
    accumulator.Reload()
    return accumulator


def available_scalar_tags(accumulator: event_accumulator.EventAccumulator) -> list[str]:
    return list(accumulator.Tags().get("scalars", []))


def load_scalar_series(accumulator: event_accumulator.EventAccumulator, tag: str) -> ScalarSeries:
    scalar_tags = available_scalar_tags(accumulator)
    if tag not in scalar_tags:
        raise KeyError(
            f"Scalar tag '{tag}' not found. Available scalar tags: {', '.join(scalar_tags) if scalar_tags else '(none)'}"
        )
    # A resumed run can contain several event files and repeated iterations.
    # Keep the newest wall-time value per step, then restore step order.
    latest_by_step = {}
    for event in accumulator.Scalars(tag):
        if not math.isfinite(event.value) or not math.isfinite(event.wall_time):
            raise ValueError(f"Non-finite scalar for tag '{tag}' at step {event.step}.")
        previous = latest_by_step.get(event.step)
        if previous is None or event.wall_time >= previous.wall_time:
            latest_by_step[event.step] = event
    events = [latest_by_step[step] for step in sorted(latest_by_step)]
    return ScalarSeries(
        tag=tag,
        steps=[int(event.step) for event in events],
        values=[float(event.value) for event in events],
        wall_times=[float(event.wall_time) for event in events],
    )


def aggregate_episode_rewards(accumulator: event_accumulator.EventAccumulator) -> ScalarSeries:
    """Build a total episode reward from RSL-RL's per-term scalar tags.

    Current RSL-RL writes ``Episode_Reward/<term>`` values instead of the old
    ``Train/mean_reward`` scalar.  All terms are emitted at the same training
    iterations, but using the intersection of steps also makes resumed runs
    safe when one term starts a little later than the others.
    """
    reward_tags = sorted(tag for tag in available_scalar_tags(accumulator) if tag.startswith("Episode_Reward/"))
    if not reward_tags:
        raise KeyError(
            f"Scalar tag '{DEFAULT_TAG}' not found and no Episode_Reward/* tags are available. "
            f"Available scalar tags: {', '.join(available_scalar_tags(accumulator)) or '(none)'}"
        )

    series = [load_scalar_series(accumulator, tag) for tag in reward_tags]
    by_step = [{step: value for step, value in zip(item.steps, item.values, strict=True)} for item in series]
    common_steps = set(by_step[0])
    for values in by_step[1:]:
        common_steps.intersection_update(values)
    steps = sorted(common_steps)
    if not steps:
        raise ValueError("Episode_Reward/* tags do not contain a common training step.")
    return ScalarSeries(
        tag=DERIVED_TOTAL_TAG,
        steps=steps,
        values=[sum(values[step] for values in by_step) for step in steps],
        wall_times=[max(item.wall_times[item.steps.index(step)] for item in series) for step in steps],
    )


def load_requested_series(accumulator: event_accumulator.EventAccumulator, tag: str) -> ScalarSeries:
    """Load an explicit tag, or transparently derive total reward for old defaults."""
    try:
        return load_scalar_series(accumulator, tag)
    except KeyError:
        if tag not in (DEFAULT_TAG, DERIVED_TOTAL_TAG):
            raise
        print(
            f"[INFO] Scalar tag '{tag}' is absent; summing Episode_Reward/* into "
            f"'{DERIVED_TOTAL_TAG}'."
        )
        return aggregate_episode_rewards(accumulator)


def moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1 or len(values) <= 1:
        return values[:]

    smoothed: list[float] = []
    running_sum = 0.0
    for index, value in enumerate(values):
        running_sum += value
        if index >= window:
            running_sum -= values[index - window]
        divisor = min(index + 1, window)
        smoothed.append(running_sum / divisor)
    return smoothed


def save_plot(
    run_dir: Path, task_name: str, series: ScalarSeries, smoothing: int, output_name: str,
    output_path: Path | None = None,
) -> Path:
    output_path = output_path or run_dir / "plots" / output_name
    output_path.parent.mkdir(parents=True, exist_ok=True)

    smoothed_values = moving_average(series.values, smoothing)

    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=140)
    if smoothing > 1:
        ax.plot(series.steps, series.values, color="#9aa5b1", linewidth=1.1, alpha=0.5, label="raw")
        ax.plot(series.steps, smoothed_values, color="#005f73", linewidth=2.0, label=f"MA({smoothing})")
    else:
        ax.plot(series.steps, series.values, color="#005f73", linewidth=2.0, label=series.tag)

    ax.set_title(f"{task_name}: {series.tag}")
    ax.set_xlabel("Training iteration")
    ax.set_ylabel(series.tag)
    ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.4)
    ax.legend()

    if series.values:
        final_value = smoothed_values[-1]
        ax.annotate(
            f"{final_value:.3f}",
            xy=(series.steps[-1], final_value),
            xytext=(8, 8),
            textcoords="offset points",
            fontsize=9,
        )

    # Mark the top-5 largest points on the displayed curve so peak behavior is visible in the output image.
    ranked_points = sorted(
        zip(series.steps, smoothed_values),
        key=lambda item: item[1],
        reverse=True,
    )
    annotated_steps: set[int] = set()
    top_points: list[tuple[int, float]] = []
    for step, value in ranked_points:
        if step in annotated_steps:
            continue
        annotated_steps.add(step)
        top_points.append((step, value))
        if len(top_points) == 5:
            break

    for index, (step, value) in enumerate(top_points, start=1):
        ax.scatter(step, value, color="#bb3e03", s=28, zorder=3)
        ax.annotate(
            f"Top {index}: ({step}, {value:.3f})",
            xy=(step, value),
            xytext=(10, 12 + (index % 2) * 14),
            textcoords="offset points",
            fontsize=8.5,
            color="#9b2226",
            bbox={"boxstyle": "round,pad=0.2", "fc": "white", "ec": "#ee9b00", "alpha": 0.85},
            arrowprops={"arrowstyle": "->", "color": "#ca6702", "lw": 0.9},
        )

    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def main() -> int:
    args = parse_args()

    try:
        run_dir = resolve_run_dir(args.log_root, args.task, args.run_dir)
        event_file = find_event_file(run_dir)
        # Reading the directory includes all event files from resumed training.
        accumulator = load_event_accumulator(run_dir)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    if args.list_tags:
        for tag in available_scalar_tags(accumulator):
            print(tag)
        if any(tag.startswith("Episode_Reward/") for tag in available_scalar_tags(accumulator)):
            print(DERIVED_TOTAL_TAG + " (derived from Episode_Reward/*)")
        return 0

    try:
        series = load_requested_series(accumulator, args.tag)
        selected = [
            index for index, step in enumerate(series.steps)
            if step >= args.start_step and (args.end_step is None or step <= args.end_step)
        ]
        if not selected:
            raise ValueError("No scalar samples matched the requested step range.")
        series = ScalarSeries(
            tag=series.tag,
            steps=[series.steps[index] for index in selected],
            values=[series.values[index] for index in selected],
            wall_times=[series.wall_times[index] for index in selected],
        )
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    task_name = run_dir.parent.name
    try:
        output_path = save_plot(
            run_dir, task_name, series, args.smoothing, args.output_name,
            args.output.expanduser().resolve() if args.output else None,
        )
        if args.csv_output:
            csv_path = args.csv_output.expanduser().resolve()
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with csv_path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["step", "wall_time_s", "tag", "raw_value", "smoothed_value"])
                writer.writerows(zip(
                    series.steps, series.wall_times, [series.tag] * len(series.steps),
                    series.values, moving_average(series.values, args.smoothing), strict=True,
                ))
            print(f"[INFO] Output CSV: {csv_path}")
    except Exception as exc:
        print(f"[ERROR] Failed to save plot: {exc}", file=sys.stderr)
        return 1

    print(f"[INFO] Run directory: {run_dir}")
    print(f"[INFO] Event files loaded: {len(list(run_dir.glob('events.out.tfevents.*')))} (latest: {event_file.name})")
    print(f"[INFO] Scalar tag: {series.tag}")
    print(f"[INFO] Points plotted: {len(series.values)}")
    print(f"[INFO] Output image: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
