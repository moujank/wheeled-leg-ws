#!/usr/bin/env python3
"""Post-process RSL-RL TensorBoard logs and plot mean reward curves.

完整运行示例:
    1. 按任务名处理该任务最新一次训练，并生成 mean reward 曲线图:
       python3 scripts/plot_mean_reward.py --task wheeled_grasp_pose_locomotion

    2. 按任务名处理，并对曲线做平滑:
       python3 scripts/plot_mean_reward.py --task wheeled_grasp_pose_locomotion --smoothing 20

    3. 直接指定某一次训练的日志目录:
       python3 scripts/plot_mean_reward.py \
           --run-dir modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_grasp_pose_locomotion/2026-07-04_16-40-52

    4. 查看某次训练里有哪些可用的 TensorBoard 标量标签:
       python3 scripts/plot_mean_reward.py --task wheeled_grasp_pose_locomotion --list-tags

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
        默认是 Train/mean_reward。
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
import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing import event_accumulator


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_ROOT = REPO_ROOT / "modules" / "wheeled_train" / "wheeled_leg" / "logs" / "rsl_rl"
DEFAULT_TAG = "Train/mean_reward"


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
    parser.add_argument("--task", type=str, help="Task / experiment directory name under logs/rsl_rl.")
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
    return parser.parse_args()


def resolve_run_dir(log_root: Path, task: str | None, run_dir: Path | None) -> Path:
    if run_dir is not None:
        resolved = run_dir.expanduser().resolve()
        if not resolved.is_dir():
            raise FileNotFoundError(f"Run directory does not exist: {resolved}")
        return resolved

    if not task:
        raise ValueError("Either --task or --run-dir must be provided.")

    task_dir = (log_root / task).resolve()
    if not task_dir.is_dir():
        raise FileNotFoundError(f"Task log directory does not exist: {task_dir}")

    run_dirs = sorted(
        [path for path in task_dir.iterdir() if path.is_dir() and path.name != "best"],
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
    events = accumulator.Scalars(tag)
    return ScalarSeries(
        tag=tag,
        steps=[int(event.step) for event in events],
        values=[float(event.value) for event in events],
        wall_times=[float(event.wall_time) for event in events],
    )


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


def save_plot(run_dir: Path, task_name: str, series: ScalarSeries, smoothing: int, output_name: str) -> Path:
    output_dir = run_dir / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / output_name

    smoothed_values = moving_average(series.values, smoothing)

    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=140)
    if smoothing > 1:
        ax.plot(series.steps, series.values, color="#9aa5b1", linewidth=1.1, alpha=0.5, label="raw")
        ax.plot(series.steps, smoothed_values, color="#005f73", linewidth=2.0, label=f"MA({smoothing})")
    else:
        ax.plot(series.steps, series.values, color="#005f73", linewidth=2.0, label="mean reward")

    ax.set_title(f"{task_name} mean reward")
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
        accumulator = load_event_accumulator(event_file)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    if args.list_tags:
        for tag in available_scalar_tags(accumulator):
            print(tag)
        return 0

    try:
        series = load_scalar_series(accumulator, args.tag)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    task_name = run_dir.parent.name
    try:
        output_path = save_plot(run_dir, task_name, series, args.smoothing, args.output_name)
    except Exception as exc:
        print(f"[ERROR] Failed to save plot: {exc}", file=sys.stderr)
        return 1

    print(f"[INFO] Run directory: {run_dir}")
    print(f"[INFO] Event file: {event_file}")
    print(f"[INFO] Scalar tag: {series.tag}")
    print(f"[INFO] Points plotted: {len(series.values)}")
    print(f"[INFO] Output image: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
