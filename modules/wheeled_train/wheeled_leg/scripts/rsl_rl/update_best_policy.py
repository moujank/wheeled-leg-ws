#!/usr/bin/env python3

"""Update the best-policy index for an RSL-RL experiment.

This script creates or updates:
- logs/rsl_rl/<experiment>/best/model.pt
- logs/rsl_rl/<experiment>/best/notes.md

It keeps the original training run untouched and stores a lightweight symlink to the selected checkpoint.
"""

from __future__ import annotations

import argparse
import os
from datetime import datetime
from pathlib import Path


def _relative_symlink_target(source: Path, link_path: Path) -> str:
    return os.path.relpath(source.resolve(), start=link_path.parent.resolve())


def _write_notes(
    notes_path: Path,
    *,
    experiment: str,
    run: str,
    checkpoint: str,
    task: str | None,
    play_task: str | None,
    comment: str | None,
) -> None:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    checkpoint_path = f"../{run}/{checkpoint}"

    play_command = ""
    if play_task:
        play_command = (
            "python scripts/rsl_rl/play.py "
            f"--task={play_task} "
            f"--experiment_name={experiment} "
            "--load_run=best "
            "--checkpoint=model.pt"
        )

    train_command = ""
    if task:
        train_command = (
            "python scripts/rsl_rl/train.py "
            f"--task={task} "
            f"--experiment_name={experiment} "
            f"--run_name=<run_name>"
        )

    notes = [
        f"# {experiment} best policy",
        "",
        f"updated_at: {now}",
        f"source_run: {run}",
        f"source_checkpoint: {checkpoint}",
        f"best_model: {checkpoint_path}",
        f"task: {task or 'TBD'}",
        f"play_task: {play_task or 'TBD'}",
        f"status: {'selected' if checkpoint != 'TBD' else 'placeholder'}",
        "",
        "## Why This Checkpoint",
        "",
        comment or "TBD",
        "",
        "## Play",
        "",
        "```bash",
        play_command or "TBD",
        "```",
        "",
        "## Train",
        "",
        "```bash",
        train_command or "TBD",
        "```",
        "",
    ]
    notes_path.write_text("\n".join(notes), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Promote an RSL-RL checkpoint to the best-policy index.")
    parser.add_argument("--experiment", required=True, help="Experiment folder under logs/rsl_rl.")
    parser.add_argument("--run", required=True, help="Run folder under logs/rsl_rl/<experiment>.")
    parser.add_argument("--checkpoint", required=True, help="Checkpoint filename, for example model_5350.pt.")
    parser.add_argument("--task", default=None, help="Training task name for documentation.")
    parser.add_argument("--play-task", default=None, help="Play task name for documentation.")
    parser.add_argument("--comment", default=None, help="Short reason for selecting this checkpoint.")
    parser.add_argument(
        "--logs-root",
        default="logs/rsl_rl",
        help="RSL-RL logs root relative to the current project directory.",
    )
    args = parser.parse_args()

    logs_root = Path(args.logs_root)
    experiment_dir = logs_root / args.experiment
    run_dir = experiment_dir / args.run
    checkpoint_path = run_dir / args.checkpoint

    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    best_dir = experiment_dir / "best"
    best_dir.mkdir(parents=True, exist_ok=True)

    best_model = best_dir / "model.pt"
    if best_model.exists() or best_model.is_symlink():
        best_model.unlink()
    best_model.symlink_to(_relative_symlink_target(checkpoint_path, best_model))

    _write_notes(
        best_dir / "notes.md",
        experiment=args.experiment,
        run=args.run,
        checkpoint=args.checkpoint,
        task=args.task,
        play_task=args.play_task,
        comment=args.comment,
    )

    print(f"[INFO] Updated best model: {best_model} -> {os.readlink(best_model)}")
    print(f"[INFO] Updated notes: {best_dir / 'notes.md'}")


if __name__ == "__main__":
    main()
