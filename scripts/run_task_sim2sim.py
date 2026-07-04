#!/usr/bin/env python3
"""Task-aware MuJoCo sim2sim dispatcher."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence


MODULE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = MODULE_ROOT / "source"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheeled_mujoco.sim2sim import build_base_locked_parser, run_base_locked_from_args  # noqa: E402
from wheeled_mujoco.tasks import BASE_LOCKED_RUNNER, TASK_REGISTRY, get_task_config  # noqa: E402


def _option_was_provided(args: Sequence[str], option: str) -> bool:
    return any(item == option or item.startswith(f"{option}=") for item in args)


def _build_dispatch_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dispatch a registered wheeled_mujoco task to its MuJoCo sim2sim runner.",
        add_help=True,
        epilog=(
            "All runner-specific arguments are forwarded to the selected sim2sim runner.\n"
            "For wheeled_base_locked this includes --viewer/--no-viewer, --duration, --cmd-vx, and other rollout flags."
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--task", default="wheeled_base_locked", choices=sorted(TASK_REGISTRY), help="Registered task name.")
    parser.add_argument("--list-tasks", action="store_true", help="List registered MuJoCo task configs and exit.")
    parser.add_argument("--dry-run", action="store_true", help="Print the selected task config without running MuJoCo.")
    parser.add_argument(
        "--runner-help",
        action="store_true",
        help="Print the full help of the selected task runner and exit.",
    )
    return parser


def _print_tasks() -> None:
    for name in sorted(TASK_REGISTRY):
        cfg = TASK_REGISTRY[name]
        print(
            f"{name}: runner={cfg.runner_kind} robot={cfg.robot_variant} "
            f"model={cfg.mjcf_path} policy={cfg.policy_path}"
        )


def main(argv: Sequence[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    dispatch_parser = _build_dispatch_parser()
    dispatch_args, remaining = dispatch_parser.parse_known_args(argv)

    if dispatch_args.list_tasks:
        _print_tasks()
        return

    task_cfg = get_task_config(dispatch_args.task)
    if dispatch_args.runner_help:
        if task_cfg.runner_kind != BASE_LOCKED_RUNNER:
            raise SystemExit(
                f"Task '{task_cfg.task_name}' is registered, but runner '{task_cfg.runner_kind}' does not expose a CLI help entry yet."
            )
        build_base_locked_parser().print_help()
        return

    if dispatch_args.dry_run:
        print(task_cfg)
        return

    if task_cfg.runner_kind != BASE_LOCKED_RUNNER:
        raise SystemExit(
            f"Task '{task_cfg.task_name}' is registered, but runner '{task_cfg.runner_kind}' is not implemented yet. "
            "Use --dry-run to inspect the reserved MuJoCo config."
        )

    base_parser = build_base_locked_parser()
    base_args = base_parser.parse_args(remaining)
    if not _option_was_provided(remaining, "--model"):
        base_args.model = task_cfg.mjcf_path
    if not _option_was_provided(remaining, "--policy"):
        base_args.policy = task_cfg.with_backend(base_args.backend).policy_path
    if not _option_was_provided(remaining, "--cmd-vx"):
        base_args.cmd_vx = task_cfg.command.lin_vel_x[0]
    if not _option_was_provided(remaining, "--cmd-vy"):
        base_args.cmd_vy = task_cfg.command.lin_vel_y[0]
    if not _option_was_provided(remaining, "--cmd-wz"):
        base_args.cmd_wz = task_cfg.command.ang_vel_z[0]

    run_base_locked_from_args(base_args)


if __name__ == "__main__":
    main()
