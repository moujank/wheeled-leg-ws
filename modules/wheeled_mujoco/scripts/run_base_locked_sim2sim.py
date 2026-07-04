#!/usr/bin/env python3
"""Command entry point for the base-locked MuJoCo sim2sim rollout."""

from __future__ import annotations

import sys
from pathlib import Path


MODULE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = MODULE_ROOT / "source"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheeled_mujoco.sim2sim import build_base_locked_parser, run_base_locked_from_args  # noqa: E402


def main() -> None:
    parser = build_base_locked_parser()
    args = parser.parse_args()
    run_base_locked_from_args(args)


if __name__ == "__main__":
    main()
