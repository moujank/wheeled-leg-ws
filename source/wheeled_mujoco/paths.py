"""Shared paths for the MuJoCo-side workspace."""

from __future__ import annotations

from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = MODULE_ROOT.parents[1]
TRAIN_ROOT = REPO_ROOT / "modules" / "wheeled_train" / "wheeled_leg"
ASSET_ROOT = MODULE_ROOT / "asset"
MJCF_ROOT = ASSET_ROOT / "mjcf"
TRAIN_LOG_ROOT = TRAIN_ROOT / "logs" / "rsl_rl"


def policy_path(log_name: str, backend: str = "torch") -> Path:
    """Return the conventional exported policy path for an rsl_rl log folder."""
    suffix = ".pt" if backend == "torch" else ".onnx"
    return TRAIN_LOG_ROOT / log_name / "best" / "exported" / f"policy{suffix}"
