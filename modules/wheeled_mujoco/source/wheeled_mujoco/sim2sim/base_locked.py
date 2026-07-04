"""Source-layer adapter for the validated base-locked MuJoCo sim2sim runner."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from argparse import Namespace
from pathlib import Path
from types import ModuleType

from wheeled_mujoco.paths import MODULE_ROOT


_LEGACY_MODULE_NAME = "_wheeled_mujoco_minimal_base_locked"
_LEGACY_RUNNER_PATH = MODULE_ROOT / "minimal_sim2sim" / "run_base_locked.py"


def load_legacy_base_locked_module() -> ModuleType:
    """Load the existing debug baseline without moving or editing it."""
    if _LEGACY_MODULE_NAME in sys.modules:
        return sys.modules[_LEGACY_MODULE_NAME]

    spec = importlib.util.spec_from_file_location(_LEGACY_MODULE_NAME, _LEGACY_RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load base_locked runner from {_LEGACY_RUNNER_PATH}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[_LEGACY_MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module


def build_base_locked_parser() -> argparse.ArgumentParser:
    """Build a full CLI parser that preserves the current base-locked defaults."""
    legacy = load_legacy_base_locked_module()

    parser = argparse.ArgumentParser(description="MuJoCo sim2sim for wheeled_base_locked")
    parser.add_argument(
        "--mode",
        choices=["isaac_play", "experimental"],
        default="isaac_play",
        help="Run mode. `isaac_play` aligns reset/command semantics with Isaac Play; `experimental` keeps custom tether/release logic.",
    )
    parser.add_argument("--model", type=Path, default=legacy.DEFAULT_MODEL, help="Path to wheeled_leg.xml")
    parser.add_argument(
        "--policy",
        type=Path,
        default=legacy.DEFAULT_PT_POLICY,
        help="Path to exported policy (.pt or .onnx)",
    )
    parser.add_argument(
        "--backend",
        choices=["torch", "onnx"],
        default="torch",
        help="Policy backend. Default uses policy.pt.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Simulation duration in seconds. Default is to run forever until the viewer/script is closed.",
    )
    parser.add_argument("--cmd-vx", type=float, default=0.5, help="Forward velocity command")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="Lateral velocity command")
    parser.add_argument("--cmd-wz", type=float, default=0.0, help="Yaw rate command")
    parser.add_argument(
        "--leg-action-scale",
        type=float,
        default=0.5,
        help="Leg joint position action scale. Isaac training uses 0.5.",
    )
    parser.add_argument(
        "--wheel-action-scale",
        type=float,
        default=10.0,
        help="Wheel joint velocity action scale. Isaac base_locked training uses 10.0.",
    )
    parser.add_argument(
        "--log-path",
        type=Path,
        default=None,
        help="Optional path to write 1 Hz diagnostic JSONL logs for MuJoCo/Isaac alignment checks.",
    )
    parser.add_argument(
        "--verbose-obs",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Print the 27D policy observation breakdown at policy handoff.",
    )
    parser.add_argument(
        "--handoff-window",
        type=float,
        default=0.5,
        help="Time window in seconds before/after policy handoff for per-step high-frequency logging.",
    )
    parser.add_argument(
        "--handoff-log-every-step",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Emit high-frequency console and JSONL logs around the policy handoff window.",
    )
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=0.5,
        help="Viewer playback speed multiplier. 0.5 means half-speed visualization without changing sim dt.",
    )
    parser.add_argument(
        "--startup-tether",
        type=float,
        default=1.0,
        help="Keep the robot stabilized by a virtual safety tether for this many seconds at startup/reset.",
    )
    parser.add_argument(
        "--tether-release",
        type=float,
        default=1.0,
        help="Duration of the smooth tether release phase after startup tethering.",
    )
    parser.add_argument(
        "--startup-stand",
        type=float,
        default=2.0,
        help="Run a stand phase for this many seconds after tether release and before locomotion commands.",
    )
    parser.add_argument(
        "--stand-strategy",
        choices=["pose_hold", "policy_zero"],
        default="policy_zero",
        help="Stand-phase strategy: hold a fixed pose, or run the policy with zero velocity command.",
    )
    parser.add_argument(
        "--deterministic-startup-reset",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use a deterministic zero-velocity startup reset instead of Isaac Play random startup pose/velocity.",
    )
    parser.add_argument(
        "--apply-soft-start-in-isaac-play",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Apply command ramp and action soft-start even in isaac_play mode to improve deployment stability.",
    )
    parser.add_argument(
        "--isaac-play-cmd-limit-early",
        type=float,
        default=0.10,
        help="Early command fraction used at the start of isaac_play command ramp.",
    )
    parser.add_argument(
        "--enable-wheel-balance-feedback",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Add a thin wheel feedback stabilizer in policy phase for pitch/yaw correction.",
    )
    parser.add_argument("--pitch-feedback-kp", type=float, default=2.0, help="Wheel common-mode pitch feedback gain.")
    parser.add_argument(
        "--pitch-rate-feedback-kd",
        type=float,
        default=0.35,
        help="Wheel common-mode pitch-rate feedback gain.",
    )
    parser.add_argument("--yaw-feedback-kp", type=float, default=0.80, help="Wheel differential yaw feedback gain.")
    parser.add_argument("--yaw-rate-feedback-kd", type=float, default=0.12, help="Wheel differential yaw-rate feedback gain.")
    parser.add_argument("--wheel-feedback-limit", type=float, default=3.0, help="Absolute limit for wheel feedback correction.")
    parser.add_argument(
        "--pitch-cmd-attenuation-start-deg",
        type=float,
        default=4.0,
        help="Start attenuating forward command when body pitch exceeds this value.",
    )
    parser.add_argument(
        "--pitch-cmd-attenuation-full-deg",
        type=float,
        default=6.0,
        help="Fully suppress forward command when body pitch exceeds this value.",
    )
    parser.add_argument(
        "--yaw-cmd-attenuation-start-deg",
        type=float,
        default=8.0,
        help="Start attenuating forward command when heading error exceeds this value.",
    )
    parser.add_argument(
        "--yaw-cmd-attenuation-full-deg",
        type=float,
        default=25.0,
        help="Fully suppress forward command when heading error exceeds this value.",
    )
    parser.add_argument(
        "--attenuate-linear-cmd-on-heading-error",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Whether heading error should attenuate body-frame linear velocity commands.",
    )
    parser.add_argument(
        "--heading-hold-enabled",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Inject a bounded heading-hold yaw-rate command during policy running when --cmd-wz is zero.",
    )
    parser.add_argument("--heading-hold-kp", type=float, default=0.7, help="Heading-hold yaw error gain.")
    parser.add_argument("--heading-hold-rate-kd", type=float, default=0.10, help="Heading-hold yaw-rate damping.")
    parser.add_argument(
        "--heading-hold-wz-limit",
        type=float,
        default=0.35,
        help="Absolute cap for the injected heading-hold yaw-rate command.",
    )
    parser.add_argument(
        "--wheel-cmd-attenuation-start",
        type=float,
        default=1.5,
        help="Start attenuating forward command when average wheel speed exceeds this value.",
    )
    parser.add_argument(
        "--wheel-cmd-attenuation-full",
        type=float,
        default=2.6,
        help="Fully suppress forward command when average wheel speed exceeds this value.",
    )
    parser.add_argument(
        "--high-speed-cmd-start",
        type=float,
        default=0.10,
        help="Requested forward speed above this value is progressively downscaled for safer MuJoCo rollout.",
    )
    parser.add_argument(
        "--high-speed-cmd-gain",
        type=float,
        default=2.4,
        help="Strength of requested forward speed downscaling above --high-speed-cmd-start.",
    )
    parser.add_argument(
        "--wheel-speed-soft-limit",
        type=float,
        default=1.7,
        help="Start applying extra common-mode wheel braking when the mean wheel speed exceeds this value.",
    )
    parser.add_argument(
        "--wheel-speed-brake-gain",
        type=float,
        default=3.0,
        help="Additional common-mode wheel braking gain once wheel speed exceeds the soft limit.",
    )
    parser.add_argument(
        "--yaw-feedback-gain-scale",
        type=float,
        default=3.5,
        help="Scale yaw wheel feedback with commanded forward speed.",
    )
    parser.add_argument(
        "--pitch-feedback-gain-scale",
        type=float,
        default=1.5,
        help="Scale pitch wheel feedback with commanded forward speed.",
    )
    parser.add_argument(
        "--real-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Slow simulation down to wall-clock speed. Enabled by default; use --no-real-time to disable.",
    )
    parser.add_argument(
        "--viewer",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Launch the official MuJoCo 3.4.0 viewer. Enabled by default; use --no-viewer to disable.",
    )
    return parser


def create_base_locked_config(args: Namespace):
    """Create the legacy Sim2SimConfig from parsed CLI args."""
    legacy = load_legacy_base_locked_module()
    return legacy.Sim2SimConfig(
        mode=args.mode,
        duration=args.duration,
        cmd_vx=args.cmd_vx,
        cmd_vy=args.cmd_vy,
        cmd_wz=args.cmd_wz,
        leg_action_scale=args.leg_action_scale,
        wheel_action_scale=args.wheel_action_scale,
        real_time=args.real_time,
        playback_speed=args.playback_speed,
        startup_tether_s=args.startup_tether,
        tether_release_s=args.tether_release,
        startup_stand_s=args.startup_stand,
        stand_strategy=args.stand_strategy,
        deterministic_startup_reset=args.deterministic_startup_reset,
        log_path=args.log_path,
        apply_soft_start_in_isaac_play=args.apply_soft_start_in_isaac_play,
        isaac_play_cmd_limit_early=args.isaac_play_cmd_limit_early,
        enable_wheel_balance_feedback=args.enable_wheel_balance_feedback,
        pitch_feedback_kp=args.pitch_feedback_kp,
        pitch_rate_feedback_kd=args.pitch_rate_feedback_kd,
        yaw_feedback_kp=args.yaw_feedback_kp,
        yaw_rate_feedback_kd=args.yaw_rate_feedback_kd,
        wheel_feedback_limit=args.wheel_feedback_limit,
        pitch_cmd_attenuation_start_deg=args.pitch_cmd_attenuation_start_deg,
        pitch_cmd_attenuation_full_deg=args.pitch_cmd_attenuation_full_deg,
        yaw_cmd_attenuation_start_deg=args.yaw_cmd_attenuation_start_deg,
        yaw_cmd_attenuation_full_deg=args.yaw_cmd_attenuation_full_deg,
        attenuate_linear_cmd_on_heading_error=args.attenuate_linear_cmd_on_heading_error,
        heading_hold_enabled=args.heading_hold_enabled,
        heading_hold_kp=args.heading_hold_kp,
        heading_hold_rate_kd=args.heading_hold_rate_kd,
        heading_hold_wz_limit=args.heading_hold_wz_limit,
        wheel_cmd_attenuation_start=args.wheel_cmd_attenuation_start,
        wheel_cmd_attenuation_full=args.wheel_cmd_attenuation_full,
        high_speed_cmd_start=args.high_speed_cmd_start,
        high_speed_cmd_gain=args.high_speed_cmd_gain,
        wheel_speed_soft_limit=args.wheel_speed_soft_limit,
        wheel_speed_brake_gain=args.wheel_speed_brake_gain,
        yaw_feedback_gain_scale=args.yaw_feedback_gain_scale,
        pitch_feedback_gain_scale=args.pitch_feedback_gain_scale,
        verbose_obs=args.verbose_obs,
        handoff_window_s=args.handoff_window,
        handoff_log_every_step=args.handoff_log_every_step,
    )


def resolve_policy_path(args: Namespace) -> Path:
    """Apply the same backend-specific policy path convention as the baseline."""
    legacy = load_legacy_base_locked_module()
    policy_path = args.policy
    if args.backend == "torch" and policy_path == legacy.DEFAULT_PT_POLICY.with_suffix(".pt"):
        policy_path = legacy.DEFAULT_PT_POLICY
    if args.backend == "onnx" and policy_path == legacy.DEFAULT_PT_POLICY:
        policy_path = legacy.DEFAULT_ONNX_POLICY
    return policy_path


def run_base_locked_from_args(args: Namespace) -> None:
    """Run the base-locked MuJoCo sim2sim rollout from parsed args."""
    legacy = load_legacy_base_locked_module()
    cfg = create_base_locked_config(args)
    runner = legacy.BaseLockedSim2Sim(args.model, resolve_policy_path(args), args.backend, cfg)
    runner.run(use_viewer=args.viewer)
