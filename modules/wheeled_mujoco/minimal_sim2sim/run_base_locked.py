#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import mujoco
import numpy as np

try:
    import torch
except ImportError:  # pragma: no cover
    torch = None

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover
    ort = None


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MODEL = REPO_ROOT / "modules" / "wheeled_mujoco" / "asset" / "mjcf" / "wheeled_leg.xml"
DEFAULT_PT_POLICY = (
    REPO_ROOT
    / "modules"
    / "wheeled_train"
    / "wheeled_leg"
    / "logs"
    / "rsl_rl"
    / "wheeled_base_locked"
    / "best"
    / "exported"
    / "policy.pt"
)
DEFAULT_ONNX_POLICY = DEFAULT_PT_POLICY.with_suffix(".onnx")

POLICY_JOINT_ORDER = ["HipL", "HipR", "KneeL", "KneeR", "HubL", "HubR"]
LEG_JOINTS = POLICY_JOINT_ORDER[:4]
WHEEL_JOINTS = POLICY_JOINT_ORDER[4:]
ACTUATOR_ORDER = ["HipL_pos", "HipR_pos", "KneeL_pos", "KneeR_pos", "HubL_vel", "HubR_vel"]
DEFAULT_STAND_JOINT_POS = {
    "HipL": 0.0,
    "HipR": 0.0,
    "KneeL": 0.0,
    "KneeR": 0.0,
    "HubL": 0.0,
    "HubR": 0.0,
}


@dataclass
class Sim2SimConfig:
    mode: str = "isaac_play"
    sim_dt: float = 1.0 / 120.0
    decimation: int = 4
    duration: float | None = None
    cmd_vx: float = 0.5
    cmd_vy: float = 0.0
    cmd_wz: float = 0.0
    leg_action_scale: float = 0.5
    wheel_action_scale: float = 10.0
    action_clip: float = 1.0
    real_time: bool = True
    playback_speed: float = 0.5
    startup_tether_s: float = 1.0
    tether_release_s: float = 1.0
    startup_stand_s: float = 2.0
    stand_strategy: str = "policy_zero"
    deterministic_startup_reset: bool = True
    stand_base_height_target: float = 0.32
    locomotion_ramp_s: float = 4.0
    tether_height_target: float = 0.32
    release_leg_action_scale: float = 0.35
    release_wheel_action_scale: float = 0.05
    policy_initial_leg_action_scale: float = 0.45
    policy_initial_wheel_action_scale: float = 0.08
    cmd_ramp_s: float = 8.0
    policy_wheel_action_limit_early: float = 0.20
    apply_soft_start_in_isaac_play: bool = True
    isaac_play_cmd_limit_early: float = 0.10
    enable_wheel_balance_feedback: bool = True
    pitch_feedback_kp: float = 2.0
    pitch_rate_feedback_kd: float = 0.35
    yaw_feedback_kp: float = 0.95
    yaw_rate_feedback_kd: float = 0.14
    wheel_feedback_limit: float = 3.6
    pitch_cmd_attenuation_start_deg: float = 3.0
    pitch_cmd_attenuation_full_deg: float = 6.0
    yaw_cmd_attenuation_start_deg: float = 8.0
    yaw_cmd_attenuation_full_deg: float = 25.0
    attenuate_linear_cmd_on_heading_error: bool = False
    heading_hold_enabled: bool = False
    heading_hold_kp: float = 0.7
    heading_hold_rate_kd: float = 0.10
    heading_hold_wz_limit: float = 0.35
    wheel_cmd_attenuation_start: float = 1.3
    wheel_cmd_attenuation_full: float = 2.4
    high_speed_cmd_start: float = 0.10
    high_speed_cmd_gain: float = 2.4
    forward_speed_cmd_attenuation_start: float = 0.22
    forward_speed_cmd_attenuation_full: float = 0.42
    wheel_speed_soft_limit: float = 1.55
    wheel_speed_brake_gain: float = 3.8
    forward_speed_soft_limit: float = 0.18
    forward_speed_brake_gain: float = 4.0
    brake_rise_time: float = 0.08
    brake_fall_time: float = 0.35
    yaw_feedback_gain_scale: float = 4.0
    pitch_feedback_gain_scale: float = 1.8
    log_path: Path | None = None
    verbose_obs: bool = False
    handoff_window_s: float = 0.5
    handoff_log_every_step: bool = True


class Phase(str, Enum):
    TETHER = "tether"
    RELEASE = "release"
    STAND = "stand"
    POLICY = "policy"


def quat_wxyz_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quat_wxyz_to_euler_rpy(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=np.float64)


def yaw_to_quat_wxyz(yaw: float) -> np.ndarray:
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


class TorchPolicyRunner:
    def __init__(self, policy_path: Path):
        if torch is None:
            raise RuntimeError("PyTorch is not available, cannot load .pt policy.")
        self.policy = torch.jit.load(str(policy_path), map_location="cpu")
        self.policy.eval()

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            action = self.policy(torch.from_numpy(obs.astype(np.float32)))
        return action.detach().cpu().numpy()


class OnnxPolicyRunner:
    def __init__(self, policy_path: Path):
        if ort is None:
            raise RuntimeError("onnxruntime is not available, cannot load .onnx policy.")
        self.session = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        outputs = self.session.run([self.output_name], {self.input_name: obs.astype(np.float32)})
        return outputs[0]


class BaseLockedSim2Sim:
    def __init__(self, model_path: Path, policy_path: Path, backend: str, cfg: Sim2SimConfig):
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = cfg.sim_dt
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(0)
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.total_mass = float(self.model.body_subtreemass[self.base_body_id])
        self.joint_qpos_adr = {
            name: self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in POLICY_JOINT_ORDER
        }
        self.joint_dof_adr = {
            name: self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in POLICY_JOINT_ORDER
        }
        self.actuator_id = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ACTUATOR_ORDER
        }
        if backend == "torch":
            self.policy = TorchPolicyRunner(policy_path)
        elif backend == "onnx":
            self.policy = OnnxPolicyRunner(policy_path)
        else:  # pragma: no cover
            raise ValueError(f"Unsupported backend: {backend}")

        self.default_joint_pos = np.zeros(len(POLICY_JOINT_ORDER), dtype=np.float32)
        self.stand_joint_pos = np.array([DEFAULT_STAND_JOINT_POS[name] for name in POLICY_JOINT_ORDER], dtype=np.float32)
        self.actuator_ctrlrange = np.array(self.model.actuator_ctrlrange, dtype=np.float64)
        self.last_action = np.zeros((1, 6), dtype=np.float32)
        self.last_ctrl = np.zeros(self.model.nu, dtype=np.float64)
        self.log_file = None
        self.handoff_logged = False
        self.tether_anchor_pos = np.zeros(3, dtype=np.float64)
        self.tether_anchor_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.heading_reference_yaw = 0.0
        self.pitch_brake_state = 0.0
        self.reset_episode_state()
        if self.cfg.log_path is not None:
            self.cfg.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = self.cfg.log_path.open("w", encoding="utf-8")

    def _reset_to_home(self) -> None:
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[:7] = np.array([0.0, 0.0, 0.32, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        if self.cfg.mode == "isaac_play":
            self._apply_isaac_play_reset(randomized=not self.cfg.deterministic_startup_reset)
        else:
            for joint_name, joint_pos in DEFAULT_STAND_JOINT_POS.items():
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id >= 0:
                    qpos_adr = self.model.jnt_qposadr[joint_id]
                    self.data.qpos[qpos_adr] = joint_pos
        mujoco.mj_forward(self.model, self.data)

    def _apply_isaac_play_reset(self, randomized: bool = True) -> None:
        # Match the Isaac play task reset events:
        # - root pose x/y/yaw randomized around the default spawn pose
        # - root linear/angular velocity randomized
        # - joint positions reset by scale around default offsets; since defaults are zero
        #   in this task, Isaac leaves all joint positions/velocities at zero.
        if randomized:
            self.data.qpos[0] = self.rng.uniform(-0.5, 0.5)
            self.data.qpos[1] = self.rng.uniform(-0.5, 0.5)
            self.data.qpos[3:7] = yaw_to_quat_wxyz(self.rng.uniform(-3.14, 3.14))
            self.data.qvel[0:6] = np.array(
                [
                    self.rng.uniform(-0.5, 0.5),
                    self.rng.uniform(-0.5, 0.5),
                    self.rng.uniform(-0.5, 0.5),
                    self.rng.uniform(-0.5, 0.5),
                    self.rng.uniform(-0.5, 0.5),
                    self.rng.uniform(-0.5, 0.5),
                ],
                dtype=np.float64,
            )
        else:
            self.data.qpos[0] = 0.0
            self.data.qpos[1] = 0.0
            self.data.qpos[3:7] = yaw_to_quat_wxyz(0.0)
            self.data.qvel[0:6] = 0.0
        self.data.qpos[2] = 0.32
        for joint_name in POLICY_JOINT_ORDER:
            self.data.qpos[self.joint_qpos_adr[joint_name]] = 0.0
            self.data.qvel[self.joint_dof_adr[joint_name]] = 0.0

    def _refresh_startup_references(self) -> None:
        self.tether_anchor_pos = self.data.qpos[0:3].copy()
        self.tether_anchor_pos[2] = self.cfg.tether_height_target
        self.tether_anchor_quat = self.data.qpos[3:7].copy()
        self.heading_reference_yaw = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))[2]

    def reset_episode_state(self) -> None:
        self._reset_to_home()
        self.last_action[:] = 0.0
        self.last_ctrl[:] = 0.0
        self.handoff_logged = False
        self.pitch_brake_state = 0.0
        self.clear_external_wrench()
        self._refresh_startup_references()

    @staticmethod
    def _attenuation_scale(value: float, start: float, full: float, min_scale: float = 0.0) -> float:
        value = float(value)
        start = float(start)
        full = float(full)
        min_scale = float(np.clip(min_scale, 0.0, 1.0))
        if value <= start:
            return 1.0
        if value >= full:
            return min_scale
        span = max(full - start, 1e-6)
        alpha = (value - start) / span
        return 1.0 - (1.0 - min_scale) * alpha

    def _drive_sign(self, cmd_vx: float, fallback: float = 0.0) -> float:
        if abs(cmd_vx) > 1e-6:
            return math.copysign(1.0, cmd_vx)
        if abs(fallback) > 1e-6:
            return math.copysign(1.0, fallback)
        return 0.0

    def _get_joint_pos_rel(self) -> np.ndarray:
        q = np.array([self.data.qpos[self.joint_qpos_adr[name]] for name in POLICY_JOINT_ORDER], dtype=np.float32)
        return q - self.default_joint_pos

    def _get_joint_vel_rel(self) -> np.ndarray:
        return np.array([self.data.qvel[self.joint_dof_adr[name]] for name in POLICY_JOINT_ORDER], dtype=np.float32)

    def _get_base_ang_vel_body(self) -> np.ndarray:
        return np.array(self.data.sensor("imu_gyro").data, dtype=np.float32)

    def _get_base_lin_vel_body(self) -> np.ndarray:
        quat_wxyz = np.array(self.data.qpos[3:7], dtype=np.float64)
        rotmat = quat_wxyz_to_rotmat(quat_wxyz)
        return (rotmat.T @ np.array(self.data.qvel[0:3], dtype=np.float64)).astype(np.float32)

    def _get_projected_gravity(self) -> np.ndarray:
        quat_wxyz = np.array(self.data.qpos[3:7], dtype=np.float64)
        rotmat = quat_wxyz_to_rotmat(quat_wxyz)
        gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        gravity_body = rotmat.T @ gravity_world
        return gravity_body.astype(np.float32)

    def build_observation(self) -> np.ndarray:
        obs = np.zeros((1, 27), dtype=np.float32)
        obs[0, 0:3] = self._get_base_ang_vel_body()
        obs[0, 3:6] = self._get_projected_gravity()
        # These command channels follow the Isaac policy convention: body-frame
        # linear velocity plus yaw-rate command, not a world-frame cruise vector.
        obs[0, 6:9] = np.array([self.cfg.cmd_vx, self.cfg.cmd_vy, self.cfg.cmd_wz], dtype=np.float32)
        obs[0, 9:15] = self._get_joint_pos_rel()
        obs[0, 15:21] = self._get_joint_vel_rel()
        obs[0, 21:27] = self.last_action[0]
        return obs

    def observation_dict(self, obs: np.ndarray) -> dict[str, list[float]]:
        flat = obs[0].astype(float)
        return {
            "base_ang_vel": flat[0:3].tolist(),
            "projected_gravity": flat[3:6].tolist(),
            "command": flat[6:9].tolist(),
            "joint_pos_rel": flat[9:15].tolist(),
            "joint_vel_rel": flat[15:21].tolist(),
            "last_action": flat[21:27].tolist(),
        }

    def print_observation(self, step: int, phase: Phase, obs: np.ndarray) -> None:
        obs_dict = self.observation_dict(obs)
        print(
            f"obs t={step * self.cfg.sim_dt:6.3f}s phase={phase.value} "
            f"ang_vel={np.array2string(np.array(obs_dict['base_ang_vel']), precision=3, floatmode='fixed')} "
            f"gravity={np.array2string(np.array(obs_dict['projected_gravity']), precision=3, floatmode='fixed')} "
            f"cmd={np.array2string(np.array(obs_dict['command']), precision=3, floatmode='fixed')} "
            f"q={np.array2string(np.array(obs_dict['joint_pos_rel']), precision=3, floatmode='fixed')} "
            f"dq={np.array2string(np.array(obs_dict['joint_vel_rel']), precision=3, floatmode='fixed')} "
            f"last={np.array2string(np.array(obs_dict['last_action']), precision=3, floatmode='fixed')}"
        )

    def hold_pose_ctrl(self) -> np.ndarray:
        ctrl = np.zeros(self.model.nu, dtype=np.float64)
        for i, joint_name in enumerate(LEG_JOINTS):
            actuator_name = f"{joint_name}_pos"
            ctrl[self.actuator_id[actuator_name]] = float(self.stand_joint_pos[i])
        for joint_name in WHEEL_JOINTS:
            actuator_name = f"{joint_name}_vel"
            ctrl[self.actuator_id[actuator_name]] = 0.0
        return ctrl

    def stand_stabilizer_ctrl(self) -> np.ndarray:
        ctrl = self.hold_pose_ctrl()

        roll, pitch, _ = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))
        base_height = float(self.data.qpos[2])
        pitch_rate = float(self.data.qvel[4])
        roll_rate = float(self.data.qvel[3])

        height_err = self.cfg.stand_base_height_target - base_height
        pitch_comp = 0.90 * pitch + 0.18 * pitch_rate
        roll_comp = 0.18 * roll + 0.04 * roll_rate
        squat_comp = np.clip(1.4 * height_err, -0.35, 0.35)

        hip_left = float(self.stand_joint_pos[0] - 0.55 * pitch_comp + 0.20 * roll_comp - 0.35 * squat_comp)
        hip_right = float(self.stand_joint_pos[1] + 0.55 * pitch_comp + 0.20 * roll_comp + 0.35 * squat_comp)
        knee_left = float(self.stand_joint_pos[2] + 0.75 * pitch_comp - 0.18 * roll_comp + 0.85 * squat_comp)
        knee_right = float(self.stand_joint_pos[3] - 0.75 * pitch_comp - 0.18 * roll_comp - 0.85 * squat_comp)

        ctrl[self.actuator_id["HipL_pos"]] = np.clip(hip_left, -1.8, 1.8)
        ctrl[self.actuator_id["HipR_pos"]] = np.clip(hip_right, -1.8, 1.8)
        ctrl[self.actuator_id["KneeL_pos"]] = np.clip(knee_left, -1.8, 1.8)
        ctrl[self.actuator_id["KneeR_pos"]] = np.clip(knee_right, -1.8, 1.8)
        ctrl[self.actuator_id["HubL_vel"]] = 0.0
        ctrl[self.actuator_id["HubR_vel"]] = 0.0
        return ctrl

    def commanded_obs(self, cmd_vx: float, cmd_vy: float, cmd_wz: float) -> np.ndarray:
        obs = self.build_observation()
        obs[0, 6:9] = np.array([cmd_vx, cmd_vy, cmd_wz], dtype=np.float32)
        return obs

    def apply_tether_wrench(self) -> None:
        self.apply_tether_wrench_with_scale(1.0)

    def apply_tether_wrench_with_scale(self, scale: float) -> None:
        self.data.xfrc_applied[self.base_body_id, :] = 0.0
        scale = float(np.clip(scale, 0.0, 1.0))

        pos_err = self.tether_anchor_pos - self.data.qpos[0:3]
        lin_vel = self.data.qvel[0:3]
        force = np.array([180.0, 180.0, 260.0]) * pos_err - np.array([35.0, 35.0, 55.0]) * lin_vel
        force += np.array([0.0, 0.0, self.total_mass * 9.81], dtype=np.float64)

        roll, pitch, yaw = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))
        ang_vel = self.data.qvel[3:6]
        target_roll, target_pitch, target_yaw = quat_wxyz_to_euler_rpy(self.tether_anchor_quat)
        angle_err = np.array([target_roll - roll, target_pitch - pitch, target_yaw - yaw], dtype=np.float64)
        torque = np.array([220.0, 220.0, 80.0]) * angle_err - np.array([18.0, 18.0, 8.0]) * ang_vel

        self.data.xfrc_applied[self.base_body_id, 0:3] = scale * force
        self.data.xfrc_applied[self.base_body_id, 3:6] = scale * torque

    def clear_external_wrench(self) -> None:
        self.data.xfrc_applied[self.base_body_id, :] = 0.0

    def enforce_tether_pose(self) -> None:
        self.data.qpos[0:3] = self.tether_anchor_pos
        self.data.qpos[3:7] = self.tether_anchor_quat
        self.data.qvel[0:6] = 0.0
        self.data.qacc[0:6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def blend_tether_pose(self, blend: float) -> None:
        blend = float(np.clip(blend, 0.0, 1.0))
        if blend <= 0.0:
            return
        self.data.qpos[0:3] = blend * self.tether_anchor_pos + (1.0 - blend) * self.data.qpos[0:3]
        self.data.qpos[3:7] = blend * self.tether_anchor_quat + (1.0 - blend) * self.data.qpos[3:7]
        quat_norm = np.linalg.norm(self.data.qpos[3:7])
        if quat_norm > 1e-8:
            self.data.qpos[3:7] /= quat_norm
        self.data.qvel[0:6] *= 1.0 - blend
        self.data.qacc[0:6] *= 1.0 - blend
        mujoco.mj_forward(self.model, self.data)

    def phase_control(self, phase: Phase) -> np.ndarray:
        if phase == Phase.TETHER and self.cfg.stand_strategy == "pose_hold":
            self.last_action[:] = 0.0
            return self.hold_pose_ctrl()

        if phase == Phase.STAND and self.cfg.stand_strategy == "pose_hold":
            self.last_action[:] = 0.0
            return self.stand_stabilizer_ctrl()

        if phase == Phase.TETHER:
            obs = self.commanded_obs(0.0, 0.0, 0.0)
        elif phase == Phase.STAND:
            obs = self.commanded_obs(0.0, 0.0, 0.0)
        else:
            obs = self.commanded_obs(self.cfg.cmd_vx, self.cfg.cmd_vy, self.cfg.cmd_wz)
        action = self.policy(obs)
        return self.policy_action_to_ctrl(action)

    def phase_command(self, phase: Phase, locomotion_alpha: float, command_alpha: float) -> tuple[float, float, float]:
        if phase != Phase.POLICY:
            return 0.0, 0.0, 0.0
        roll, pitch, yaw = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))
        del roll
        wheel_vel = np.array([self.data.qvel[self.joint_dof_adr[name]] for name in WHEEL_JOINTS], dtype=np.float64)
        wheel_mean = float(np.mean(wheel_vel))
        drive_sign = self._drive_sign(self.cfg.cmd_vx, fallback=wheel_mean)
        forward_pitch_deg = max(0.0, math.degrees(drive_sign * pitch))
        pitch_cmd_scale = self._attenuation_scale(
            forward_pitch_deg,
            self.cfg.pitch_cmd_attenuation_start_deg,
            self.cfg.pitch_cmd_attenuation_full_deg,
        )

        yaw_err = math.atan2(math.sin(self.heading_reference_yaw - yaw), math.cos(self.heading_reference_yaw - yaw))
        yaw_abs_deg = abs(math.degrees(yaw_err))
        yaw_cmd_scale = self._attenuation_scale(
            yaw_abs_deg,
            self.cfg.yaw_cmd_attenuation_start_deg,
            self.cfg.yaw_cmd_attenuation_full_deg,
        )

        wheel_mean_abs = abs(float(np.mean(wheel_vel)))
        wheel_cmd_scale = self._attenuation_scale(
            wheel_mean_abs,
            self.cfg.wheel_cmd_attenuation_start,
            self.cfg.wheel_cmd_attenuation_full,
        )
        base_linvel_body = self._get_base_lin_vel_body()
        forward_speed = max(0.0, drive_sign * float(base_linvel_body[0]))
        forward_speed_cmd_scale = self._attenuation_scale(
            forward_speed,
            self.cfg.forward_speed_cmd_attenuation_start,
            self.cfg.forward_speed_cmd_attenuation_full,
        )

        requested_speed_abs = abs(self.cfg.cmd_vx)
        if requested_speed_abs <= self.cfg.high_speed_cmd_start:
            requested_speed_scale = 1.0
        else:
            requested_speed_scale = 1.0 / (
                1.0 + self.cfg.high_speed_cmd_gain * (requested_speed_abs - self.cfg.high_speed_cmd_start)
            )

        linear_cmd_scale = pitch_cmd_scale * wheel_cmd_scale * forward_speed_cmd_scale * requested_speed_scale
        if self.cfg.attenuate_linear_cmd_on_heading_error:
            linear_cmd_scale *= yaw_cmd_scale

        heading_hold_wz = 0.0
        if self.cfg.heading_hold_enabled and abs(self.cfg.cmd_wz) <= 1e-6:
            yaw_rate = float(self.data.qvel[5])
            heading_hold_wz = self.cfg.heading_hold_kp * yaw_err - self.cfg.heading_hold_rate_kd * yaw_rate
            heading_hold_wz = float(
                np.clip(heading_hold_wz, -self.cfg.heading_hold_wz_limit, self.cfg.heading_hold_wz_limit)
            )
        target_wz = yaw_cmd_scale * self.cfg.cmd_wz + heading_hold_wz
        if self.cfg.mode == "isaac_play":
            if not self.cfg.apply_soft_start_in_isaac_play:
                return (
                    linear_cmd_scale * self.cfg.cmd_vx,
                    linear_cmd_scale * self.cfg.cmd_vy,
                    target_wz,
                )
            alpha = float(np.clip(command_alpha, 0.0, 1.0))
            early_limit = self.cfg.isaac_play_cmd_limit_early
            cmd_limit = early_limit + (1.0 - early_limit) * alpha
            return (
                linear_cmd_scale * cmd_limit * self.cfg.cmd_vx,
                linear_cmd_scale * cmd_limit * self.cfg.cmd_vy,
                cmd_limit * target_wz,
            )
        alpha = float(np.clip(command_alpha, 0.0, 1.0))
        return (
            linear_cmd_scale * alpha * self.cfg.cmd_vx,
            linear_cmd_scale * alpha * self.cfg.cmd_vy,
            alpha * target_wz,
        )

    def shaped_policy_action(self, action: np.ndarray, phase: Phase, locomotion_alpha: float) -> np.ndarray:
        if self.cfg.mode == "isaac_play" and not self.cfg.apply_soft_start_in_isaac_play:
            return action
        shaped = action.copy()
        alpha = float(np.clip(locomotion_alpha, 0.0, 1.0))
        if phase == Phase.RELEASE:
            leg_scale = self.cfg.release_leg_action_scale
            wheel_scale = self.cfg.release_wheel_action_scale
        else:
            leg_scale = self.cfg.policy_initial_leg_action_scale + (1.0 - self.cfg.policy_initial_leg_action_scale) * alpha
            wheel_scale = self.cfg.policy_initial_wheel_action_scale + (
                1.0 - self.cfg.policy_initial_wheel_action_scale
            ) * alpha
        shaped[:, 0:4] *= leg_scale
        shaped[:, 4:6] *= wheel_scale
        # Diagnostic clamp: keep wheels conservative early in policy handoff to see
        # whether forward pitching is mainly caused by wheel drive or by leg posture.
        early_wheel_limit = self.cfg.policy_wheel_action_limit_early + (1.0 - self.cfg.policy_wheel_action_limit_early) * alpha
        shaped[:, 4:6] = np.clip(shaped[:, 4:6], -early_wheel_limit, early_wheel_limit)
        return shaped

    def policy_action_to_ctrl(self, action: np.ndarray) -> np.ndarray:
        clipped = np.clip(action.reshape(1, 6), -self.cfg.action_clip, self.cfg.action_clip).astype(np.float32)
        self.last_action[:] = clipped

        ctrl = np.zeros(self.model.nu, dtype=np.float64)
        for i, joint_name in enumerate(LEG_JOINTS):
            actuator_name = f"{joint_name}_pos"
            ctrl[self.actuator_id[actuator_name]] = clipped[0, i] * self.cfg.leg_action_scale
        for i, joint_name in enumerate(WHEEL_JOINTS, start=4):
            actuator_name = f"{joint_name}_vel"
            ctrl[self.actuator_id[actuator_name]] = clipped[0, i] * self.cfg.wheel_action_scale
        return ctrl

    def apply_wheel_balance_feedback(self, ctrl: np.ndarray, phase: Phase) -> np.ndarray:
        if not self.cfg.enable_wheel_balance_feedback or phase != Phase.POLICY:
            return ctrl
        roll, pitch, yaw = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))
        del roll
        pitch_rate = float(self.data.qvel[4])
        yaw_rate = float(self.data.qvel[5])
        yaw_err = math.atan2(math.sin(self.heading_reference_yaw - yaw), math.cos(self.heading_reference_yaw - yaw))

        yaw_correction = self.cfg.yaw_feedback_kp * yaw_err - self.cfg.yaw_rate_feedback_kd * yaw_rate
        wheel_mean = 0.5 * (ctrl[self.actuator_id["HubL_vel"]] + ctrl[self.actuator_id["HubR_vel"]])
        wheel_diff = 0.5 * (ctrl[self.actuator_id["HubL_vel"]] - ctrl[self.actuator_id["HubR_vel"]])
        drive_sign = 0.0
        if abs(wheel_mean) > 1e-6:
            drive_sign = math.copysign(1.0, wheel_mean)
        elif abs(self.cfg.cmd_vx) > 1e-6:
            drive_sign = math.copysign(1.0, self.cfg.cmd_vx)

        commanded_speed = abs(self.cfg.cmd_vx)
        pitch_gain_scale = 1.0 + self.cfg.pitch_feedback_gain_scale * commanded_speed
        yaw_gain_scale = 1.0 + self.cfg.yaw_feedback_gain_scale * commanded_speed
        forward_pitch = drive_sign * pitch
        forward_pitch_rate = drive_sign * pitch_rate
        pitch_brake = (
            pitch_gain_scale * self.cfg.pitch_feedback_kp * max(0.0, forward_pitch)
            + pitch_gain_scale * self.cfg.pitch_rate_feedback_kd * max(0.0, forward_pitch_rate)
        )
        if abs(wheel_mean) > self.cfg.wheel_speed_soft_limit:
            pitch_brake += self.cfg.wheel_speed_brake_gain * (abs(wheel_mean) - self.cfg.wheel_speed_soft_limit)
        yaw_oversteer = max(0.0, abs(yaw_err) - math.radians(3.0))
        yaw_correction += math.copysign(0.35 * abs(wheel_diff) * yaw_oversteer, yaw_err)
        yaw_correction *= yaw_gain_scale

        pitch_brake = float(np.clip(pitch_brake, -self.cfg.wheel_feedback_limit, self.cfg.wheel_feedback_limit))
        yaw_correction = float(np.clip(yaw_correction, -self.cfg.wheel_feedback_limit, self.cfg.wheel_feedback_limit))

        ctrl = ctrl.copy()
        ctrl[self.actuator_id["HubL_vel"]] -= pitch_brake + yaw_correction
        ctrl[self.actuator_id["HubR_vel"]] -= pitch_brake - yaw_correction
        ctrl[self.actuator_id["HubL_vel"]] = np.clip(
            ctrl[self.actuator_id["HubL_vel"]],
            self.actuator_ctrlrange[self.actuator_id["HubL_vel"], 0],
            self.actuator_ctrlrange[self.actuator_id["HubL_vel"], 1],
        )
        ctrl[self.actuator_id["HubR_vel"]] = np.clip(
            ctrl[self.actuator_id["HubR_vel"]],
            self.actuator_ctrlrange[self.actuator_id["HubR_vel"], 0],
            self.actuator_ctrlrange[self.actuator_id["HubR_vel"], 1],
        )
        return ctrl

    def contact_flags(self) -> tuple[bool, bool]:
        base_contact = False
        leg_contact = False
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
            geom2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
            pair = f"{geom1} {geom2}".lower()
            if "ground" not in pair:
                continue
            if "base_link_col" in pair or "base_link_contact" in pair:
                base_contact = True
            if any(name in pair for name in ["hip", "knee"]):
                leg_contact = True
        return base_contact, leg_contact

    def write_diag_log(
        self,
        step: int,
        phase: Phase,
        tether_scale: float,
        cmd_vx: float,
        cmd_vy: float,
        cmd_wz: float,
        base_contact: bool,
        leg_contact: bool,
        obs: np.ndarray | None = None,
    ) -> None:
        if self.log_file is None:
            return
        roll, pitch, yaw = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))
        record = {
            "t": step * self.cfg.sim_dt,
            "phase": phase.value,
            "tether_scale": tether_scale,
            "cmd": [cmd_vx, cmd_vy, cmd_wz],
            "base_pos": self.data.qpos[0:3].tolist(),
            "base_quat_wxyz": self.data.qpos[3:7].tolist(),
            "base_linvel": self.data.qvel[0:3].tolist(),
            "base_angvel": self.data.qvel[3:6].tolist(),
            "base_rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
            "joint_pos": {name: float(self.data.qpos[self.joint_qpos_adr[name]]) for name in POLICY_JOINT_ORDER},
            "joint_vel": {name: float(self.data.qvel[self.joint_dof_adr[name]]) for name in POLICY_JOINT_ORDER},
            "last_action": self.last_action[0].astype(float).tolist(),
            "last_ctrl": self.last_ctrl.astype(float).tolist(),
            "base_contact": base_contact,
            "leg_contact": leg_contact,
            "leg_action_scale": self.cfg.leg_action_scale,
            "wheel_action_scale": self.cfg.wheel_action_scale,
        }
        if obs is not None:
            record["observation"] = self.observation_dict(obs)
        self.log_file.write(json.dumps(record, ensure_ascii=True) + "\n")

    def _run_loop(self, viewer=None) -> None:
        num_steps = None if self.cfg.duration is None else int(self.cfg.duration / self.cfg.sim_dt)
        ctrl = self.hold_pose_ctrl()
        wall_t0 = time.perf_counter()
        tether_steps = int(self.cfg.startup_tether_s / self.cfg.sim_dt)
        tether_release_steps = int(self.cfg.tether_release_s / self.cfg.sim_dt)
        stand_steps = int(self.cfg.startup_stand_s / self.cfg.sim_dt)
        locomotion_ramp_steps = max(1, int(self.cfg.locomotion_ramp_s / self.cfg.sim_dt))
        cmd_ramp_steps = max(1, int(self.cfg.cmd_ramp_s / self.cfg.sim_dt))
        handoff_window_steps = max(1, int(self.cfg.handoff_window_s / self.cfg.sim_dt))

        print("Starting minimal sim2sim")
        print(f"model:   {self.model.nq=} {self.model.nv=} {self.model.nu=}")
        print(f"policy:  obs_dim=27 act_dim=6 backend={self.policy.__class__.__name__}")
        print(
            f"command: vx={self.cfg.cmd_vx:.3f} vy={self.cfg.cmd_vy:.3f} wz={self.cfg.cmd_wz:.3f} "
            f"dt={self.cfg.sim_dt:.6f} decimation={self.cfg.decimation} "
            f"mode={self.cfg.mode} stand_strategy={self.cfg.stand_strategy} "
            f"leg_scale={self.cfg.leg_action_scale:.2f} wheel_scale={self.cfg.wheel_action_scale:.2f} "
            f"playback_speed={self.cfg.playback_speed:.2f} "
            f"tether_s={tether_steps * self.cfg.sim_dt:.2f} "
            f"release_s={tether_release_steps * self.cfg.sim_dt:.2f} "
            f"stand_s={stand_steps * self.cfg.sim_dt:.2f}"
        )

        step = 0
        while num_steps is None or step < num_steps:
            if step < tether_steps:
                phase = Phase.TETHER
            elif step < tether_steps + tether_release_steps:
                phase = Phase.RELEASE
            elif step < tether_steps + tether_release_steps + stand_steps:
                phase = Phase.STAND
            else:
                phase = Phase.POLICY

            if phase == Phase.TETHER:
                self.enforce_tether_pose()

            if phase == Phase.TETHER:
                self.apply_tether_wrench()
                tether_scale = 1.0
            elif phase == Phase.RELEASE:
                release_progress = (step - tether_steps) / tether_release_steps
                tether_scale = max(0.0, 1.0 - release_progress)
                self.apply_tether_wrench_with_scale(tether_scale)
            else:
                if phase == Phase.STAND:
                    stand_progress = (step - tether_steps - tether_release_steps) / max(1, stand_steps)
                    tether_scale = max(0.0, 1.0 - stand_progress)
                    self.apply_tether_wrench_with_scale(tether_scale)
                else:
                    tether_scale = 0.0
                    self.clear_external_wrench()

            if phase in (Phase.RELEASE, Phase.POLICY):
                policy_start = tether_steps
                policy_progress = (step - policy_start) / locomotion_ramp_steps
                locomotion_alpha = min(1.0, max(0.0, policy_progress))
            else:
                locomotion_alpha = 0.0

            if phase == Phase.POLICY:
                policy_step_from_release = step - (tether_steps + tether_release_steps + stand_steps)
                command_alpha = min(1.0, max(0.0, policy_step_from_release / cmd_ramp_steps))
            else:
                command_alpha = 0.0

            cmd_vx, cmd_vy, cmd_wz = self.phase_command(phase, locomotion_alpha, command_alpha)
            current_obs = self.commanded_obs(cmd_vx, cmd_vy, cmd_wz)

            if phase not in (Phase.RELEASE, Phase.POLICY) or step % self.cfg.decimation == 0:
                if phase == Phase.RELEASE:
                    if self.cfg.stand_strategy == "policy_zero":
                        obs = self.commanded_obs(0.0, 0.0, 0.0)
                        action = self.policy(obs)
                        action = self.shaped_policy_action(action, phase, locomotion_alpha)
                        ctrl = self.policy_action_to_ctrl(action)
                    else:
                        self.last_action[:] = 0.0
                        ctrl = self.stand_stabilizer_ctrl()
                elif phase in (Phase.POLICY,) and self.cfg.stand_strategy == "policy_zero":
                    obs = current_obs
                    action = self.policy(obs)
                    action = self.shaped_policy_action(action, phase, locomotion_alpha)
                    ctrl = self.policy_action_to_ctrl(action)
                elif phase in (Phase.POLICY,):
                    obs = current_obs
                    action = self.policy(obs)
                    action = self.shaped_policy_action(action, phase, locomotion_alpha)
                    ctrl = self.policy_action_to_ctrl(action)
                else:
                    ctrl = self.phase_control(phase)

            if self.cfg.verbose_obs and phase in (Phase.RELEASE, Phase.POLICY):
                if phase == Phase.POLICY and not self.handoff_logged:
                    print("Policy handoff observation snapshot:")
                    self.print_observation(step, phase, current_obs)
                    self.handoff_logged = True

            self.data.ctrl[:] = ctrl
            self.data.ctrl[:] = self.apply_wheel_balance_feedback(self.data.ctrl.copy(), phase)
            self.last_ctrl[:] = self.data.ctrl
            mujoco.mj_step(self.model, self.data)

            if phase == Phase.TETHER:
                self.enforce_tether_pose()
            elif phase == Phase.RELEASE:
                self.blend_tether_pose(tether_scale)

            if viewer is not None:
                viewer.sync()

            if self.cfg.real_time:
                sim_time = (step + 1) * self.cfg.sim_dt / max(self.cfg.playback_speed, 1e-6)
                elapsed = time.perf_counter() - wall_t0
                sleep_time = sim_time - elapsed
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

            if step % int(1.0 / self.cfg.sim_dt) == 0:
                base_pos = self.data.qpos[0:3].copy()
                base_linvel = self.data.qvel[0:3].copy()
                roll, pitch, yaw = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))
                hub_vel = np.array([self.data.qvel[self.joint_dof_adr[name]] for name in WHEEL_JOINTS], dtype=np.float64)
                ctrl_low = self.actuator_ctrlrange[:, 0]
                ctrl_high = self.actuator_ctrlrange[:, 1]
                ctrl_sat = np.logical_or(
                    np.isclose(self.last_ctrl, ctrl_low, atol=1e-3),
                    np.isclose(self.last_ctrl, ctrl_high, atol=1e-3),
                )
                base_contact, leg_contact = self.contact_flags()
                print(
                    f"t={step * self.cfg.sim_dt:6.2f}s "
                    f"phase={phase.value} "
                    f"tether_scale={tether_scale:.2f} cmd=({cmd_vx:+.2f},{cmd_vy:+.2f},{cmd_wz:+.2f}) "
                    f"base_pos=({base_pos[0]:+.3f}, {base_pos[1]:+.3f}, {base_pos[2]:+.3f}) "
                    f"base_linvel=({base_linvel[0]:+.3f}, {base_linvel[1]:+.3f}, {base_linvel[2]:+.3f}) "
                    f"rpy_deg=({math.degrees(roll):+.1f}, {math.degrees(pitch):+.1f}, {math.degrees(yaw):+.1f}) "
                    f"hub_vel=({hub_vel[0]:+.3f}, {hub_vel[1]:+.3f}) "
                    f"base_contact={base_contact} leg_contact={leg_contact} "
                    f"ctrl_sat={ctrl_sat.astype(int).tolist()} "
                    f"last_action={np.array2string(self.last_action[0], precision=3, floatmode='fixed')}"
                )
                self.write_diag_log(
                    step,
                    phase,
                    tether_scale,
                    cmd_vx,
                    cmd_vy,
                    cmd_wz,
                    base_contact,
                    leg_contact,
                    obs=current_obs,
                )

            handoff_start = tether_steps + tether_release_steps + stand_steps
            if self.cfg.handoff_log_every_step and handoff_start - handoff_window_steps <= step <= handoff_start + handoff_window_steps:
                base_pos = self.data.qpos[0:3].copy()
                roll, pitch, yaw = quat_wxyz_to_euler_rpy(np.array(self.data.qpos[3:7], dtype=np.float64))
                handoff_base_contact, handoff_leg_contact = self.contact_flags()
                print(
                    f"handoff t={step * self.cfg.sim_dt:6.3f}s phase={phase.value} "
                    f"cmd=({cmd_vx:+.2f},{cmd_vy:+.2f},{cmd_wz:+.2f}) "
                    f"base_z={base_pos[2]:+.3f} pitch_deg={math.degrees(pitch):+.2f} roll_deg={math.degrees(roll):+.2f} yaw_deg={math.degrees(yaw):+.2f} "
                    f"base_contact={handoff_base_contact} leg_contact={handoff_leg_contact} "
                    f"ctrl={np.array2string(self.last_ctrl, precision=3, floatmode='fixed')} "
                    f"act={np.array2string(self.last_action[0], precision=3, floatmode='fixed')}"
                )
                self.write_diag_log(
                    step,
                    phase,
                    tether_scale,
                    cmd_vx,
                    cmd_vy,
                    cmd_wz,
                    base_contact=handoff_base_contact,
                    leg_contact=handoff_leg_contact,
                    obs=current_obs,
                )

            step += 1

        print("Finished minimal sim2sim")
        if self.log_file is not None:
            self.log_file.close()

    def run(self, use_viewer: bool = False) -> None:
        self.reset_episode_state()
        if use_viewer:
            import mujoco.viewer

            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                self._run_loop(viewer=viewer)
        else:
            self._run_loop(viewer=None)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal MuJoCo 3.4.0 sim2sim for wheeled-base-locked")
    parser.add_argument(
        "--mode",
        choices=["isaac_play", "experimental"],
        default="isaac_play",
        help="Run mode. `isaac_play` aligns reset/command semantics with Isaac Play; `experimental` keeps custom tether/release logic.",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="Path to wheeled_leg.xml")
    parser.add_argument(
        "--policy",
        type=Path,
        default=DEFAULT_PT_POLICY,
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
    parser.add_argument("--pitch-rate-feedback-kd", type=float, default=0.35, help="Wheel common-mode pitch-rate feedback gain.")
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    policy_path = args.policy
    if args.backend == "torch" and policy_path == DEFAULT_PT_POLICY.with_suffix(".pt"):
        policy_path = DEFAULT_PT_POLICY
    if args.backend == "onnx" and policy_path == DEFAULT_PT_POLICY:
        policy_path = DEFAULT_ONNX_POLICY

    cfg = Sim2SimConfig(
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
    runner = BaseLockedSim2Sim(args.model, policy_path, args.backend, cfg)
    runner.run(use_viewer=args.viewer)


if __name__ == "__main__":
    main()
