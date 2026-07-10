from __future__ import annotations

import argparse
import time
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


# ============================================================================
# Minimal sim2sim 前三步：
#
# 第 1 步：固定最小程序骨架
# - 明确模型路径、时间参数、命令量
# - 加载 MuJoCo model/data
#
# 第 2 步：固定所有后续要用到的索引映射
# - joint -> qpos/qvel
# - actuator -> ctrl
# - body/site/sensor -> id / adr / dim
#
# 第 3 步：写显式 reset
# - 不依赖 XML 里的 home keyframe
# - 直接以当前 Isaac 训练默认零位作为最小基线
#
# 第 4 步：写 observation builder 第一版
# - 先严格做对 27D 结构和顺序
# - 暂时不加训练噪声
# - 暂时不接 policy，只打印每个分块验证语义
#
# 第 5 步：写 action adapter 第一版
# - 先不接 policy，先用固定测试 raw action
# - 明确 raw action -> scaled target -> data.ctrl 的映射
# - 先不做 wheel 取反，不加保护逻辑
#
# 第 6 步：接 policy inference + 可视化界面
# - 支持 torch jit / onnx 两种最小 backend
# - 没有策略文件时自动回退到测试 action
# - 支持 MuJoCo 官方 viewer 做实时可视化
#
# 注意：
# 这一版的目标是把“最小 sim2sim 闭环”真正串起来，
# 但仍然不加入 startup 保护、噪声、归一化细节和复杂稳定性补丁。
# ============================================================================

ROOT_DIR = Path(__file__).resolve().parents[3]
MODEL_XML = ROOT_DIR / "modules" / "wheeled_mujoco" / "asset" / "mjcf" / "wheeled_leg.xml"
POLICY_PATH = ROOT_DIR / "PLACEHOLDER_POLICY.pt"

# MuJoCo 当前 XML 中的仿真步长。
# 这里先显式写死，避免后面控制逻辑“默认相信 XML”，导致排查时不知道真正采用的是哪套时间基线。
SIM_DT = 0.002

# Isaac 训练侧时间基线。
# 已知训练配置为：
#   sim.dt = 1 / 120
#   decimation = 4
# 因此训练时 policy 的控制周期约为 0.033333 秒。
ISAAC_SIM_DT = 1.0 / 120.0
ISAAC_DECIMATION = 4
CONTROL_DT = ISAAC_SIM_DT * ISAAC_DECIMATION
CONTROL_STEPS = round(CONTROL_DT / SIM_DT)

# 最小闭环里先把命令量固定成常量，后面 observation builder 会直接把它们拼进 obs。
CMD_VX = 0.2
CMD_VY = 0.0
CMD_WZ = 0.0

# 第一版只跑 2 秒，目的只是确认程序壳子、reset 和控制 tick 都工作正常。
ROLLOUT_SECONDS = 8.0
OBS_PRINT_ON_FIRST_TICK_ONLY = True
ACTION_PRINT_ON_FIRST_TICK_ONLY = True
VIS_SYNC_TO_REALTIME = True

# 训练 policy 的原始 action 顺序。
# 后面 action adapter 必须严格按这个顺序解释网络输出。
TRAIN_ACTION_ORDER = ["HipL", "HipR", "KneeL", "KneeR", "HubL", "HubR"]

# MuJoCo actuator 的 ctrl 写入顺序。
# 注意：这和训练 action 顺序是两回事，虽然当前正好一一对应，但概念上必须分开。
MUJOCO_CTRL_ORDER = ["HipL_pos", "HipR_pos", "KneeL_pos", "KneeR_pos", "HubL_vel", "HubR_vel"]

# MuJoCo 内部 joint state 的“自然读取顺序”。
# 当前是按 qpos/qvel 地址排列的顺序，不等于训练 action 顺序。
# 后面做 observation builder 时，必须把读取出来的关节状态重排成训练侧要求的顺序。
MUJOCO_STATE_ORDER = ["HipL", "KneeL", "HubL", "HipR", "KneeR", "HubR"]

# 这些 body/site/sensor 名字先集中列出来，避免在代码深处写散乱的字符串字面量。
# 这样一旦资产名字变化，只需改这里和索引构建逻辑。
BODY_NAMES = [
    "base_link",
    "left_hip_link",
    "left_knee_link",
    "left_wheel_link",
    "right_hip_link",
    "right_knee_link",
    "right_wheel_link",
]

SITE_NAMES = [
    "imu",
    "base_com",
    "left_wheel_site",
    "right_wheel_site",
]

SENSOR_NAMES = [
    "imu_gyro",
    "imu_linear_velocity",
    "imu_linear_acceleration",
    "imu_orientation",
    "imu_position",
    "imu_upvector",
    "imu_global_linvel",
    "imu_global_angvel",
    "HipL_pos",
    "HipL_vel",
    "HipR_pos",
    "HipR_vel",
    "KneeL_pos",
    "KneeL_vel",
    "KneeR_pos",
    "KneeR_vel",
    "HubL_pos",
    "HubL_vel",
    "HubR_pos",
    "HubR_vel",
    "HipL_torque",
    "HipR_torque",
    "KneeL_torque",
    "KneeR_torque",
    "HubL_torque",
    "HubR_torque",
]

# 当前训练基线：
# - Hip/Knee position action scale = 0.5
# - Hub velocity action scale = 10.0
LEG_ACTION_SCALE = 0.5
WHEEL_ACTION_SCALE = 10.0
WHEEL_SIGN_LEFT = 1.0
WHEEL_SIGN_RIGHT = 1.0

# 第一版先把关节默认零位全部定为 0。
# 这与当前 Isaac 训练资产默认零位一致。
DEFAULT_JOINT_POS = {
    "HipL": 0.0,
    "HipR": 0.0,
    "KneeL": 0.0,
    "KneeR": 0.0,
    "HubL": 0.0,
    "HubR": 0.0,
}

# 不接 policy 时，先用固定 raw action 做语义检查。
# 这里故意选一个非零、且每维不同的测试向量，避免因为数值重复而看不出顺序错误。
TEST_RAW_ACTION = np.array([0.10, -0.20, 0.30, -0.40, 0.25, -0.35], dtype=np.float64)


class TorchPolicyRunner:
    """最小 Torch JIT policy loader。

    这里假设导出产物是 `torch.jit.load(...)` 可直接加载的 policy.pt。
    输入 obs 形状固定为 `(27,)`，输出 action 期望为 `(6,)`。
    """

    def __init__(self, policy_path: Path):
        if torch is None:
            raise RuntimeError("PyTorch is not available, cannot load .pt policy.")
        self.policy = torch.jit.load(str(policy_path), map_location="cpu")
        self.policy.eval()

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        obs_tensor = torch.from_numpy(obs.astype(np.float32))
        with torch.no_grad():
            action = self.policy(obs_tensor)
        action_np = action.detach().cpu().numpy()
        return np.asarray(action_np, dtype=np.float64).reshape(-1)


class OnnxPolicyRunner:
    """最小 ONNX policy loader。"""

    def __init__(self, policy_path: Path):
        if ort is None:
            raise RuntimeError("onnxruntime is not available, cannot load .onnx policy.")
        self.session = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        outputs = self.session.run([self.output_name], {self.input_name: obs.astype(np.float32)[None, :]})
        return np.asarray(outputs[0], dtype=np.float64).reshape(-1)


def joint_info(model: mujoco.MjModel, name: str) -> dict[str, object]:
    """读取单个 joint 的核心索引信息。

    这里固定三类信息：
    1. joint id：后面需要调试关节名字和 MuJoCo 内部对象时会用到
    2. qpos_adr：后面读 joint position 时用
    3. qvel_adr：后面读 joint velocity 时用
    4. axis：这里只是缓存方便打印和调试，后面不直接拿它做控制
    """
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"joint not found: {name}")
    return {
        "id": joint_id,
        "qpos_adr": int(model.jnt_qposadr[joint_id]),
        "qvel_adr": int(model.jnt_dofadr[joint_id]),
        "axis": model.jnt_axis[joint_id].copy(),
    }


def actuator_info(model: mujoco.MjModel, name: str) -> dict[str, object]:
    """读取单个 actuator 的核心索引信息。

    最重要的是 ctrl_index。
    后面不管 policy 输出什么，最终都要落到 data.ctrl[ctrl_index] 上。
    """
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if actuator_id < 0:
        raise ValueError(f"actuator not found: {name}")
    joint_id = int(model.actuator_trnid[actuator_id][0])
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    return {
        "id": actuator_id,
        "ctrl_index": actuator_id,
        "joint_id": joint_id,
        "joint_name": joint_name,
    }


def body_info(model: mujoco.MjModel, name: str) -> dict[str, object]:
    """读取 body id。

    这一版虽然还没正式写 observation builder，
    但后面 base pose、base velocity、接触调试都会依赖 body id。
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError(f"body not found: {name}")
    return {"id": body_id}


def site_info(model: mujoco.MjModel, name: str) -> dict[str, object]:
    """读取 site id。

    imu、轮子 site 都属于后面 observation/debug 的关键参考点。
    """
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise ValueError(f"site not found: {name}")
    return {"id": site_id}


def sensor_info(model: mujoco.MjModel, name: str) -> dict[str, object]:
    """读取 sensor 在 sensordata 中的地址和维度。

    这样后面读观测时不需要硬编码 sensordata 的切片位置。
    """
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        raise ValueError(f"sensor not found: {name}")
    return {
        "id": sensor_id,
        "adr": int(model.sensor_adr[sensor_id]),
        "dim": int(model.sensor_dim[sensor_id]),
    }


def build_index_map(model: mujoco.MjModel) -> dict[str, object]:
    """一次性构建所有后续会用到的索引映射。

    这一步的原则是：
    - 所有“名字 -> 索引”的工作只做一次
    - 主循环里不再反复查名字
    - 后面如果 observation/action 有错，优先检查这里，而不是去怀疑主循环
    """
    return {
        "train_action_order": TRAIN_ACTION_ORDER,
        "mujoco_ctrl_order": MUJOCO_CTRL_ORDER,
        "mujoco_state_order": MUJOCO_STATE_ORDER,
        "joints": {name: joint_info(model, name) for name in TRAIN_ACTION_ORDER},
        "actuators": {name: actuator_info(model, name) for name in MUJOCO_CTRL_ORDER},
        "bodies": {name: body_info(model, name) for name in BODY_NAMES},
        "sites": {name: site_info(model, name) for name in SITE_NAMES},
        "sensors": {name: sensor_info(model, name) for name in SENSOR_NAMES},
    }


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """四元数共轭。

    MuJoCo 的 freejoint quaternion 采用 (w, x, y, z) 顺序。
    对单位四元数来说，共轭就是逆旋转，后面做世界系 -> 机体系向量投影会用到。
    """
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """用单位四元数 q 旋转向量 v。

    这里采用显式公式，避免引入额外依赖。
    q 格式固定为 (w, x, y, z)。
    """
    w, x, y, z = q
    q_vec = np.array([x, y, z], dtype=np.float64)
    t = 2.0 * np.cross(q_vec, v)
    return v + w * t + np.cross(q_vec, t)


def world_vector_to_base_frame(base_quat_wxyz: np.ndarray, world_vec: np.ndarray) -> np.ndarray:
    """把世界系向量转到 base 机体系。

    projected_gravity 的核心就是：
    - 先取世界系重力方向
    - 再投影到 base frame
    """
    return quat_rotate(quat_conjugate(base_quat_wxyz), world_vec)


def reset_state(model: mujoco.MjModel, data: mujoco.MjData, index_map: dict[str, object]) -> None:
    """显式 reset 到最小 sim2sim 基线状态。

    这里刻意不使用 XML 的 home keyframe，原因有两个：
    1. 当前 home qpos 本身就有可疑点，不适合作为唯一基线
    2. sim2sim 里 reset 语义必须由我们自己明确控制，不能依赖资产文件隐含状态

    当前采用的最小基线直接对齐到 Isaac 训练资产默认零位：
    - base pos = (0, 0, 0.32)
    - base quat = (1, 0, 0, 0)
    - 所有 base 速度 = 0
    - 六个关节位置 = 0
    - 六个关节速度 = 0
    - 所有 ctrl = 0
    """
    # 先让 MuJoCo 自己把 data 清成初始值，避免上一轮 rollout 的残留状态污染当前 reset。
    mujoco.mj_resetData(model, data)

    # freejoint 的 qpos 布局：
    #   qpos[0:3] = base xyz
    #   qpos[3:7] = base quaternion (w, x, y, z)
    data.qpos[0:3] = [0.0, 0.0, 0.32]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    # freejoint 的 qvel 布局：
    #   qvel[0:3] = base linear velocity
    #   qvel[3:6] = base angular velocity
    data.qvel[0:6] = 0.0

    # 训练默认零位里六个关节全为 0。
    for joint_name in TRAIN_ACTION_ORDER:
        qpos_adr = index_map["joints"][joint_name]["qpos_adr"]
        qvel_adr = index_map["joints"][joint_name]["qvel_adr"]
        data.qpos[qpos_adr] = 0.0
        data.qvel[qvel_adr] = 0.0

    # 第一版 ctrl 明确清零，避免 actuator 上一次写入留下的目标值继续影响当前轮次。
    data.ctrl[:] = 0.0

    # 把 qpos/qvel 的改动前向传播到派生量（xpos, xquat, sensordata 等）。
    mujoco.mj_forward(model, data)


def build_policy_observation(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    index_map: dict[str, object],
    last_action: np.ndarray,
    command: np.ndarray,
) -> dict[str, np.ndarray]:
    """构建第一版 27D policy observation。

    当前严格遵守训练侧的 observation 顺序：
    1. base_ang_vel       3D
    2. projected_gravity  3D
    3. velocity_commands  3D
    4. joint_pos          6D
    5. joint_vel          6D
    6. last_action        6D

    第一版只保证：
    - 维度正确
    - 顺序正确
    - 坐标系转换逻辑明确

    当前暂不做：
    - 训练噪声
    - observation normalization
    - 更复杂的 IMU 语义逼近
    """
    # freejoint base quaternion, MuJoCo 顺序为 (w, x, y, z)
    base_quat = np.array(data.qpos[3:7], dtype=np.float64)

    # MuJoCo freejoint qvel[3:6] 就是 base 的角速度。
    # 当前先直接把它当作 base_ang_vel 使用。
    base_ang_vel = np.array(data.qvel[3:6], dtype=np.float64)

    # 世界系重力方向。
    # 这里取单位重力方向而不是 -9.81 数值版，
    # 因为训练里的 projected_gravity 关注的是方向而不是重力模长。
    world_gravity_dir = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    projected_gravity = world_vector_to_base_frame(base_quat, world_gravity_dir)

    velocity_commands = np.asarray(command, dtype=np.float64).reshape(3)

    # 训练要求的 joint 顺序是 TRAIN_ACTION_ORDER，而不是 MuJoCo 内部 qpos 地址顺序。
    # 当前默认零位全 0，因此第一版 joint_pos_rel = 当前 joint qpos 本身。
    joint_pos = np.array(
        [data.qpos[index_map["joints"][joint_name]["qpos_adr"]] for joint_name in TRAIN_ACTION_ORDER],
        dtype=np.float64,
    )
    joint_vel = np.array(
        [data.qvel[index_map["joints"][joint_name]["qvel_adr"]] for joint_name in TRAIN_ACTION_ORDER],
        dtype=np.float64,
    )

    obs = np.concatenate(
        [
            base_ang_vel,
            projected_gravity,
            velocity_commands,
            joint_pos,
            joint_vel,
            last_action,
        ]
    )

    return {
        "base_ang_vel": base_ang_vel,
        "projected_gravity": projected_gravity,
        "velocity_commands": velocity_commands,
        "joint_pos": joint_pos,
        "joint_vel": joint_vel,
        "last_action": last_action.copy(),
        "obs": obs,
    }


def adapt_action_to_ctrl(
    raw_action: np.ndarray,
    index_map: dict[str, object],
    wheel_sign_left: float = WHEEL_SIGN_LEFT,
    wheel_sign_right: float = WHEEL_SIGN_RIGHT,
    handoff_alpha: float = 1.0,
) -> dict[str, np.ndarray]:
    """把 6D raw action 适配成 MuJoCo ctrl。

    当前严格遵守训练动作语义：
    - raw_action[0:4] 对应 HipL/HipR/KneeL/KneeR 的 position action
    - raw_action[4:6] 对应 HubL/HubR 的 velocity action

    映射规则：
    - 腿：target_pos = default_joint_pos + raw * 0.5
    - 轮：target_vel = raw * 10.0

    第一版保持最直接映射：
    - 不自动对 wheel 取反
    - 不加入 startup / balance / safety 补丁
    """
    if raw_action.shape != (6,):
        raise ValueError(f"expected raw_action shape (6,), got {raw_action.shape}")

    # index_map 这一版虽然还没参与数值计算，但保留这个入参有两个好处：
    # 1. 让 action adapter 的接口形状和后续正式版保持一致
    # 2. 如果后面要从索引表拿 joint 默认零位、clip 参数或方向补偿，就不用改函数签名
    _ = index_map

    leg_scaled = raw_action[:4] * LEG_ACTION_SCALE
    wheel_scaled = raw_action[4:] * WHEEL_ACTION_SCALE
    wheel_scaled[0] *= wheel_sign_left
    wheel_scaled[1] *= wheel_sign_right

    joint_targets = {
        "HipL": DEFAULT_JOINT_POS["HipL"] + leg_scaled[0],
        "HipR": DEFAULT_JOINT_POS["HipR"] + leg_scaled[1],
        "KneeL": DEFAULT_JOINT_POS["KneeL"] + leg_scaled[2],
        "KneeR": DEFAULT_JOINT_POS["KneeR"] + leg_scaled[3],
        "HubL": wheel_scaled[0],
        "HubR": wheel_scaled[1],
    }

    # 第一版先在 Python 里显式 clip。
    # 这样后面看到 ctrl 数值时，能明确知道这是程序逻辑主动限幅，而不是 XML 背后偷偷截断。
    joint_targets["HipL"] = float(np.clip(joint_targets["HipL"], -3.14, 3.14))
    joint_targets["HipR"] = float(np.clip(joint_targets["HipR"], -3.14, 3.14))
    joint_targets["KneeL"] = float(np.clip(joint_targets["KneeL"], -3.14, 3.14))
    joint_targets["KneeR"] = float(np.clip(joint_targets["KneeR"], -3.14, 3.14))
    joint_targets["HubL"] = float(np.clip(joint_targets["HubL"], -10.0, 10.0))
    joint_targets["HubR"] = float(np.clip(joint_targets["HubR"], -10.0, 10.0))

    ctrl = np.zeros(len(MUJOCO_CTRL_ORDER), dtype=np.float64)
    ctrl[0] = joint_targets["HipL"]
    ctrl[1] = joint_targets["HipR"]
    ctrl[2] = joint_targets["KneeL"]
    ctrl[3] = joint_targets["KneeR"]
    ctrl[4] = joint_targets["HubL"]
    ctrl[5] = joint_targets["HubR"]

    # handoff_alpha 用于最小软接入：
    # - 0.0 表示完全不给 policy/action
    # - 1.0 表示完全信任当前 action
    # 第一版只做整体线性缩放，不额外设计 stand phase。
    handoff_alpha = float(np.clip(handoff_alpha, 0.0, 1.0))
    ctrl *= handoff_alpha

    return {
        "raw_action": raw_action.copy(),
        "leg_scaled": leg_scaled.copy(),
        "wheel_scaled": wheel_scaled.copy(),
        "handoff_alpha": np.array([handoff_alpha], dtype=np.float64),
        "ctrl": ctrl,
    }


def apply_ctrl(data: mujoco.MjData, ctrl: np.ndarray) -> None:
    """把 action adapter 生成的 ctrl 写进 MuJoCo。"""
    if ctrl.shape != (len(MUJOCO_CTRL_ORDER),):
        raise ValueError(
            f"expected ctrl shape {(len(MUJOCO_CTRL_ORDER),)}, got {ctrl.shape}"
        )
    data.ctrl[:] = ctrl


def load_policy_runner(policy_path: Path | None, backend: str) -> object | None:
    """按 backend 加载 policy。

    如果 policy_path 为空或文件不存在，则返回 None，
    主循环会自动回退到固定测试 action 模式。
    """
    if policy_path is None:
        return None
    if not policy_path.exists():
        print(f"[policy] file not found, fallback to test action: {policy_path}")
        return None

    print(f"[policy] loading backend={backend} path={policy_path}")
    if backend == "torch":
        return TorchPolicyRunner(policy_path)
    if backend == "onnx":
        return OnnxPolicyRunner(policy_path)
    raise ValueError(f"unsupported backend: {backend}")


def get_raw_action(
    policy_runner: object | None,
    obs_dict: dict[str, np.ndarray],
) -> tuple[np.ndarray, str]:
    """返回当前 control tick 使用的 raw action。

    返回值里额外带一个字符串 mode，方便日志里明确当前是：
    - policy inference 模式
    - test action 回退模式
    """
    if policy_runner is None:
        return TEST_RAW_ACTION.copy(), "test_action"

    obs = obs_dict["obs"]
    raw_action = policy_runner(obs)
    raw_action = np.asarray(raw_action, dtype=np.float64).reshape(-1)
    if raw_action.shape != (6,):
        raise RuntimeError(f"policy output shape must be (6,), got {raw_action.shape}")
    return raw_action, "policy"


def print_config_summary(args: argparse.Namespace) -> None:
    """打印当前最小 sim2sim 配置。

    第一版优先保证“所有关键假设都显式可见”，
    而不是追求日志简洁。
    """
    print("== Minimal sim2sim step 1 config ==")
    print(f"MODEL_XML      : {MODEL_XML}")
    print(f"POLICY_PATH    : {POLICY_PATH}")
    print(f"SIM_DT         : {SIM_DT}")
    print(f"ISAAC_SIM_DT   : {ISAAC_SIM_DT}")
    print(f"ISAAC_DECIMATE : {ISAAC_DECIMATION}")
    print(f"CONTROL_DT     : {CONTROL_DT}")
    print(f"CONTROL_STEPS  : {CONTROL_STEPS}")
    print(f"COMMAND        : vx={args.cmd_vx}, vy={args.cmd_vy}, wz={args.cmd_wz}")
    print(f"ROLLOUT_SEC    : {args.duration}")
    print(f"WHEEL_SIGN     : left={args.wheel_sign_left}, right={args.wheel_sign_right}")
    print(f"HANDOFF_SEC    : {args.handoff_seconds}")


def print_index_summary(index_map: dict[str, object]) -> None:
    """打印最关键的顺序和索引信息。

    这里打印的目的不是“漂亮”，而是为了后面排查：
    - action 顺序是否混了
    - qpos/qvel 地址是否读错
    - ctrl 写入是否绑错 actuator
    """
    print("== Order summary ==")
    print(f"train_action_order : {index_map['train_action_order']}")
    print(f"mujoco_ctrl_order  : {index_map['mujoco_ctrl_order']}")
    print(f"mujoco_state_order : {index_map['mujoco_state_order']}")

    print("== Joint summary ==")
    for name in TRAIN_ACTION_ORDER:
        info = index_map["joints"][name]
        print(
            f"{name}: joint_id={info['id']} qpos_adr={info['qpos_adr']} "
            f"qvel_adr={info['qvel_adr']} axis={info['axis']}"
        )

    print("== Actuator summary ==")
    for name in MUJOCO_CTRL_ORDER:
        info = index_map["actuators"][name]
        print(
            f"{name}: actuator_id={info['id']} ctrl_index={info['ctrl_index']} "
            f"joint={info['joint_name']}"
        )


def print_reset_summary(data: mujoco.MjData, index_map: dict[str, object]) -> None:
    """打印 reset 后的关键状态。

    reset 是 sim2sim 的基础语义之一。
    第一版必须让 reset 结果可以被肉眼直接确认。
    """
    print("== Reset summary ==")
    print(
        "base qpos xyz/quat : "
        f"{data.qpos[0]:+.6f} {data.qpos[1]:+.6f} {data.qpos[2]:+.6f} | "
        f"{data.qpos[3]:+.6f} {data.qpos[4]:+.6f} {data.qpos[5]:+.6f} {data.qpos[6]:+.6f}"
    )
    print(
        "base qvel lin/ang  : "
        f"{data.qvel[0]:+.6f} {data.qvel[1]:+.6f} {data.qvel[2]:+.6f} | "
        f"{data.qvel[3]:+.6f} {data.qvel[4]:+.6f} {data.qvel[5]:+.6f}"
    )

    print("joint reset state  :")
    for joint_name in TRAIN_ACTION_ORDER:
        qpos_adr = index_map["joints"][joint_name]["qpos_adr"]
        qvel_adr = index_map["joints"][joint_name]["qvel_adr"]
        print(
            f"  {joint_name}: "
            f"qpos={data.qpos[qpos_adr]:+.6f} "
            f"qvel={data.qvel[qvel_adr]:+.6f}"
        )

    print(f"ctrl reset state   : {list(data.ctrl)}")


def print_observation_summary(obs_dict: dict[str, np.ndarray]) -> None:
    """打印 observation 的分块和总维度。

    observation builder 第一版最重要的验证项不是“数值看起来高级”，
    而是：
    - 总长度是否为 27
    - 每一块长度是否符合训练定义
    - joint/last_action 顺序是否符合训练语义
    """
    print("== Observation summary ==")
    print(
        f"base_ang_vel      shape={obs_dict['base_ang_vel'].shape} "
        f"value={obs_dict['base_ang_vel']}"
    )
    print(
        f"projected_gravity shape={obs_dict['projected_gravity'].shape} "
        f"value={obs_dict['projected_gravity']}"
    )
    print(
        f"velocity_commands shape={obs_dict['velocity_commands'].shape} "
        f"value={obs_dict['velocity_commands']}"
    )
    print(
        f"joint_pos         shape={obs_dict['joint_pos'].shape} "
        f"value={obs_dict['joint_pos']}"
    )
    print(
        f"joint_vel         shape={obs_dict['joint_vel'].shape} "
        f"value={obs_dict['joint_vel']}"
    )
    print(
        f"last_action       shape={obs_dict['last_action'].shape} "
        f"value={obs_dict['last_action']}"
    )
    print(f"full_obs          shape={obs_dict['obs'].shape}")


def print_action_summary(action_dict: dict[str, np.ndarray]) -> None:
    """打印 action adapter 的输入和输出。

    这一层的首要目标不是让机器人动得好，
    而是确认：
    - raw action 顺序没搞错
    - scale 没搞错
    - ctrl 写入顺序没搞错
    """
    print("== Action summary ==")
    print(f"raw_action   shape={action_dict['raw_action'].shape} value={action_dict['raw_action']}")
    print(f"leg_scaled   shape={action_dict['leg_scaled'].shape} value={action_dict['leg_scaled']}")
    print(
        f"wheel_scaled shape={action_dict['wheel_scaled'].shape} "
        f"value={action_dict['wheel_scaled']}"
    )
    print(f"handoff_alpha shape={action_dict['handoff_alpha'].shape} value={action_dict['handoff_alpha']}")
    print(f"ctrl         shape={action_dict['ctrl'].shape} value={action_dict['ctrl']}")


def parse_args() -> argparse.Namespace:
    """解析最小 sim2sim 命令行参数。

    第一版只保留最必要的控制项：
    - policy 路径
    - backend 类型
    - 是否开启 viewer
    - 仿真时长
    """
    parser = argparse.ArgumentParser(description="Minimal MuJoCo sim2sim for wheeled_leg.")
    parser.add_argument(
        "--policy",
        type=Path,
        default=POLICY_PATH,
        help="Path to exported policy (.pt or .onnx). If missing, fallback to test action mode.",
    )
    parser.add_argument(
        "--backend",
        choices=["torch", "onnx"],
        default="torch",
        help="Policy backend type.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=ROLLOUT_SECONDS,
        help="Simulation duration in seconds.",
    )
    parser.add_argument(
        "--viewer",
        dest="viewer",
        action="store_true",
        help="Launch MuJoCo viewer.",
    )
    parser.add_argument(
        "--no-viewer",
        dest="viewer",
        action="store_false",
        help="Run without MuJoCo viewer.",
    )
    parser.add_argument("--cmd-vx", type=float, default=CMD_VX, help="Commanded forward velocity.")
    parser.add_argument("--cmd-vy", type=float, default=CMD_VY, help="Commanded lateral velocity.")
    parser.add_argument("--cmd-wz", type=float, default=CMD_WZ, help="Commanded yaw rate.")
    parser.add_argument(
        "--wheel-sign-left",
        type=float,
        default=WHEEL_SIGN_LEFT,
        help="Multiply left wheel action by this sign/scalar before writing ctrl.",
    )
    parser.add_argument(
        "--wheel-sign-right",
        type=float,
        default=WHEEL_SIGN_RIGHT,
        help="Multiply right wheel action by this sign/scalar before writing ctrl.",
    )
    parser.add_argument(
        "--handoff-seconds",
        type=float,
        default=0.0,
        help="Linearly ramp action from 0 to full scale over this duration.",
    )
    parser.set_defaults(viewer=True)
    return parser.parse_args()


def run_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    index_map: dict[str, object],
    policy_runner: object | None,
    duration_s: float,
    command: np.ndarray,
    wheel_sign_left: float,
    wheel_sign_right: float,
    handoff_seconds: float,
    viewer=None,
) -> None:
    """最小 sim2sim 主循环。

    当前循环已经具备最小闭环语义：
    - reset 后进入仿真
    - 每个 control tick 构 observation
    - 从 policy 或测试 action 生成 raw action
    - 通过 action adapter 写 ctrl
    - 更新 last_action
    - 单步推进 MuJoCo

    但它仍然刻意保持极简：
    - 不做 reset 终止判定
    - 不做 startup handoff
    - 不做 command ramp
    - 不做安全补丁
    """
    last_action = np.zeros(len(TRAIN_ACTION_ORDER), dtype=np.float64)
    total_steps = int(duration_s / SIM_DT)

    print("== Runtime mode ==")
    print("policy_mode      :", "enabled" if policy_runner is not None else "fallback_test_action")
    print("viewer           :", "enabled" if viewer is not None else "disabled")
    print("duration_s       :", duration_s)
    print("total_steps      :", total_steps)
    print("command          :", command)
    print("wheel_sign       :", np.array([wheel_sign_left, wheel_sign_right], dtype=np.float64))
    print("handoff_seconds  :", handoff_seconds)

    wall_start = time.perf_counter()

    for step in range(total_steps):
        if step % CONTROL_STEPS == 0:
            sim_time = step * SIM_DT
            print(f"[control tick] step={step} sim_time={sim_time:.3f}")

            obs_dict = build_policy_observation(model, data, index_map, last_action, command)
            if (step == 0) or (not OBS_PRINT_ON_FIRST_TICK_ONLY):
                print_observation_summary(obs_dict)

            raw_action, action_mode = get_raw_action(policy_runner, obs_dict)
            if handoff_seconds > 0.0:
                handoff_alpha = min(1.0, data.time / handoff_seconds)
            else:
                handoff_alpha = 1.0

            action_dict = adapt_action_to_ctrl(
                raw_action,
                index_map,
                wheel_sign_left=wheel_sign_left,
                wheel_sign_right=wheel_sign_right,
                handoff_alpha=handoff_alpha,
            )
            apply_ctrl(data, action_dict["ctrl"])

            if (step == 0) or (not ACTION_PRINT_ON_FIRST_TICK_ONLY):
                print(f"action_source  : {action_mode}")
                print_action_summary(action_dict)

            last_action = raw_action.copy()

        mujoco.mj_step(model, data)

        if viewer is not None:
            viewer.sync()
            if VIS_SYNC_TO_REALTIME:
                target_wall_elapsed = data.time
                wall_elapsed = time.perf_counter() - wall_start
                sleep_s = target_wall_elapsed - wall_elapsed
                if sleep_s > 0.0:
                    time.sleep(sleep_s)

    print("== Rollout done ==")
    print(f"final_time={data.time:.6f}")


def main() -> None:
    args = parse_args()
    print_config_summary(args)

    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    index_map = build_index_map(model)
    policy_runner = load_policy_runner(args.policy, args.backend)

    print("== Model loaded ==")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} model_dt={model.opt.timestep}")
    print_index_summary(index_map)

    # 在真正进入 rollout 之前先显式 reset 一次，
    # 让后续所有仿真都从我们自己定义的最小基线开始。
    reset_state(model, data, index_map)
    print_reset_summary(data, index_map)

    if args.viewer:
        from mujoco import viewer as mj_viewer

        with mj_viewer.launch_passive(model, data) as viewer:
            run_loop(
                model=model,
                data=data,
                index_map=index_map,
                policy_runner=policy_runner,
                duration_s=args.duration,
                command=np.array([args.cmd_vx, args.cmd_vy, args.cmd_wz], dtype=np.float64),
                wheel_sign_left=args.wheel_sign_left,
                wheel_sign_right=args.wheel_sign_right,
                handoff_seconds=args.handoff_seconds,
                viewer=viewer,
            )
    else:
        run_loop(
            model=model,
            data=data,
            index_map=index_map,
            policy_runner=policy_runner,
            duration_s=args.duration,
            command=np.array([args.cmd_vx, args.cmd_vy, args.cmd_wz], dtype=np.float64),
            wheel_sign_left=args.wheel_sign_left,
            wheel_sign_right=args.wheel_sign_right,
            handoff_seconds=args.handoff_seconds,
            viewer=None,
        )


if __name__ == "__main__":
    main()
