import argparse
from pathlib import Path

import mujoco


XML_PATH = (
    Path(__file__).resolve().parents[1] / "mjcf" / "wheeled_leg.xml"
)


LEG_ACTUATORS = {
    "HipL": ("HipL_pos", 0.1),
    "HipR": ("HipR_pos", 0.1),
    "KneeL": ("KneeL_pos", 0.1),
    "KneeR": ("KneeR_pos", 0.1),
}

WHEEL_ACTUATORS = {
    "HubL": ("HubL_vel", 1.0),
    "HubR": ("HubR_vel", 1.0),
}


def reset_data(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = [0.0, 0.0, 0.32]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"actuator not found: {name}")
    return aid


def joint_id(model: mujoco.MjModel, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise ValueError(f"joint not found: {name}")
    return jid


def body_id(model: mujoco.MjModel, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise ValueError(f"body not found: {name}")
    return bid


def run_leg_check(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_name: str,
    steps: int,
) -> None:
    actuator_name, target = LEG_ACTUATORS[joint_name]
    aid = actuator_id(model, actuator_name)
    jid = joint_id(model, joint_name)
    qpos_adr = model.jnt_qposadr[jid]
    qvel_adr = model.jnt_dofadr[jid]
    axis = model.jnt_axis[jid]
    child_body_name = {
        "HipL": "left_hip_link",
        "HipR": "right_hip_link",
        "KneeL": "left_knee_link",
        "KneeR": "right_knee_link",
    }[joint_name]
    bid = body_id(model, child_body_name)

    print(f"== Leg Axis Check: {joint_name} ==")
    print(f"actuator={actuator_name} ctrl_index={aid}")
    print(f"axis={axis} qpos_adr={qpos_adr} qvel_adr={qvel_adr}")

    original_gravity = model.opt.gravity.copy()
    model.opt.gravity[:] = 0.0

    for sign in (1.0, -1.0):
        reset_data(model, data)
        start_qpos = float(data.qpos[qpos_adr])
        start_body_xpos = data.xpos[bid].copy()

        data.ctrl[aid] = sign * target
        for _ in range(steps):
            mujoco.mj_step(model, data)

        end_qpos = float(data.qpos[qpos_adr])
        end_qvel = float(data.qvel[qvel_adr])
        end_body_xpos = data.xpos[bid].copy()
        body_delta = end_body_xpos - start_body_xpos

        print(
            f"cmd={sign * target:+.3f} "
            f"qpos: {start_qpos:+.6f} -> {end_qpos:+.6f} "
            f"qvel={end_qvel:+.6f}"
        )
        print(
            f"{child_body_name} world delta xyz="
            f"[{body_delta[0]:+.6f}, {body_delta[1]:+.6f}, {body_delta[2]:+.6f}]"
        )

    model.opt.gravity[:] = original_gravity


def run_wheel_check(model: mujoco.MjModel, data: mujoco.MjData, steps: int) -> None:
    left_actuator, target = WHEEL_ACTUATORS["HubL"]
    right_actuator, _ = WHEEL_ACTUATORS["HubR"]
    left_aid = actuator_id(model, left_actuator)
    right_aid = actuator_id(model, right_actuator)
    hubl_jid = joint_id(model, "HubL")
    hubr_jid = joint_id(model, "HubR")
    hubl_qvel_adr = model.jnt_dofadr[hubl_jid]
    hubr_qvel_adr = model.jnt_dofadr[hubr_jid]
    base_bid = body_id(model, "base_link")

    print("== Wheel Axis Check: HubL + HubR ==")
    print(f"HubL axis={model.jnt_axis[hubl_jid]}")
    print(f"HubR axis={model.jnt_axis[hubr_jid]}")

    for sign in (1.0, -1.0):
        reset_data(model, data)
        start_base_pos = data.xpos[base_bid].copy()

        data.ctrl[left_aid] = sign * target
        data.ctrl[right_aid] = sign * target
        for _ in range(steps):
            mujoco.mj_step(model, data)

        end_base_pos = data.xpos[base_bid].copy()
        base_delta = end_base_pos - start_base_pos
        hubl_qvel = float(data.qvel[hubl_qvel_adr])
        hubr_qvel = float(data.qvel[hubr_qvel_adr])

        print(
            f"cmd_left=cmd_right={sign * target:+.3f} "
            f"HubL_qvel={hubl_qvel:+.6f} HubR_qvel={hubr_qvel:+.6f}"
        )
        print(
            "base_link world delta xyz="
            f"[{base_delta[0]:+.6f}, {base_delta[1]:+.6f}, {base_delta[2]:+.6f}]"
        )
        print(
            "Interpretation: if delta x > 0 under positive same-sign wheel command, "
            "then same-sign command drives +x forward in MuJoCo."
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check MuJoCo joint axis/control sign semantics for wheeled_leg."
    )
    parser.add_argument(
        "--joint",
        choices=["HipL", "HipR", "KneeL", "KneeR", "wheels", "all"],
        default="all",
        help="Which check to run.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=200,
        help="Simulation steps per test command.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    print(f"Loaded model: {XML_PATH}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} dt={model.opt.timestep}")

    if args.joint == "all":
        for joint_name in ("HipL", "HipR", "KneeL", "KneeR"):
            run_leg_check(model, data, joint_name, args.steps)
        run_wheel_check(model, data, args.steps)
        return

    if args.joint == "wheels":
        run_wheel_check(model, data, args.steps)
        return

    run_leg_check(model, data, args.joint, args.steps)


if __name__ == "__main__":
    main()
