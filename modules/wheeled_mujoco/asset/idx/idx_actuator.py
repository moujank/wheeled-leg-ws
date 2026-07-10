import mujoco
model = mujoco.MjModel.from_xml_path("/home/moujank/project/wheeled-leg-ws/modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml")

actuator_names = ["HipL_pos", "HipR_pos", "KneeL_pos", "KneeR_pos", "HubL_vel","HubR_vel"]
for name in actuator_names:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    trnid = model.actuator_trnid[aid][0]
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
    print(
        f"{name}: actuator_id={aid}, ctrl_index={aid}, joint={joint_name}, "
        f"ctrlrange={model.actuator_ctrlrange[aid]}, forcerange={model.actuator_forcerange[aid]}"
    )