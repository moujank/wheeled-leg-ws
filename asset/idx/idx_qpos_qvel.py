import mujoco
model = mujoco.MjModel.from_xml_path("/home/moujank/project/wheeled-leg-ws/modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml")

joint_names = ["HipL", "HipR", "KneeL", "KneeR", "HubL", "HubR"]
joint_type = {
    mujoco.mjtJoint.mjJNT_FREE: "FREE",
    mujoco.mjtJoint.mjJNT_BALL: "BALL",
    mujoco.mjtJoint.mjJNT_SLIDE: "SLIDE",
    mujoco.mjtJoint.mjJNT_HINGE: "HINGE",
}
for name in joint_names:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    print(f"{name}: joint_id={jid}, qpos_adr={model.jnt_qposadr[jid]}, qvel_adr={model.jnt_dofadr[jid]}")
for name in joint_names:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    print(
        f"{name}: id={jid}, type={joint_type.get(model.jnt_type[jid],model.jnt_type[jid])}, "
        f"qpos_adr={model.jnt_qposadr[jid]}, qvel_adr={model.jnt_dofadr[jid]}, "
        f"axis={model.jnt_axis[jid]}"
    )