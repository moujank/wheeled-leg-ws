import mujoco
model = mujoco.MjModel.from_xml_path("/home/moujank/project/wheeled-leg-ws/modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml")
for i in range(model.nu):
    print(f"ctrl[{i}] ->", mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i))

for i in range(model.njnt):
    print(f"joint[{i}] ->", mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i))