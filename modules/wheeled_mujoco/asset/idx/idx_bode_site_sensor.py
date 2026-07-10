import mujoco
model = mujoco.MjModel.from_xml_path("/home/moujank/project/wheeled-leg-ws/modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml")

for obj_type, names in [
    (mujoco.mjtObj.mjOBJ_BODY, ["base_link", "left_wheel_link", "right_wheel_link"]),
    (mujoco.mjtObj.mjOBJ_SITE, ["imu", "base_com", "left_wheel_site","right_wheel_site"]),
]:
    for name in names:
        obj_id = mujoco.mj_name2id(model, obj_type, name)
        print(f"{name}: id={obj_id}")

for i in range(model.nsensor):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
    adr = model.sensor_adr[i]
    dim = model.sensor_dim[i]
    print(f"{i}: {name}, adr={adr}, dim={dim}")