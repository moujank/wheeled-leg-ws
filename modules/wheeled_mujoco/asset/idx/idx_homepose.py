import mujoco
model = mujoco.MjModel.from_xml_path("/home/moujank/project/wheeled-leg-ws/modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml")

print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)

for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(i, name, "qpos_adr=", model.jnt_qposadr[i], "qvel_adr=",
    model.jnt_dofadr[i], "type=", model.jnt_type[i])

import re
from pathlib import Path

text = Path("/home/moujank/project/wheeled-leg-ws/modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml").read_text()
m = re.search(r'<key name="home" qpos="([^"]+)"', text)
print(m.group(1) if m else "home not found")