import time
import os

import mujoco
import mujoco.viewer

this_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(this_dir, "mjmodel.xml")
model = mujoco.MjModel.from_xml_path(xml_path)

data  = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()

    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(model, data)
        viewer.sync()

