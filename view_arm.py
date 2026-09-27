import mujoco
import mujoco.viewer
import time
import math
import os

XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_2dof.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
if key_id >= 0:
    mujoco.mj_resetDataKeyframe(model, data, key_id)
else:
    mujoco.mj_resetData(model, data)

j1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
j2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint2")

j1_lo, j1_hi = [math.degrees(v) for v in model.jnt_range[j1]]
j2_lo, j2_hi = [math.degrees(v) for v in model.jnt_range[j2]]

a1 = model.jnt_qposadr[j1]
a2 = model.jnt_qposadr[j2]

print()
print("  joint1 range : [%.1f, %.1f] deg" % (j1_lo, j1_hi))
print("  joint2 range : [%.1f, %.1f] deg" % (j2_lo, j2_hi))
print()
print("  Drag the act1 / act2 sliders in the left panel.")
print("  Push act2 positive to check the fold direction.")
print("  Close the window to quit.")
print()

TOL = 0.5

with mujoco.viewer.launch_passive(model, data) as viewer:
    last = 0.0
    while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(model, data)
        viewer.sync()

        now = time.time()
        if now - last > 0.1:
            last = now

            q1 = math.degrees(data.qpos[a1])
            q2 = math.degrees(data.qpos[a2])
            interior = 180.0 - abs(q2)

            f1 = ""
            if q1 <= j1_lo + TOL:
                f1 = " <MIN>"
            elif q1 >= j1_hi - TOL:
                f1 = " <MAX>"

            f2 = ""
            if q2 <= j2_lo + TOL:
                f2 = " <EXTENDED>"
            elif q2 >= j2_hi - TOL:
                f2 = " <FOLD>"

            tip = data.site_xpos[0]

            print("\r  j1 %8.2f%-7s   j2 %8.2f%-11s   interior %6.1f   tip (%.3f, %.3f, %.3f)"
                  % (q1, f1, q2, f2, interior, tip[0], tip[1], tip[2]),
                  end="", flush=True)

        wait = model.opt.timestep - (time.time() - step_start)
        if wait > 0:
            time.sleep(wait)

print()
print()
print("  Viewer closed.")
