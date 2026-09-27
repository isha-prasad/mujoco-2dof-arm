import mujoco
import os

XML_PATH   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_2dof.xml")
SAVE_PATH  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_2dof_compiled.mjb")

print(f"Loading  : {os.path.abspath(XML_PATH)}")

try:
    model = mujoco.MjModel.from_xml_path(XML_PATH)
except Exception as e:
    print(f"\n[ERROR] {e}")
    raise SystemExit(1)

print("Status   : compiled OK\n")

print(f"  bodies   : {model.nbody}")
print(f"  joints   : {model.njnt}")
print(f"  geoms    : {model.ngeom}")
print(f"  actuators: {model.nu}")
print(f"  sensors  : {model.nsensor}")
print(f"  meshes   : {model.nmesh}")
print()

for i in range(model.njnt):
    name  = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    lo, hi = model.jnt_range[i]
    lo_deg = lo * 180 / 3.14159265358979
    hi_deg = hi * 180 / 3.14159265358979
    limited = bool(model.jnt_limited[i])
    print(f"  [{i}] {name:10s}  limited={limited}  range=[{lo_deg:.1f}°, {hi_deg:.1f}°]")
print()

data = mujoco.MjData(model)
mujoco.mj_kinematics(model, data)
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    x, y, z = data.xpos[i]
    print(f"  [{i}] {name:10s}  pos=({x:.4f}, {y:.4f}, {z:.4f})")
print()

for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    lo, hi = model.actuator_ctrlrange[i]
    print(f"  [{i}] {name:10s}  ctrlrange=[{lo:.1f}, {hi:.1f}]")
print()

mujoco.mj_saveModel(model, SAVE_PATH)
print(f"Saved    : {os.path.abspath(SAVE_PATH)}")
