# 2-DOF Robotic Arm in MuJoCo

A hand-written MJCF model of a two-link planar arm (base → Arm1 → Arm2), built from a SolidWorks assembly, with joint limits derived analytically. The 6.5° joint1 limit is confirmed in MuJoCo contact simulation; the joint2 limit is provisional (see Known issues).

**Mechanical design:** [@reetu019](https://github.com/reetu019), SolidWorks model, used with permission.

**Simulation, MJCF, and joint-limit analysis:** [@isha-prasad](https://github.com/isha-prasad)

## What's in this repo

| File | Purpose |
|---|---|
| `arm_2dof.xml` | MJCF model: two hinge joints about Y, actuated at both joints |
| `load_arm.py` | Compiles and validates the model, saves a `.mjb` |
| `view_arm.py` | Passive viewer with live joint-angle readout |
| `meshes/` | Arm1 and Arm2 STLs exported from SolidWorks (units: mm) |

Both joints use `position` actuators (act1: kp = 30, act2: kp = 20; kv = 3 and force limited to ±50 N·m via a default class). Each `ctrlrange` matches its joint range, given in radians.

## Model summary

- Units: CAD in mm and grams; mesh scale is `0.001` and masses are converted to kg.
- Links: L1 = 180 mm (pivot to pivot), L2 = 150 mm (pivot to tip), both 40 mm wide in-plane and 20 mm thick along Y. Arm1 has a 15 mm pin at the elbow that Arm2 mounts on.
- Masses (from SolidWorks): Arm1 169.13 g, Arm2 131.39 g, base slab 1559.52 g, shoulder bracket 42.35 g.
- The slab and shoulder bracket are primitive boxes, not meshes. The bracket box is visual-only (`contype="0" conaffinity="0"`) because, as a simplified box, it overlaps Arm1's shoulder end.

## Joint limits

Convention: zero means fully extended. Positive angles lower the arm (right-hand rule about +Y), and Arm2 folds in the positive direction.

| Joint | `range` (deg) |
|---|---|
| joint1 | `-173.9 6.5` |
| joint2 | `0 170` (provisional, see Known issues) |

### Why joint1 stops at 6.5° and −173.9°

The shoulder pivot is 36.07 mm above the slab's top face and 140.04 mm from the slab edge. Arm1 reaches ~200 mm, so it overhangs that edge. That means the first contact is not with the top face or the floor: it's the arm's lower edge (20 mm below its centerline) touching the slab's top **corner**. That contact happens at

```
36.07·cos θ − 140.04·sin θ = 20   →   θ ≈ 6.50°
```

Floor contact would only happen at 14.72°, so the corner is the limit that matters. Contact simulation in MuJoCo stops the arm at the same 6.50°.

Folding back the other way, the same corner contact happens at the slab's opposite edge, 148.36 mm from the pivot:

```
36.07·cos θ − 148.36·sin θ = 20   →   θ ≈ 6.14°   →   joint1 = −(180 − 6.14) ≈ −173.9°
```

## Running it

```bash
pip install -r requirements.txt
python load_arm.py    # compile + validate
python view_arm.py    # interactive viewer
```

Tested with MuJoCo 3.11.0 and Python 3.13.5 on Windows.

## Known issues

- **Arm1 and Arm2 interpenetrate at the elbow.** Arm2 sits in (nearly) the same plane as Arm1 instead of beside it along the bolt axis. MuJoCo excludes parent–child contacts by default, so the overlap isn't detected. The joint1 limits (Arm1 vs slab corners) are unaffected; the joint2 fold limit will be re-derived after the fix. Tracked in [Issue #1](https://github.com/isha-prasad/mujoco-2dof-arm/issues/1).

## A thing to watch out for

`<compiler angle="degree"/>` converts joint `range` to radians, but **not** actuator `ctrlrange` or keyframe `qpos`. Those are always in radians.

## Limitations

- Not validated against the physical arm. Limits are derived from CAD geometry only.
- Masses are from CAD; inertia tensors are computed by MuJoCo from each geom's shape at that mass (uniform density). Joint friction and damping are not identified from hardware.
- The shoulder bracket is a simplified, visual-only box, so contacts with it are not simulated.
- No controller or trajectory tracking beyond the basic actuators.
- MuJoCo joint limits are soft; expect ~0.02–0.03° overshoot at the stops.

## License

Code (`*.py`, `arm_2dof.xml`): MIT, see `LICENSE`.

Mechanical design and meshes: © [@reetu019](https://github.com/reetu019), all rights reserved, used with permission.
