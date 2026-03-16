#!/usr/bin/env python
"""Quick test: apply oscillating torque and print base position. Run from SimpleChimera: mjpython scripts/test_mujoco_motion.py"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mujoco

def find_model():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for rel in ["robot/six_leg_insect.xml", "SimpleChimera/robot/six_leg_insect.xml"]:
        path = os.path.join(base, rel)
        if os.path.isfile(path):
            return path
    return os.path.join(base, "robot", "six_leg_insect.xml")

def main():
    path = find_model()
    if not os.path.isfile(path):
        print("Model not found:", path)
        return
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    nu = model.nu
    print("Actuators (nu):", nu, "Expect 18")
    mujoco.mj_resetData(model, data)
    data.qpos[0] = 0
    data.qpos[1] = 0
    data.qpos[2] = 0.085
    data.qpos[3] = 1
    data.qpos[4] = 0
    data.qpos[5] = 0
    data.qpos[6] = 0
    mujoco.mj_forward(model, data)
    t = 0
    dt = model.opt.timestep
    for step in range(500):
        # Simple oscillating torque so legs move
        ctrl = 0.04 * np.sin(t + np.linspace(0, 2*np.pi, nu))
        ctrl[9:18] = -ctrl[9:18]
        np.copyto(data.ctrl, ctrl)
        mujoco.mj_step(model, data)
        t += dt
        if step % 100 == 0:
            x, y, z = data.qpos[0], data.qpos[1], data.qpos[2]
            vx = data.qvel[0]
            print(f"Step {step}: pos=({x:.4f}, {y:.4f}, {z:.4f}) vx={vx:.5f} ctrl[0]={data.ctrl[0]:.4f}")
    print("If pos/vx changed, sim and actuators work.")

if __name__ == "__main__":
    main()
