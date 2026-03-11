#!/usr/bin/env python
"""
Run trained SNN balance policy with PyBullet GUI.

Usage:
  python run.py --policy best_policy.npz --gui
"""

import argparse
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env.balance_env import BalanceEnv
from snn.snn_controller import policy_from_params, cpg_snn_controller_from_params


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=str, default="best_policy.npz", help="Path to .npz with 'params'")
    parser.add_argument("--gui", action="store_true", help="Show PyBullet GUI")
    parser.add_argument("--steps", type=int, default=5000, help="Max steps to run")
    parser.add_argument("--max_torque", type=float, default=None, help="Max joint torque (default: env default; use 0.15 for policies trained with old env)")
    parser.add_argument("--torque_scale", type=float, default=1.0, help="Scale policy output (e.g. 0.5 = gentler; use if still flying)")
    parser.add_argument("--smooth", type=float, default=0.82, help="Action smoothing 0=no smoothing, 1=full (default 0.82 allows faster leg movement)")
    parser.add_argument("--no-cpg", action="store_true", help="Use raw SNN only (no CPG march rhythm)")
    args = parser.parse_args()

    data = np.load(args.policy)
    params = data["params"]
    if args.no_cpg:
        policy = policy_from_params(params)
    else:
        policy = cpg_snn_controller_from_params(params)
    kwargs = {"gui": args.gui, "max_steps": args.steps}
    if args.max_torque is not None:
        kwargs["max_torque"] = args.max_torque
    env = BalanceEnv(**kwargs)
    state = env.reset()
    policy.reset()
    total_reward = 0.0
    episode_ended_printed = False
    action_smooth = None  # exponential moving average for smoother motion
    for t in range(args.steps):
        try:
            action_raw = policy.act(state) * args.torque_scale
            if action_smooth is None:
                action_smooth = action_raw.copy()
            else:
                action_smooth = args.smooth * action_smooth + (1.0 - args.smooth) * action_raw
            action = action_smooth
            state, reward, done, _ = env.step(action)
        except Exception as e:
            print(f"Simulation error at step {t+1}: {e}")
            break
        total_reward += reward
        if done and not args.gui:
            print(f"Episode finished at step {t+1}, total reward {total_reward:.2f}")
            break
        if done and args.gui and not episode_ended_printed:
            print(f"Episode ended at step {t+1} (fell); continuing simulation for {args.steps} steps...")
            episode_ended_printed = True
    if not done or not args.gui:
        print(f"Ran {min(t+1, args.steps)} steps, total reward {total_reward:.2f}")
    if not np.isfinite(total_reward):
        print("Warning: total reward was NaN/inf; policy may have overflowed.")
    if args.gui:
        input("Press Enter to close the simulation...")
    env.close()


if __name__ == "__main__":
    main()
