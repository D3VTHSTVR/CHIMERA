#!/usr/bin/env python
"""
Run the trained SNN balance policy (learned walk only).

Usage:
  python train.py --steps 2000 --pop 20 --gen 10   # train first
  python run.py --policy best_policy.npz --gui      # then walk with learned policy
"""

import argparse
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from env.balance_env_mujoco import BalanceEnv
    _env_backend = "MuJoCo"
except ImportError:
    from env import BalanceEnv
    _env_backend = "PyBullet"
from snn.snn_controller import policy_from_params, cpg_snn_controller_from_params


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=str, default="best_policy.npz", help="Path to .npz with 'params' (from train.py)")
    parser.add_argument("--gui", action="store_true", help="Show GUI")
    parser.add_argument("--steps", type=int, default=5000, help="Max steps to run")
    parser.add_argument("--max_torque", type=float, default=None, help="Max joint torque (default: env default)")
    parser.add_argument("--torque_scale", type=float, default=1.0, help="Scale policy output")
    parser.add_argument("--smooth", type=float, default=0.35, help="Action smoothing 0=raw policy, 1=full (default 0.35 so learned walk is visible)")
    parser.add_argument("--no-cpg", action="store_true", help="Use raw SNN only (no CPG)")
    args = parser.parse_args()

    if not os.path.isfile(args.policy):
        print(f"Policy file not found: {args.policy}. Train first: python train.py --steps 2000 --pop 20 --gen 10", file=sys.stderr)
        sys.exit(1)
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
    if args.gui and _env_backend == "MuJoCo":
        print("Using MuJoCo env (torque applied from step 0).", flush=True)
    elif args.gui and _env_backend == "PyBullet":
        print("Using PyBullet env (torque may be delayed by warmup).", flush=True)
    state = env.reset()
    policy.reset()
    total_reward = 0.0
    episode_ended_printed = False
    smooth = args.smooth
    action_smooth = None
    for t in range(args.steps):
        try:
            action_raw = policy.act(state) * args.torque_scale
            if action_smooth is None:
                action_smooth = action_raw.copy()
            else:
                action_smooth = smooth * action_smooth + (1.0 - smooth) * action_raw
            action = action_smooth
            state, reward, done, info = env.step(action)
        except Exception as e:
            print(f"Simulation error at step {t+1}: {e}")
            break
        total_reward += reward
        if done and not args.gui:
            print(f"Episode finished at step {t+1}, total reward {total_reward:.2f}")
            break
        if done and args.gui and not episode_ended_printed:
            reason = info.get("done_reason", "unknown")
            if reason == "fell":
                print(f"Fly fell at step {t+1} (tilt > 57°). Resetting so it can walk again...", flush=True)
                state = env.reset()
                policy.reset()
                episode_ended_printed = False
                total_reward = 0.0
            elif reason == "timeout":
                print(f"Episode ended at step {t+1} (max steps).", flush=True)
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
