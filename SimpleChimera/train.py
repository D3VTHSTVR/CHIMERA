#!/usr/bin/env python
"""
Train SNN balance policy with a simple evolution strategy.

Usage:
  python train.py --steps 200 --pop 20 --gen 10
  Saves best policy to best_policy.npz
"""

import argparse
import os
import sys
import numpy as np

# Project root
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env.balance_env import BalanceEnv
from snn.snn_controller import (
    make_policy_params,
    policy_from_params,
    param_dim,
    cpg_snn_controller_from_params,
)


def evaluate_policy(env, policy, max_steps=2500):
    """Run one episode; return total reward and steps."""
    state = env.reset()
    policy.reset()
    total_reward = 0.0
    for _ in range(max_steps):
        action = policy.act(state)
        state, reward, done, _ = env.step(action)
        total_reward += reward
        if done:
            break
    # Replace NaN with very bad reward so evolution doesn't use it
    if not np.isfinite(total_reward):
        total_reward = -1e9
    return total_reward, env._step_count


def mutate(params, sigma=0.1):
    """Add Gaussian noise to parameters."""
    return params + np.random.randn(len(params)) * sigma


def train(pop_size=20, n_generations=10, max_steps=2500, sigma=0.08, seed=0, use_cpg=True):
    np.random.seed(seed)
    env = BalanceEnv(gui=False, max_steps=max_steps)
    dim = param_dim()
    # Initial population
    population = [make_policy_params() for _ in range(pop_size)]
    best_params = population[0].copy()
    best_reward = -1e9

    def make_policy(params):
        if use_cpg:
            return cpg_snn_controller_from_params(params)
        return policy_from_params(params)

    for gen in range(n_generations):
        rewards = []
        for i, params in enumerate(population):
            policy = make_policy(params)
            r, steps = evaluate_policy(env, policy, max_steps)
            rewards.append(r)
            if r > best_reward:
                best_reward = r
                best_params = params.copy()
                print(f"  gen {gen+1} ind {i}: new best reward {r:.2f} (steps {steps})", flush=True)
        rewards = np.array(rewards)
        # Ignore NaN for stats (some policies can still overflow in edge cases)
        mean_r = np.nanmean(rewards)
        max_r = np.nanmax(rewards) if np.any(np.isfinite(rewards)) else best_reward
        # Next gen: keep best, mutate rest from best + random
        population = [best_params.copy()]
        for _ in range(pop_size - 1):
            population.append(mutate(best_params, sigma=sigma))
        print(f"gen {gen+1}/{n_generations} mean_reward={mean_r:.2f} max={max_r:.2f}", flush=True)

    env.close()
    return best_params, best_reward


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=2000, help="Max steps per episode")
    parser.add_argument("--pop", type=int, default=20, help="Population size")
    parser.add_argument("--gen", type=int, default=10, help="Generations")
    parser.add_argument("--sigma", type=float, default=0.08, help="Mutation std")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=str, default="best_policy.npz", help="Output file")
    parser.add_argument("--no-cpg", action="store_true", help="Train raw SNN only (no CPG)")
    args = parser.parse_args()

    print("Training SNN balance policy (evolution strategy)" + (" with CPG march" if not args.no_cpg else " raw SNN") + "...", flush=True)
    best_params, best_reward = train(
        pop_size=args.pop,
        n_generations=args.gen,
        max_steps=args.steps,
        sigma=args.sigma,
        seed=args.seed,
        use_cpg=not args.no_cpg,
    )
    np.savez(args.out, params=best_params, reward=best_reward)
    print(f"Saved best policy to {args.out} (reward={best_reward:.2f})", flush=True)


if __name__ == "__main__":
    main()
