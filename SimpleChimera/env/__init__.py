# Prefer MuJoCo when available so torque is applied from step 0 (PyBullet env gates torque behind long warmup).
try:
    from .balance_env_mujoco import BalanceEnv
except ImportError:
    from .balance_env import BalanceEnv

__all__ = ["BalanceEnv"]
