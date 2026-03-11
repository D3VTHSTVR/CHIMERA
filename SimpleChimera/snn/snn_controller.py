"""
Minimal Leaky Integrate-and-Fire (LIF) SNN for balance control.

State (40) -> input layer -> LIF hidden (32) -> readout -> 18 torques (3 DOF per leg).
No backprop: parameters are optimized by evolution.
"""

import numpy as np

STATE_DIM = 40   # 4 base + 18 joint pos + 18 joint vel
ACTION_DIM = 18  # 6 legs × 3 (Coxa, Femur, Tibia)
HIDDEN_DIM = 32


class SNNController:
    """
    LIF SNN: state -> W_in -> membrane V -> spike -> W_out -> action (torques).
    One update per env step; output is tanh(W_out @ V) for smooth torques.
    """

    def __init__(self, W_in, W_out, tau=0.02, threshold=1.0):
        # W_in: (HIDDEN_DIM, STATE_DIM), W_out: (HIDDEN_DIM, ACTION_DIM)
        self.W_in = np.asarray(W_in, dtype=np.float64)
        self.W_out = np.asarray(W_out, dtype=np.float64)
        self.tau = float(tau)
        self.threshold = float(threshold)
        self.V = np.zeros(HIDDEN_DIM, dtype=np.float64)

    def reset(self):
        self.V.fill(0.0)

    def act(self, state):
        state = np.asarray(state, dtype=np.float64).ravel()
        if state.size != STATE_DIM:
            state = np.resize(state, STATE_DIM)
        # Normalize state to prevent large I = W_in @ state (40 dims can overflow)
        state = np.clip(state, -5, 5)
        # Scale input by 1/sqrt(STATE_DIM) so I stays bounded
        I = (self.W_in @ state) / np.sqrt(STATE_DIM)
        I = np.clip(I, -10.0, 10.0)
        # LIF: dV/dt = (I - V)/tau
        tau = max(self.tau, 0.001)
        self.V = self.V + (I - self.V) / tau
        # Clamp V to prevent overflow/NaN
        self.V = np.clip(self.V, -20.0, 20.0)
        self.V = np.nan_to_num(self.V, nan=0.0, posinf=20.0, neginf=-20.0)
        # Soft spike (smooth): rate proportional to V above 0
        rate = np.maximum(0, np.minimum(self.V, self.threshold))
        # Output: tanh(rate @ W_out) -> (18,) in [-1, 1]
        out = np.tanh(rate @ self.W_out)
        out = np.nan_to_num(out, nan=0.0, posinf=1.0, neginf=-1.0)
        return out.astype(np.float32)


def make_policy_params(scale_in=0.08, scale_out=0.2):
    """Random policy parameters (for evolution). Returns flat vector."""
    # Smaller scale_in with 40-dim state to keep I = W_in@state from overflowing
    W_in = np.random.randn(HIDDEN_DIM, STATE_DIM) * scale_in
    W_out = np.random.randn(HIDDEN_DIM, ACTION_DIM) * scale_out
    tau = 0.02
    threshold = 1.0
    return np.concatenate([W_in.ravel(), W_out.ravel(), [tau, threshold]])


def policy_from_params(params):
    """Build SNNController from flat parameter vector."""
    params = np.asarray(params, dtype=np.float64).ravel()
    n_in = HIDDEN_DIM * STATE_DIM
    n_out = HIDDEN_DIM * ACTION_DIM
    W_in = params[:n_in].reshape(HIDDEN_DIM, STATE_DIM)
    W_out = params[n_in : n_in + n_out].reshape(HIDDEN_DIM, ACTION_DIM)
    tau = params[n_in + n_out] if len(params) > n_in + n_out else 0.02
    threshold = params[n_in + n_out + 1] if len(params) > n_in + n_out + 1 else 1.0
    tau = np.clip(tau, 0.005, 0.1)
    threshold = np.clip(threshold, 0.5, 2.0)
    return SNNController(W_in, W_out, tau=tau, threshold=threshold)


def param_dim():
    """Length of flat parameter vector."""
    return HIDDEN_DIM * STATE_DIM + HIDDEN_DIM * ACTION_DIM + 2


# --- CPG + SNN combined controller ---

def cpg_snn_controller_from_params(params, cpg_freq_hz=0.22, cpg_amplitude=0.08, cpg_weight=0.4, snn_weight=0.3):
    """
    Build a combined CPG+SNN controller.
    CPG drives the alternating tripod march; SNN adds balance modulation.
    """
    from .cpg import TripodCPG
    snn = policy_from_params(params)
    cpg = TripodCPG(freq_hz=cpg_freq_hz, amplitude=cpg_amplitude)
    return CPGSNNController(cpg, snn, cpg_weight=cpg_weight, snn_weight=snn_weight)


class CPGSNNController:
    """
    CPG produces march rhythm; SNN produces balance correction.
    action = clip(cpg_weight * cpg_output + snn_weight * snn_output, -1, 1)
    """

    def __init__(self, cpg, snn, cpg_weight=0.6, snn_weight=0.5):
        self.cpg = cpg
        self.snn = snn
        self.cpg_weight = float(cpg_weight)
        self.snn_weight = float(snn_weight)

    def reset(self):
        self.cpg.reset()
        self.snn.reset()

    def act(self, state):
        cpg_out = self.cpg.get_torque_signal()
        self.cpg.step()
        snn_out = self.snn.act(state)
        combined = self.cpg_weight * cpg_out + self.snn_weight * snn_out
        return np.clip(combined, -1.0, 1.0).astype(np.float32)
