"""Spiking CPG network using snnTorch LIF neurons.

Mirrors the topology of the oscillator CPG: flexor-extensor reciprocal
inhibition per joint, intraleg coupling (Coxa->Femur->Tibia), and
interleg coupling for tripod gait.
"""

import numpy as np
import torch
import torch.nn as nn

try:
    import snntorch as snn
except ImportError:
    snn = None

from NeuroMechFly.control.snn_controller import _actuated_joints, _neuron_names


def _build_cpg_connectivity():
    """Build weight matrix for CPG-like connectivity.

    Returns (n_neurons, n_neurons) weight matrix.
    Neuron order: joint_LFCoxa_flexion, joint_LFCoxa_extension, joint_LFFemur_flexion, ...
    """
    names = _neuron_names()
    n = len(names)
    W = np.zeros((n, n))

    def idx(name):
        try:
            return names.index(name)
        except ValueError:
            return None

    # Reciprocal inhibition within each joint (flexor <-> extensor)
    joints = _actuated_joints()
    for i, joint in enumerate(joints):
        base = joint.replace('joint_', '')
        flex = idx(f"joint_{base}_flexion")
        ext = idx(f"joint_{base}_extension")
        if flex is not None and ext is not None:
            W[flex, ext] = -2.0
            W[ext, flex] = -2.0

    # Intraleg coupling: Coxa -> Femur -> Tibia (same leg)
    sides = ('L', 'R')
    positions = ('F', 'M', 'H')
    for side in sides:
        for pos in positions:
            coxa_label = 'Coxa_roll' if pos in ('M', 'H') else 'Coxa'
            chain = [coxa_label, 'Femur', 'Tibia']
            for seg_idx in range(2):
                from_seg = chain[seg_idx]
                to_seg = chain[seg_idx + 1]
                for action in ('flexion', 'extension'):
                    src = idx(f"joint_{side}{pos}{from_seg}_{action}")
                    dst = idx(f"joint_{side}{pos}{to_seg}_{action}")
                    if src is not None and dst is not None:
                        W[dst, src] = 1.0

    # Interleg coupling (tripod: LF-RF, LM-RH, LH-RM)
    interleg = [
        (('L', 'F', 'Coxa'), ('R', 'F', 'Coxa')),
        (('L', 'F', 'Coxa'), ('R', 'M', 'Coxa_roll')),
        (('R', 'M', 'Coxa_roll'), ('L', 'H', 'Coxa_roll')),
        (('R', 'F', 'Coxa'), ('L', 'M', 'Coxa_roll')),
        (('L', 'M', 'Coxa_roll'), ('R', 'H', 'Coxa_roll')),
    ]
    for (s1, p1, s1seg), (s2, p2, s2seg) in interleg:
        for action in ('flexion', 'extension'):
            src = idx(f"joint_{s1}{p1}{s1seg}_{action}")
            dst = idx(f"joint_{s2}{p2}{s2seg}_{action}")
            if src is not None and dst is not None:
                W[dst, src] = 0.8

    # Bias for sustained activity (prevent silence)
    np.fill_diagonal(W, 0.5)

    return W.astype(np.float32)


class SpikingCPGNetwork:
    """Recurrent SNN with CPG topology. Steps return spike array for decoder."""

    def __init__(self, beta=0.85, threshold=1.0, bias=0.3, device='cpu'):
        """
        Parameters
        ----------
        beta : float
            LIF membrane decay (0-1).
        threshold : float
            Firing threshold.
        bias : float
            Constant input current for sustained oscillation.
        device : str
            'cpu' or 'cuda'.
        """
        if snn is None:
            raise ImportError(
                "snnTorch is required for SNN controller. Install with: pip install snntorch"
            )
        self.device = device
        self.n_neurons = len(_neuron_names())

        W = _build_cpg_connectivity()
        self.W = torch.tensor(W, dtype=torch.float32, device=device)
        self.bias = torch.full(
            (self.n_neurons,), bias, dtype=torch.float32, device=device
        )

        self.lif = snn.Leaky(
            beta=beta,
            threshold=threshold,
            init_hidden=False,
        ).to(device)

        self.mem = None
        self._initialized = False

    def _init_state(self):
        # Initialize membrane potential (snn.Leaky expects batch x features)
        self.mem = torch.zeros(
            1, self.n_neurons, device=self.device, dtype=torch.float32
        )
        self._initialized = True

    def step(self, dt):
        """
        Advance simulation by one timestep.

        Parameters
        ----------
        dt : float
            Timestep in seconds (used for potential future substepping).

        Returns
        -------
        spikes : ndarray, shape (n_neurons,)
            Binary spike values (0 or 1) for each neuron.
        """
        if not self._initialized:
            self._init_state()

        # Current = W @ prev_spikes + bias
        # For first step, use small random to break symmetry
        if not hasattr(self, '_prev_spk'):
            self._prev_spk = torch.zeros(
                1, self.n_neurons, device=self.device, dtype=torch.float32
            )
            self._prev_spk[0, 0] = 1.0  # Kick one neuron to start oscillation

        cur = torch.mm(self._prev_spk, self.W.t()) + self.bias
        spk, self.mem = self.lif(cur, self.mem)
        self._prev_spk = spk

        return spk.detach().cpu().numpy().flatten()
