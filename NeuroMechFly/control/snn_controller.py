"""SNN controller with spike-to-activation decoder for NeuroMechFly.

Replaces the CPG with an external spiking neural network (snnTorch).
Decodes spike trains to phase-like values for muscle compatibility.
"""

import numpy as np

# Actuated joints match neuromuscular_control (Coxa, Femur, Tibia; Coxa_roll for M, H)
SIDES = ('L', 'R')
POSITIONS = ('F', 'M', 'H')
SEGMENTS = ('Coxa', 'Femur', 'Tibia')


def _actuated_joints():
    """Return list of actuated joint names matching neuromuscular_control."""
    joints = []
    for side in SIDES:
        for pos in POSITIONS:
            for seg in SEGMENTS:
                j = f"joint_{side}{pos}{seg}"
                if pos in ('M', 'H') and seg == 'Coxa':
                    j = j.replace('Coxa', 'Coxa_roll')
                joints.append(j)
    return joints


def _neuron_names():
    """Return list of neuron names (flexion/extension per joint)."""
    names = []
    for joint in _actuated_joints():
        names.append(f"{joint}_flexion")
        names.append(f"{joint}_extension")
    return names


def _phase_param_names():
    """Return param names muscles expect: phase_<joint>_flexion, etc."""
    return [
        f"phase_{joint}_{suffix}"
        for joint in _actuated_joints()
        for suffix in ('flexion', 'extension')
    ]


class SpikeToPhaseDecoder:
    """Convert spike rates to phase-like values for muscle activation.

    Muscles expect activation = amp * (1 + sin(phase)).
    This decoder maps spike rates to phase in [-pi, pi].
    Uses leaky integration for smooth output.
    """

    def __init__(self, n_neurons, tau=0.05, scale=2.0):
        """
        Parameters
        ----------
        n_neurons : int
            Number of output neurons.
        tau : float
            Time constant for leaky integration (seconds).
        scale : float
            Scale factor to map integrated rate to phase range.
        """
        self.n_neurons = n_neurons
        self.tau = tau
        self.scale = scale
        self.integrated = np.zeros(n_neurons)

    def step(self, spikes, dt):
        """Update decoder with new spikes and return phase values.

        Parameters
        ----------
        spikes : ndarray, shape (n_neurons,)
            Binary spike values (0 or 1) for this timestep.
        dt : float
            Timestep in seconds.

        Returns
        -------
        phases : ndarray, shape (n_neurons,)
            Phase values in [-pi, pi] for muscle activation.
        """
        # Leaky integration: d/dt x = -x/tau + spikes
        decay = np.exp(-dt / self.tau)
        self.integrated = decay * self.integrated + spikes
        # Map to [-pi, pi] so 1 + sin(phase) gives [0, 2]
        # Use tanh to bound; scale to get oscillation range
        normalized = np.tanh(self.integrated * self.scale)
        phases = np.pi * normalized
        return phases.astype(np.float64)


class SNNController:
    """Controller that runs an external SNN and decodes spikes for muscles.

    Writes to container.neural.states with the same parameter names
    the muscles expect: phase_<joint>_flexion, phase_<joint>_extension,
    amp_<joint>_flexion, amp_<joint>_extension.
    """

    def __init__(self, container, snn_network, time_step=1e-4, decoder_tau=0.05):
        """
        Parameters
        ----------
        container : Container
            FARMS container; will add neural namespace and states.
        snn_network : object
            SNN network with step(dt) -> (spikes_dict or spikes_array).
            Must return spikes for each neuron in _neuron_names() order.
        time_step : float
            Simulation timestep in seconds.
        decoder_tau : float
            Decoder leaky integration time constant.
        """
        self.container = container
        self.snn_network = snn_network
        self.time_step = time_step
        self.neuron_names = _neuron_names()
        self.phase_param_names = _phase_param_names()
        self.actuated_joints = _actuated_joints()
        self.n_neurons = len(self.neuron_names)

        # Add neural namespace and states table (same structure as CPG)
        container.add_namespace('neural')
        neural = container.neural
        neural.add_table('states')

        for name in self.phase_param_names:
            neural.states.add_parameter(name)
        for joint in self.actuated_joints:
            neural.states.add_parameter('amp_' + joint + '_flexion')
            neural.states.add_parameter('amp_' + joint + '_extension')

        self.decoder = SpikeToPhaseDecoder(self.n_neurons, tau=decoder_tau)
        self._amp_default = 1.0

    @property
    def graph(self):
        """Dummy graph for compatibility with DrosophilaSimulation.num_oscillators."""
        class _DummyGraph:
            def number_of_nodes(self):
                return len(self.neuron_names)
        g = _DummyGraph()
        g.neuron_names = self.neuron_names
        return g

    def setup_integrator(self):
        """No-op for compatibility with BulletSimulation.initialize_simulation."""
        pass

    def step(self, dt=None):
        """Step the SNN and write decoded phases to container.

        Parameters
        ----------
        dt : float, optional
            Timestep; uses self.time_step if not provided.
        """
        dt = dt if dt is not None else self.time_step
        spikes = self.snn_network.step(dt)
        phases = self.decoder.step(spikes, dt)

        for i, name in enumerate(self.phase_param_names):
            self.container.neural.states.get_parameter(name).value = phases[i]

        # Set default amp if not already set (muscles use amp for gain)
        for joint in self.actuated_joints:
            for suffix in ('_flexion', '_extension'):
                param = self.container.neural.states.get_parameter(
                    'amp_' + joint + suffix
                )
                if not hasattr(param, '_snn_initialized') or not param._snn_initialized:
                    param.value = self._amp_default
                    param._snn_initialized = True
