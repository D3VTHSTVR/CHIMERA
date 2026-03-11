"""Unit tests for NeuroMechFly.control (muscle model, parameters)."""
import pytest
import numpy as np

from NeuroMechFly.control.spring_damper_muscles import Parameters, SDAntagonistMuscle


class TestParameters:
    def test_default_values(self):
        p = Parameters()
        assert p.alpha == 0.0
        assert p.beta == 0.0
        assert p.gamma == 0.0
        assert p.delta == 0.0
        assert p.rest_pos == 0.0

    def test_custom_values(self):
        p = Parameters(alpha=1.0, beta=2.0, gamma=0.5, delta=0.1, rest_pos=0.3)
        assert p.alpha == 1.0
        assert p.beta == 2.0
        assert p.rest_pos == 0.3


class TestMuscleTorqueFormula:
    """Test the Ekeberg torque formula logic in isolation."""

    def _compute_torque(
        self, alpha, beta, gamma, delta, rest_pos,
        jpos, jvel, flexor_phase, extensor_phase, flexor_amp, extensor_amp
    ):
        """Pure implementation of the torque formula for testing."""
        flexor_act = flexor_amp * (1 + np.sin(flexor_phase))
        extensor_act = extensor_amp * (1 + np.sin(extensor_phase))
        passive_stiff = beta * gamma * (rest_pos - jpos)
        damp = delta * jvel
        passive_torque = passive_stiff - damp
        active_co = alpha * (flexor_act - extensor_act)
        active_stiff = beta * (flexor_act + extensor_act)
        active_torque = active_co + active_stiff * (rest_pos - jpos)
        return passive_torque + active_torque

    def test_passive_only_at_rest(self):
        """At rest position with zero velocity, passive torque should be zero."""
        tau = self._compute_torque(
            alpha=1.0, beta=1.0, gamma=1.0, delta=0.1, rest_pos=0.0,
            jpos=0.0, jvel=0.0,
            flexor_phase=0.0, extensor_phase=0.0,
            flexor_amp=0.0, extensor_amp=0.0
        )
        assert abs(tau) < 1e-10

    def test_passive_stiffness_opposes_displacement(self):
        """Positive displacement from rest -> negative (restoring) passive torque."""
        tau = self._compute_torque(
            alpha=0.0, beta=1.0, gamma=1.0, delta=0.0, rest_pos=0.0,
            jpos=0.1, jvel=0.0,
            flexor_phase=0.0, extensor_phase=0.0,
            flexor_amp=0.0, extensor_amp=0.0
        )
        assert tau < 0

    def test_damping_opposes_velocity(self):
        """Positive velocity -> damping adds negative torque."""
        tau = self._compute_torque(
            alpha=0.0, beta=1.0, gamma=1.0, delta=0.5, rest_pos=0.0,
            jpos=0.0, jvel=1.0,
            flexor_phase=0.0, extensor_phase=0.0,
            flexor_amp=0.0, extensor_amp=0.0
        )
        assert tau < 0
