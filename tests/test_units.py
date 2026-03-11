"""Unit tests for NeuroMechFly.sdf.units."""
import pytest

from NeuroMechFly.sdf.units import SimulationUnitScaling


class TestSimulationUnitScaling:
    def test_default_scaling(self):
        u = SimulationUnitScaling()
        assert u.meters == 1
        assert u.seconds == 1
        assert u.kilograms == 1

    def test_derived_units(self):
        u = SimulationUnitScaling(meters=1, seconds=1, kilograms=1)
        assert u.velocity == 1.0
        assert u.acceleration == 1.0
        assert u.newtons == 1.0
        assert u.torques == 1.0
        assert u.volume == 1.0
        assert u.density == 1.0
        assert u.hertz == 1.0

    def test_scaled_velocity(self):
        u = SimulationUnitScaling(meters=2, seconds=1, kilograms=1)
        assert u.velocity == 2.0
