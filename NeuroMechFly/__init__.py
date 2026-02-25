"""NeuroMechFly: Neuromechanical simulation of Drosophila melanogaster.

A data-driven computational model for synthesizing experimental datasets
and testing theories of neuromechanical behavioral control. Uses PyBullet
for physics, FARMS Network for neural control (CPG/SNN), and Ekeberg-style
muscle models for actuation.

Subpackages:
    control: Muscle model, CPG, SNN controller
    experiments: Kinematic replay, neuromuscular optimization
    sdf: SDF parsing and PyBullet loading
    simulation: PyBullet simulation core
    utils: Plotting, profiling, path utilities
"""
# See PEP 440 for suitable version numbers.
__version__ = '1.0.dev0'
