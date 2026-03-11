"""Integration tests for NeuroMechFly (require full install, PyBullet, etc.)."""
import pytest

# Skip all integration tests if heavy deps are unavailable
pytest.importorskip("pybullet")
pytest.importorskip("farms_container")


def test_import_main_modules():
    """Verify core modules can be imported."""
    from NeuroMechFly.sdf.units import SimulationUnitScaling
    from NeuroMechFly.sdf.utils import joint_name_to_index, find_root
    from NeuroMechFly.control.spring_damper_muscles import Parameters
    assert SimulationUnitScaling is not None
    assert Parameters is not None


def test_model_sdf_read_flies_model():
    """Verify Drosophila SDF model can be loaded if data exists."""
    import os
    from NeuroMechFly.sdf.sdf import ModelSDF
    from NeuroMechFly.sdf.units import SimulationUnitScaling

    # Locate a test SDF - use Drosophila if available
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(pkg_dir, "data", "design", "sdf")
    if not os.path.isdir(data_dir):
        pytest.skip("Data directory not found")

    sdf_files = [f for f in os.listdir(data_dir) if f.endswith(".sdf")]
    if not sdf_files:
        pytest.skip("No SDF files in data/design/sdf")

    # Try to load the first fly model (e.g. Drosophila_*.sdf)
    fly_sdf = None
    for f in sdf_files:
        if "Drosophila" in f or "fly" in f.lower():
            fly_sdf = os.path.join(data_dir, f)
            break
    if fly_sdf is None:
        fly_sdf = os.path.join(data_dir, sdf_files[0])

    models = ModelSDF.read(fly_sdf)
    assert len(models) >= 1
    model = models[0]
    assert model.links
    assert model.joints
    from NeuroMechFly.sdf.utils import find_root
    root = find_root(model)
    assert root in [l.name for l in model.links]
