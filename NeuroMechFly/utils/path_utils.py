"""Path utilities for NeuroMechFly."""

import pkgutil
from pathlib import Path


def get_neuromechfly_path() -> Path:
    """Return the path to the NeuroMechFly data directory.

    When the package is installed in a venv inside the repo (e.g. ~/NeuroMechFly/venv),
    prefer the repo root so that data/ is found. Otherwise use the package location.
    """
    pkg_path = Path(pkgutil.get_loader("NeuroMechFly").get_filename()).parents[1]
    # If site-packages is inside a venv within the repo, use repo root for data
    # site-packages -> lib -> venv -> repo_root
    repo_root = pkg_path.parent.parent.parent
    data_sdf = repo_root / "data" / "design" / "sdf"
    if data_sdf.exists():
        return repo_root
    return pkg_path
