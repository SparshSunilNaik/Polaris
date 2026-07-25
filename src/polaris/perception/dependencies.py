from __future__ import annotations

import os
import site
import sys
from pathlib import Path


def add_homebrew_python_site() -> Path:
    """Expose Homebrew's native Gazebo/OpenCV modules to the project venv."""
    override = os.environ.get("POLARIS_HOMEBREW_PYTHON_SITE")
    path = Path(override) if override else Path(
        f"/opt/homebrew/lib/python{sys.version_info.major}.{sys.version_info.minor}"
        "/site-packages"
    )
    if not path.is_dir():
        raise RuntimeError(
            f"Homebrew Python packages were not found at {path}. "
            "Set POLARIS_HOMEBREW_PYTHON_SITE to the correct directory."
        )
    site.addsitedir(str(path))
    return path
