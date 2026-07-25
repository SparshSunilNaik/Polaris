from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def write_navigation_artifact(
    payload: dict[str, Any],
    output_dir: str | Path,
    stem: str,
) -> Path:
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{stem}_navigation.json"
    path.write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return path
