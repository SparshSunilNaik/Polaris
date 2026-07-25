from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from polaris.control.models import SafeMotion


def append_supervisor_transition(
    transitions: list[dict[str, Any]],
    previous: tuple[str, str, str] | None,
    safe: SafeMotion,
    waypoint_id: str,
    timestamp_monotonic: float,
) -> tuple[str, str, str]:
    confirmation = asdict(safe.emergency_confirmation)
    confirmation["state"] = safe.emergency_confirmation.state.value
    signature = (
        safe.supervisor_mode.value,
        confirmation["state"],
        safe.emergency_confirmation.reason,
    )
    if signature == previous:
        return signature
    transitions.append(
        {
            "timestamp_monotonic": timestamp_monotonic,
            "waypoint_id": waypoint_id,
            "mode": safe.supervisor_mode.value,
            "reasons": list(safe.reasons),
            "emergency_confirmation": confirmation,
        }
    )
    return signature


def write_mission_artifact(
    payload: dict[str, Any],
    output_dir: str | Path,
    stem: str,
) -> Path:
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{stem}_mission.json"
    path.write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return path
