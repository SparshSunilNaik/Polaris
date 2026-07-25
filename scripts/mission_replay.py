#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from polaris.mission.replay import (  # noqa: E402
    format_mission_replay,
    validate_mission_replay_payload,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a read-only Project Polaris mission artifact replay"
    )
    parser.add_argument("artifact", type=Path, help="Path to a mission artifact JSON file")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    path = args.artifact.expanduser()
    if not path.is_file():
        print(f"mission_replay: file does not exist: {path}", file=sys.stderr)
        return 2
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        print(f"mission_replay: invalid JSON in {path}: {exc}", file=sys.stderr)
        return 3
    except OSError as exc:
        print(f"mission_replay: unable to read {path}: {exc}", file=sys.stderr)
        return 2
    try:
        validate_mission_replay_payload(payload)
    except ValueError as exc:
        print(f"mission_replay: malformed artifact {path}: {exc}", file=sys.stderr)
        return 4
    print(format_mission_replay(payload, path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
