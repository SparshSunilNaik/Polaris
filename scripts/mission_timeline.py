#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: mission_timeline.py <artifact.json>", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    payload = json.loads(path.read_text(encoding="utf-8"))
    events = payload.get("events")
    if not events:
        print("No mission events recorded in this artifact.")
        return 0
    for event in events:
        print(f"+{float(event['elapsed_s']):07.3f} {event['event_type']:<30} {event['message']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
