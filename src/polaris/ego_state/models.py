from __future__ import annotations

from dataclasses import asdict, dataclass, field
from time import monotonic, time
from typing import Any

from polaris.perception.models import VisualCondition
from polaris.perception.depth.models import DepthState


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Orientation:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class VisualState:
    camera_connected: bool = False
    frame_age_ms: float = float("inf")
    fps: float = 0.0
    visual_health_score: float = 0.0
    visual_condition: VisualCondition = VisualCondition.CRITICAL
    brightness: float = 0.0
    contrast: float = 0.0
    blur_score: float = 0.0
    texture_score: float = 0.0
    feature_count: int = 0
    feature_coverage: float = 0.0
    overexposed_ratio: float = 0.0
    underexposed_ratio: float = 0.0
    stale: bool = True
    reasons: tuple[str, ...] = ("No camera frame received",)
    last_visual_update_monotonic: float = 0.0


@dataclass
class NavigationState:
    # PX4 local NED frame:
    # x = north, y = east, z = down.
    position_ned_m: Vector3 = field(default_factory=Vector3)
    velocity_ned_m_s: Vector3 = field(default_factory=Vector3)
    orientation: Orientation = field(default_factory=Orientation)
    visual: VisualState = field(default_factory=VisualState)
    depth: DepthState = field(default_factory=DepthState)
    depth_required_for_motion: bool = False

    local_position_ok: bool = False
    global_position_ok: bool = False
    home_position_ok: bool = False
    armable: bool = False

    armed: bool = False
    in_air: bool = False
    flight_mode: str = "UNKNOWN"

    battery_remaining: float | None = None
    battery_voltage_v: float | None = None

    last_position_update_monotonic: float = 0.0
    last_attitude_update_monotonic: float = 0.0
    last_flight_mode_update_monotonic: float = 0.0
    last_armed_update_monotonic: float = 0.0
    last_in_air_update_monotonic: float = 0.0
    updated_at_unix: float = 0.0

    @property
    def altitude_m(self) -> float:
        return -self.position_ned_m.z

    @property
    def horizontal_speed_m_s(self) -> float:
        vn = self.velocity_ned_m_s.x
        ve = self.velocity_ned_m_s.y
        return (vn * vn + ve * ve) ** 0.5

    def position_is_stale(self, timeout_s: float = 1.0) -> bool:
        if self.last_position_update_monotonic == 0.0:
            return True
        return monotonic() - self.last_position_update_monotonic > timeout_s

    def attitude_is_stale(self, timeout_s: float = 1.0) -> bool:
        if self.last_attitude_update_monotonic == 0.0:
            return True
        return monotonic() - self.last_attitude_update_monotonic > timeout_s

    def touch(self) -> None:
        self.updated_at_unix = time()

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["visual"]["visual_condition"] = self.visual.visual_condition.value
        result["depth"]["condition"] = self.depth.condition.value
        result["depth"]["risk_grid"] = [list(row) for row in self.depth.risk_grid]
        return result
