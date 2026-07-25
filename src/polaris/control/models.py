from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, fields
from enum import Enum
from pathlib import Path
from typing import Any


class AvoidanceMode(str, Enum):
    CLEAR = "CLEAR"
    CAUTION = "CAUTION"
    AVOID_LEFT = "AVOID_LEFT"
    AVOID_RIGHT = "AVOID_RIGHT"
    RETREAT = "RETREAT"
    HOVER = "HOVER"
    ABORT = "ABORT"
    LAND = "LAND"


class SupervisorMode(str, Enum):
    PASS_THROUGH = "PASS_THROUGH"
    SLOW = "SLOW"
    AVOID_LEFT = "AVOID_LEFT"
    AVOID_RIGHT = "AVOID_RIGHT"
    RETREAT = "RETREAT"
    HOVER = "HOVER"
    ABORT = "ABORT"
    LAND = "LAND"


class EmergencyConfirmationState(str, Enum):
    CLEAR = "CLEAR"
    SUSPECTED = "SUSPECTED"
    CONFIRMED = "CONFIRMED"


class EscapeDirection(str, Enum):
    NONE = "NONE"
    LEFT = "LEFT"
    RIGHT = "RIGHT"


class RecommendedAction(str, Enum):
    CONTINUE = "CONTINUE"
    SLOW = "SLOW"
    SIDESTEP = "SIDESTEP"
    RETREAT = "RETREAT"
    HOVER = "HOVER"
    ABORT = "ABORT"
    LAND = "LAND"


@dataclass(frozen=True)
class AvoidanceConfig:
    control_loop_hz: float = 10.0
    takeoff_altitude_m: float = 2.5
    forward_speed_m_s: float = 0.40
    cautious_speed_m_s: float = 0.16
    sidestep_speed_m_s: float = 0.25
    retreat_speed_m_s: float = 0.20
    climb_speed_m_s: float = 0.18
    yaw_rate_deg_s: float = 15.0
    maximum_speed_m_s: float = 0.60
    caution_threshold: float = 0.45
    stop_threshold: float = 0.62
    emergency_threshold: float = 0.90
    emergency_confirmation_frames: int = 2
    emergency_confirmation_timeout_s: float = 0.80
    emergency_confirmation_max_variance: float = 0.0016
    clear_threshold: float = 0.35
    minimum_escape_margin: float = 0.08
    blocked_persistence_updates: int = 2
    clear_persistence_updates: int = 3
    retreat_persistence_updates: int = 3
    preferred_direction_hold_s: float = 2.0
    sidestep_cooldown_s: float = 1.0
    maximum_avoidance_duration_s: float = 8.0
    maximum_reversals: int = 2
    minimum_visual_health: float = 0.35
    minimum_depth_health: float = 0.40
    minimum_temporal_stability: float = 0.75
    critical_temporal_stability: float = 0.40
    stale_timeout_ms: float = 1200.0
    maximum_state_age_ms: float = 1000.0
    control_loop_timeout_s: float = 0.50
    maximum_displacement_m: float = 14.0
    maximum_run_duration_s: float = 60.0
    maximum_forward_travel_m: float = 12.0
    readiness_depth_samples: int = 5
    start_delay_s: float = 2.0
    failure_hover_s: float = 1.5
    final_hover_s: float = 2.0
    depth_required_for_motion: bool = True
    automatic_landing_on_abort: bool = True
    preferred_turn_direction: str = "LEFT"

    @classmethod
    def from_json(cls, path: str | Path) -> AvoidanceConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown reactive-avoidance settings: {sorted(unknown)}")
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        numeric = [
            value
            for item in fields(self)
            if isinstance((value := getattr(self, item.name)), (int, float))
            and not isinstance(value, bool)
        ]
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("Reactive-avoidance numeric settings must be finite")
        if self.control_loop_hz <= 0 or self.takeoff_altitude_m <= 0:
            raise ValueError("Control rate and takeoff altitude must be positive")
        positive_limits = (
            self.stale_timeout_ms,
            self.maximum_state_age_ms,
            self.control_loop_timeout_s,
            self.emergency_confirmation_timeout_s,
            self.maximum_displacement_m,
            self.maximum_run_duration_s,
            self.maximum_avoidance_duration_s,
            self.maximum_forward_travel_m,
            self.preferred_direction_hold_s,
            self.sidestep_cooldown_s,
            self.failure_hover_s,
            self.final_hover_s,
        )
        if any(value <= 0 for value in positive_limits) or self.start_delay_s < 0:
            raise ValueError("Timeouts, bounds, and durations must be positive")
        speeds = (
            self.forward_speed_m_s,
            self.cautious_speed_m_s,
            self.sidestep_speed_m_s,
            self.retreat_speed_m_s,
            self.climb_speed_m_s,
        )
        if any(speed <= 0 or speed > self.maximum_speed_m_s for speed in speeds):
            raise ValueError("Configured speeds must be positive and within maximum_speed_m_s")
        if not (
            0 <= self.clear_threshold
            < self.caution_threshold
            < self.stop_threshold
            < self.emergency_threshold
            <= 1
        ):
            raise ValueError("Risk thresholds must be ordered clear < caution < stop < emergency")
        unit_values = (
            self.minimum_visual_health,
            self.minimum_depth_health,
            self.minimum_temporal_stability,
            self.critical_temporal_stability,
            self.minimum_escape_margin,
        )
        if any(value < 0 or value > 1 for value in unit_values):
            raise ValueError("Health, stability, and margin settings must be in [0,1]")
        if self.critical_temporal_stability >= self.minimum_temporal_stability:
            raise ValueError("critical_temporal_stability must be below minimum")
        counts = (
            self.blocked_persistence_updates,
            self.clear_persistence_updates,
            self.retreat_persistence_updates,
            self.readiness_depth_samples,
            self.emergency_confirmation_frames,
        )
        if any(count < 1 for count in counts) or self.maximum_reversals < 0:
            raise ValueError("Persistence counts must be positive")
        if self.emergency_confirmation_frames < 2:
            raise ValueError("Emergency confirmation requires at least two frames")
        if self.emergency_confirmation_max_variance < 0:
            raise ValueError("Emergency confirmation variance must be non-negative")
        if self.preferred_turn_direction not in {"LEFT", "RIGHT"}:
            raise ValueError("preferred_turn_direction must be LEFT or RIGHT")


@dataclass(frozen=True)
class DesiredMotion:
    forward_m_s: float = 0.0
    right_m_s: float = 0.0
    down_m_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    source: str = "autonomy"
    timestamp_monotonic: float = 0.0

    @classmethod
    def hover(cls, timestamp_monotonic: float, source: str = "safety") -> DesiredMotion:
        return cls(source=source, timestamp_monotonic=timestamp_monotonic)

    def finite(self) -> bool:
        return all(
            math.isfinite(value)
            for value in (
                self.forward_m_s,
                self.right_m_s,
                self.down_m_s,
                self.yaw_rate_deg_s,
                self.timestamp_monotonic,
            )
        )


@dataclass(frozen=True)
class RiskSummary:
    centre: float
    left: float
    right: float
    upper: float
    lower: float
    forward: float
    valid: bool

    @classmethod
    def from_grid(cls, grid: tuple[tuple[float, float, float], ...]) -> RiskSummary:
        try:
            values = [float(value) for row in grid for value in row]
            valid = len(values) == 9 and all(math.isfinite(value) and 0 <= value <= 1 for value in values)
            if not valid:
                return cls(0, 0, 0, 0, 0, 0, False)
            return cls(
                centre=values[4],
                left=max(values[0], values[3]),
                right=max(values[2], values[5]),
                upper=max(values[0], values[1], values[2]),
                lower=max(values[6], values[7], values[8]),
                forward=max(values[4], 0.5 * values[1]),
                valid=True,
            )
        except (TypeError, ValueError):
            return cls(0, 0, 0, 0, 0, 0, False)


@dataclass
class AvoidanceState:
    mode: AvoidanceMode = AvoidanceMode.CLEAR
    chosen_direction: EscapeDirection = EscapeDirection.NONE
    mode_entered_at: float = 0.0
    direction_chosen_at: float = 0.0
    avoidance_episode_started_at: float = 0.0
    blocked_count: int = 0
    clear_count: int = 0
    left_blocked_count: int = 0
    right_blocked_count: int = 0
    retreat_count: int = 0
    reversal_count: int = 0
    last_depth_sequence: int = 0
    last_risk_grid: tuple[tuple[float, float, float], ...] | None = None
    last_command: DesiredMotion = field(default_factory=DesiredMotion)
    last_transition_reason: str = "Initialized"


@dataclass(frozen=True)
class EmergencyConfirmation:
    state: EmergencyConfirmationState = EmergencyConfirmationState.CLEAR
    observed_frames: int = 0
    required_frames: int = 2
    age_s: float = 0.0
    timeout_s: float = 0.8
    centre_risk: float = 0.0
    variance: float = 0.0
    neighbour_agreement: bool = False
    reason: str = "No emergency proximity evidence"


@dataclass(frozen=True)
class SafeMotion:
    forward_m_s: float
    right_m_s: float
    down_m_s: float
    yaw_rate_deg_s: float
    accepted: bool
    modified: bool
    supervisor_mode: SupervisorMode
    reasons: tuple[str, ...]
    hard_override: bool
    recommended_action: RecommendedAction
    timestamp_monotonic: float
    source_state_age_ms: float
    emergency_confirmation: EmergencyConfirmation = field(
        default_factory=EmergencyConfirmation
    )

    @classmethod
    def hover(
        cls,
        timestamp_monotonic: float,
        mode: SupervisorMode,
        reasons: tuple[str, ...],
        hard_override: bool,
        state_age_ms: float,
        action: RecommendedAction = RecommendedAction.HOVER,
        emergency_confirmation: EmergencyConfirmation | None = None,
    ) -> SafeMotion:
        return cls(
            0.0,
            0.0,
            0.0,
            0.0,
            accepted=False,
            modified=True,
            supervisor_mode=mode,
            reasons=reasons,
            hard_override=hard_override,
            recommended_action=action,
            timestamp_monotonic=timestamp_monotonic,
            source_state_age_ms=state_age_ms,
            emergency_confirmation=(
                EmergencyConfirmation()
                if emergency_confirmation is None
                else emergency_confirmation
            ),
        )


@dataclass(frozen=True)
class AvoidanceDecision:
    mode: AvoidanceMode
    requested_command: DesiredMotion
    proposed_command: DesiredMotion
    risk: RiskSummary
    reasons: tuple[str, ...]
    confidence_mode: str
    timestamp_monotonic: float
    chosen_direction: EscapeDirection
    blocked_count: int
    clear_count: int
    reversal_count: int
    elapsed_avoidance_s: float
    caution_threshold: float
    stop_threshold: float
    emergency_threshold: float
    autonomy_enabled: bool = True
    safe_command: SafeMotion | None = None

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["mode"] = self.mode.value
        result["chosen_direction"] = self.chosen_direction.value
        if self.safe_command is not None:
            result["safe_command"]["supervisor_mode"] = self.safe_command.supervisor_mode.value
            result["safe_command"]["recommended_action"] = self.safe_command.recommended_action.value
        return result
