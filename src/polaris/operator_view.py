from __future__ import annotations

import json
import textwrap
from dataclasses import dataclass, fields
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

from polaris.ego_state.confidence import ConfidenceReport, SafetyMode
from polaris.ego_state.models import NavigationState
from polaris.perception.dependencies import add_homebrew_python_site
from polaris.perception.models import CameraFrame

if TYPE_CHECKING:
    from polaris.control.models import AvoidanceDecision
    from polaris.ego_context.models import EgoContextSnapshot
    from polaris.navigation.models import GoalDecision, RecoveryDecision
    from polaris.simulation.collision_monitor import CollisionState
    from polaris.mission.models import MissionState, ObservationDecision
    from polaris.ego_context.semantic_validation import SemanticValidationRecord

add_homebrew_python_site()

import cv2  # noqa: E402
import numpy as np  # noqa: E402


WINDOW_TITLE = "Project Polaris Operator View"

BACKGROUND = (23, 6, 2)
PANEL = (42, 23, 15)
PANEL_ALT = (59, 41, 30)
BORDER = (85, 65, 51)
TEXT = (252, 250, 248)
MUTED = (184, 163, 148)
CYAN = (238, 211, 34)
GREEN = (94, 197, 34)
YELLOW = (21, 204, 250)
ORANGE = (22, 115, 249)
RED = (68, 68, 239)


@dataclass(frozen=True)
class OperatorViewConfig:
    window_width: int = 1600
    window_height: int = 900
    target_display_fps: float = 10.0
    depth_device: str = "auto"
    target_depth_fps: float = 3.0
    rgb_grid_overlay: bool = True
    screenshot_output_dir: str = "artifacts/operator_screenshots"
    visual_config: str = "config/visual_health.json"
    depth_config: str = "config/depth_anything.json"
    confidence_config: str = "config/confidence.json"

    @classmethod
    def from_json(cls, path: str | Path) -> OperatorViewConfig:
        values = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unknown Operator View settings: {sorted(unknown)}")
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        if self.window_width < 1000 or self.window_height < 650:
            raise ValueError("Operator View must be at least 1000x650")
        if self.target_display_fps <= 0 or self.target_depth_fps <= 0:
            raise ValueError("Display and depth FPS targets must be positive")
        if self.depth_device not in {"auto", "mps", "cpu"}:
            raise ValueError("depth_device must be auto, mps, or cpu")


def screenshot_path(
    output_dir: str | Path,
    timestamp: datetime | None = None,
) -> Path:
    stamp = timestamp or datetime.now().astimezone()
    return Path(output_dir) / f"{stamp.strftime('%Y%m%dT%H%M%S_%f')[:-3]}_operator.png"


def is_quit_key(key: int) -> bool:
    return key in (ord("q"), 27)


def format_ego_lines(state: NavigationState) -> list[str]:
    position = state.position_ned_m
    velocity = state.velocity_ned_m_s
    attitude = state.orientation
    battery_percent = (
        "unknown"
        if state.battery_remaining is None
        else f"{state.battery_remaining * 100:.1f}%"
    )
    battery_voltage = (
        "unknown" if state.battery_voltage_v is None else f"{state.battery_voltage_v:.1f} V"
    )
    battery = f"{battery_percent} / {battery_voltage}"
    return [
        f"Position NED   N {position.x:+7.2f}  E {position.y:+7.2f}  D {position.z:+7.2f} m",
        f"Velocity NED   N {velocity.x:+6.2f}  E {velocity.y:+6.2f}  D {velocity.z:+6.2f} m/s",
        f"Altitude {state.altitude_m:6.2f} m    Horizontal speed {state.horizontal_speed_m_s:5.2f} m/s",
        f"Attitude       R {attitude.roll_deg:+6.1f}  P {attitude.pitch_deg:+6.1f}  Y {attitude.yaw_deg:+6.1f} deg",
        f"Armed {yes_no(state.armed)}    In air {yes_no(state.in_air)}    Armable {yes_no(state.armable)}",
        f"Local position {ok_bad(state.local_position_ok)}    Position stale {yes_no(state.position_is_stale())}",
        f"Attitude stale {yes_no(state.attitude_is_stale())}    Battery {battery}",
    ]


def format_visual_lines(state: NavigationState) -> list[str]:
    visual = state.visual
    age = "never" if not np.isfinite(visual.frame_age_ms) else f"{visual.frame_age_ms:.0f} ms"
    return [
        f"Condition {visual.visual_condition.value:<8}   Score {visual.visual_health_score:.3f}",
        f"Camera FPS {visual.fps:5.1f}   Frame age {age}   Stale {yes_no(visual.stale)}",
        f"Brightness {visual.brightness:6.1f}   Contrast {visual.contrast:6.1f}",
        f"Blur {visual.blur_score:7.1f}   Texture {visual.texture_score:6.2f}",
        f"Features {visual.feature_count:4d}   Coverage {visual.feature_coverage:.3f}",
        f"Overexposed {visual.overexposed_ratio:.3f}   Underexposed {visual.underexposed_ratio:.3f}",
    ]


def format_depth_lines(state: NavigationState, depth_enabled: bool) -> list[str]:
    if not depth_enabled:
        return ["Depth disabled by --no-depth", "RGB and Ego State remain active"]
    depth = state.depth
    return [
        f"Condition {depth.condition.value:<8}   Score {depth.depth_health_score:.3f}",
        f"Latency {depth.inference_latency_ms:6.1f} ms   Inference FPS {depth.inference_fps:4.2f}",
        f"Source age {depth.source_frame_age_ms:6.1f} ms   Stale {yes_no(depth.stale)}",
        f"Stability {depth.temporal_stability:.3f}   Device {depth.device or 'loading'}",
        f"Nearest {depth.nearest_region:<10}   Safest {depth.safest_region:<10}",
        f"Centre {depth.centre_risk:.3f}   Left {depth.left_risk:.3f}   Right {depth.right_risk:.3f}",
    ]


def format_safety_lines(
    report: ConfidenceReport,
    depth_enabled: bool = True,
) -> list[str]:
    depth_health = f"{report.depth_health:.3f}" if depth_enabled else "DISABLED"
    depth_freshness = f"{report.depth_freshness:.0f}" if depth_enabled else "DISABLED"
    return [
        f"Overall {report.overall_health:.3f}   Visual {report.visual_health:.3f}   Depth {depth_health}",
        f"Localization {report.localization_health:.3f}   Telemetry {report.telemetry_health:.3f}",
        f"Battery {report.battery_health:.3f}   Motion {report.motion_health:.3f}",
        f"Camera fresh {report.camera_freshness:.0f}   Depth fresh {depth_freshness}",
    ]


def format_avoidance_lines(decision: AvoidanceDecision) -> list[str]:
    safe = decision.safe_command
    supervisor = "PENDING" if safe is None else safe.supervisor_mode.value
    issued = (0.0, 0.0, 0.0, 0.0) if safe is None else (
        safe.forward_m_s,
        safe.right_m_s,
        safe.down_m_s,
        safe.yaw_rate_deg_s,
    )
    hard = False if safe is None else safe.hard_override
    reasons = decision.reasons if safe is None else safe.reasons
    reason = reasons[0] if reasons else "None"
    requested = decision.requested_command
    confirmation = None if safe is None else safe.emergency_confirmation
    emergency = (
        "PENDING 0/0"
        if confirmation is None
        else f"{confirmation.state.value} {confirmation.observed_frames}/{confirmation.required_frames}"
    )
    confirmation_reason = (
        reason
        if confirmation is None or confirmation.state.value == "CLEAR"
        else confirmation.reason
    )
    displayed_centre = (
        decision.risk.centre
        if confirmation is None or confirmation.state.value == "CLEAR"
        else confirmation.centre_risk
    )
    return [
        f"Enabled {yes_no(decision.autonomy_enabled)}   Mode {decision.mode.value}",
        f"Supervisor {supervisor}   Emergency {emergency}",
        f"Requested F {requested.forward_m_s:+.2f}  R {requested.right_m_s:+.2f}",
        f"Issued    F {issued[0]:+.2f}  R {issued[1]:+.2f}  D {issued[2]:+.2f}  Y {issued[3]:+.1f}",
        f"Thresholds C {decision.caution_threshold:.2f}  S {decision.stop_threshold:.2f}  E {decision.emergency_threshold:.2f}",
        f"Centre {displayed_centre:.3f}   Age {0.0 if confirmation is None else confirmation.age_s:.1f} s   Variance {0.0 if confirmation is None else confirmation.variance:.4f}",
        f"Hard override {yes_no(hard)}   Elapsed {decision.elapsed_avoidance_s:.1f} s",
        f"Reason {textwrap.shorten(confirmation_reason, width=46, placeholder='...')}",
    ]


def avoidance_hard_override_banner(decision: AvoidanceDecision | None) -> str | None:
    if (
        decision is None
        or decision.safe_command is None
        or not decision.safe_command.hard_override
    ):
        return None
    return (
        "SAFETY SUPERVISOR HARD OVERRIDE - "
        f"{decision.safe_command.supervisor_mode.value}"
    )


def format_navigation_lines(
    goal: GoalDecision,
    recovery: RecoveryDecision | None,
    collision: CollisionState | None,
    avoidance: AvoidanceDecision | None,
) -> list[str]:
    metrics = goal.metrics
    goal_result = (
        "REACHED" if goal.goal_reached else "FAILED" if goal.failed else "ACTIVE"
    )
    recovery_state = "IDLE" if recovery is None else recovery.state.value
    recovery_action = "NONE" if recovery is None else recovery.action.value
    selected = (
        "NONE"
        if recovery is None or recovery.selected_heading_deg is None
        else f"{recovery.selected_heading_deg:+.0f} deg"
    )
    candidate_text = "none"
    if recovery is not None and recovery.candidates:
        candidate_text = " ".join(
            f"{item.heading_deg:+.0f}:{item.median_score:.2f}"
            for item in recovery.candidates[-5:]
        )
    collision_detected = collision is not None and collision.collision_detected
    collision_count = 0 if collision is None else collision.collision_count
    contacted = "NONE" if collision is None else collision.latest_contacted_entity
    avoidance_mode = "NONE" if avoidance is None else avoidance.mode.value
    supervisor = (
        "PENDING"
        if avoidance is None or avoidance.safe_command is None
        else avoidance.safe_command.supervisor_mode.value
    )
    emergency = (
        "PENDING"
        if avoidance is None or avoidance.safe_command is None
        else avoidance.safe_command.emergency_confirmation.state.value
    )
    return [
        f"Goal {goal.goal.goal_id}   State {goal.state.value}   Result {goal_result}",
        f"Target N {goal.goal.target_north_m:+.1f} E {goal.goal.target_east_m:+.1f} D {goal.goal.target_down_m:+.1f} m",
        f"Distance {metrics.goal_distance_m:.2f} m   Bearing {metrics.goal_bearing_deg:+.1f} deg   Progress {metrics.progress_over_window_m:+.2f} m",
        f"No progress {metrics.time_without_progress_s:.1f} s   Path {metrics.path_length_proxy_m:.2f} m   Displacement {metrics.displacement_from_start_m:.2f} m",
        f"Hold error {metrics.hold_position_error_m:.2f} m   Cross-track {metrics.cross_track_error_m:.2f} m",
        f"Recovery {recovery_state}/{recovery_action}   Attempt {0 if recovery is None else recovery.attempt_count}   Selected {selected}",
        f"View scores {textwrap.shorten(candidate_text, width=62, placeholder='...')}",
        f"Collision {yes_no(collision_detected)} count {collision_count}   Entity {textwrap.shorten(contacted, width=34, placeholder='...')}",
        f"Avoidance {avoidance_mode}   Supervisor {supervisor}   Emergency {emergency}",
    ]


def format_mission_lines(
    mission: MissionState,
    goal: GoalDecision,
    recovery: RecoveryDecision | None,
    collision: CollisionState | None,
    avoidance: AvoidanceDecision | None,
    *,
    observation: ObservationDecision | None = None,
    ego_context: EgoContextSnapshot | None = None,
) -> list[str]:
    selected = (
        "NONE"
        if recovery is None or recovery.selected_heading_deg is None
        else f"{recovery.selected_heading_deg:+.0f} deg"
    )
    recovery_state = "IDLE" if recovery is None else recovery.state.value
    collision_count = 0 if collision is None else collision.collision_count
    contacted = "NONE" if collision is None else collision.latest_contacted_entity
    supervisor = (
        "PENDING"
        if avoidance is None or avoidance.safe_command is None
        else avoidance.safe_command.supervisor_mode.value
    )
    avoidance_mode = "NONE" if avoidance is None else avoidance.mode.value
    avoidance_elapsed = 0.0 if avoidance is None else avoidance.elapsed_avoidance_s
    safe = None if avoidance is None else avoidance.safe_command
    command = goal.requested_motion if safe is None else safe
    reasons = (
        goal.reasons
        if avoidance is None
        else avoidance.reasons
        if safe is None
        else safe.reasons
    )
    safety_reason = mission.abort_reason or (reasons[0] if reasons else "None")
    confirmation = None if safe is None else safe.emergency_confirmation
    emergency = (
        "PENDING 0/0"
        if confirmation is None
        else f"{confirmation.state.value} {confirmation.observed_frames}/{confirmation.required_frames}"
    )
    centre_risk = (
        0.0
        if avoidance is None
        else avoidance.risk.centre
        if confirmation is None or confirmation.state.value == "CLEAR"
        else confirmation.centre_risk
    )
    context_revision = "" if ego_context is None else f"   Ctx r{ego_context.revision}"
    context_strategy = (
        "" if ego_context is None else f"/{ego_context.navigation.strategy}"
    )
    context_counts = (
        ""
        if ego_context is None
        else "   C/S/F "
        f"{len(ego_context.mission.completed_waypoints)}/"
        f"{len(ego_context.mission.skipped_waypoints)}/"
        f"{len(ego_context.mission.failed_waypoints)}"
    )
    scene = _format_scene_context(ego_context)
    if observation is not None:
        return [
            f"Mission {mission.mission_name}   {mission.state.value}/{mission.result.value}   Timer {mission.mission_elapsed_s:.1f} s{context_revision}",
            f"Waypoint {mission.current_waypoint_index + 1}/{mission.total_waypoints} {mission.current_waypoint_id}   Type OBSERVATION   Completion {mission.current_completion_mode}",
            f"Observation {observation.state.value}   Band {observation.config.minimum_goal_distance_m:.1f}-{observation.config.maximum_goal_distance_m:.1f} m   Goal {observation.goal_distance_m:.2f} m",
            f"Heading {observation.heading_error_deg:+.1f} deg   Dwell {observation.dwell_elapsed_s:.1f}/{observation.config.required_stable_s:.1f} s   Frames {observation.fresh_observation_count}/{observation.config.minimum_fresh_observations}",
            f"Health V {observation.visual_health:.2f} D {observation.depth_health:.2f} S {observation.depth_stability:.2f}   Centre {observation.centre_risk:.3f}",
            f"Command F {command.forward_m_s:+.2f} R {command.right_m_s:+.2f} D {command.down_m_s:+.2f} Y {command.yaw_rate_deg_s:+.1f}   Supervisor {supervisor}",
            f"Avoidance {avoidance_mode}   Recovery {recovery_state}   {scene}",
            f"Collision {collision_count} {textwrap.shorten(contacted, width=24)}   Landing {mission.landing_result}{context_counts}",
        ]
    return [
        f"Mission {mission.mission_name}   {mission.state.value}/{mission.result.value}   Timer {mission.mission_elapsed_s:.1f} s{context_revision}",
        f"Waypoint {mission.current_waypoint_index + 1}/{mission.total_waypoints} {mission.current_waypoint_id}   Attempt {mission.current_attempt}   Progress {mission.progress_percent:.0f}%",
        f"Navigator {goal.state.value}{context_strategy}   Goal {goal.metrics.goal_distance_m:.2f} m   Cross-track {goal.metrics.cross_track_error_m:.2f} m",
        f"Avoidance {avoidance_mode} {avoidance_elapsed:.1f} s   Recovery {recovery_state} {selected}   Supervisor {supervisor}",
        f"Emergency {emergency}   Age {0.0 if confirmation is None else confirmation.age_s:.1f} s   Centre {centre_risk:.3f}",
        f"Command F {command.forward_m_s:+.2f} R {command.right_m_s:+.2f} D {command.down_m_s:+.2f} Y {command.yaw_rate_deg_s:+.1f}",
        f"Safety reason {textwrap.shorten(safety_reason, width=24, placeholder='...')}   {scene}",
        f"Collision {collision_count} {textwrap.shorten(contacted, width=24)}   Landing {mission.landing_result}{context_counts}",
    ]


def format_ego_context_lines(snapshot: EgoContextSnapshot) -> list[str]:
    latest = (
        "NONE"
        if not snapshot.recent_events
        else snapshot.recent_events[-1].message
    )
    return [
        f"Context r{snapshot.revision}   Events {len(snapshot.recent_events)}",
        f"Objective {textwrap.shorten(snapshot.mission.current_objective, width=48, placeholder='...')}   Completion {snapshot.mission.completion_mode}",
        f"Navigation {snapshot.navigation.strategy}   Safety {snapshot.safety.supervisor_state}",
        _format_scene_context(snapshot),
        f"Raw scene {textwrap.shorten(snapshot.perception.scene_summary, width=58, placeholder='...')}",
        f"Latest {textwrap.shorten(latest, width=62, placeholder='...')}",
    ]


def _format_scene_context(snapshot: EgoContextSnapshot | None) -> str:
    if snapshot is None:
        return "VLM DISABLED"
    perception = snapshot.perception
    fused = perception.fused_scene
    target = "?" if fused.target_visible is None else yes_no(fused.target_visible)
    landmarks = ",".join(fused.visible_landmarks[:3]) or "NONE"
    validation = format_semantic_validation_line(
        perception.latest_semantic_validation
    )
    return textwrap.shorten(
        f"VLM {perception.vlm_status} FUSE {fused.status.value} "
        f"T{target} E{fused.evidence_count} {validation} L{landmarks}",
        width=72,
        placeholder="...",
    )


def format_semantic_validation_line(
    record: SemanticValidationRecord | None,
) -> str:
    if record is None:
        return "VAL NONE"
    target = record.expected_target_label or "UNSPECIFIED"
    return (
        f"VAL {record.classification.value} {target}/"
        f"{record.evidence_source.value}"
    )


def yes_no(value: bool) -> str:
    return "YES" if value else "NO"


def ok_bad(value: bool) -> str:
    return "OK" if value else "BAD"


def risk_color(value: float) -> tuple[int, int, int]:
    risk = float(np.clip(value, 0.0, 1.0))
    if risk <= 0.5:
        ratio = risk * 2.0
        clear = np.array((180, 105, 35), dtype=np.float32)
        caution = np.array(YELLOW, dtype=np.float32)
        color = clear * (1.0 - ratio) + caution * ratio
    else:
        ratio = (risk - 0.5) * 2.0
        caution = np.array(YELLOW, dtype=np.float32)
        danger = np.array(RED, dtype=np.float32)
        color = caution * (1.0 - ratio) + danger * ratio
    return tuple(int(channel) for channel in color)


def render_risk_panel(
    width: int,
    height: int,
    state: NavigationState,
    depth_enabled: bool = True,
) -> np.ndarray:
    panel = np.full((height, width, 3), PANEL, dtype=np.uint8)
    _title(panel, "3x3 RELATIVE PROXIMITY RISK", 14, 28)
    _text(panel, "0 = CLEAR / FAR     1 = NEAR / HIGH CONCERN", (14, 49), 0.40, MUTED)
    if not depth_enabled:
        _placeholder(panel, "DEPTH DISABLED", "Run without --no-depth to enable risk")
        return panel
    depth = state.depth
    if not depth.available or depth.stale:
        label = "DEPTH STALE" if depth.stale and depth.inference_timestamp_monotonic else "DEPTH UNAVAILABLE"
        _placeholder(panel, label, _first_reason(depth.reasons))
        return panel

    compact = height < 175
    left = 14
    top = 55 if compact else 62
    gap = 5
    cell_width = (width - left * 2 - gap * 2) // 3
    cell_height = max(1, (height - top - 8 - gap * 2) // 3)
    names = (
        ("UP_LEFT", "UP", "UP_RIGHT"),
        ("LEFT", "CENTRE", "RIGHT"),
        ("DOWN_LEFT", "DOWN", "DOWN_RIGHT"),
    )
    for row in range(3):
        for column in range(3):
            value = state.depth.risk_grid[row][column]
            x0 = left + column * (cell_width + gap)
            y0 = top + row * (cell_height + gap)
            x1 = x0 + cell_width
            y1 = y0 + cell_height
            cv2.rectangle(panel, (x0, y0), (x1, y1), risk_color(value), -1)
            region = names[row][column]
            thickness = 3 if region == depth.nearest_region else 1
            border = TEXT if region == depth.nearest_region else BORDER
            cv2.rectangle(panel, (x0, y0), (x1, y1), border, thickness)
            marker = "HIGH " if region == depth.nearest_region else "SAFE " if region == depth.safest_region else ""
            label_y = y0 + (10 if compact else 18)
            value_y = y1 - (3 if compact else 8)
            _text(panel, marker + region, (x0 + 5, label_y), 0.25 if compact else 0.38, TEXT, 1)
            _text(panel, f"{value:.2f}", (x0 + 5, value_y), 0.34 if compact else 0.55, TEXT, 1 if compact else 2)
    return panel


class OperatorRenderer:
    def __init__(self, config: OperatorViewConfig) -> None:
        self.config = config

    def compose(
        self,
        state: NavigationState,
        report: ConfidenceReport,
        frame: CameraFrame | None,
        proximity: np.ndarray | None,
        *,
        display_fps: float,
        depth_enabled: bool,
        grid_overlay: bool,
        paused: bool,
        help_visible: bool,
        avoidance: AvoidanceDecision | None = None,
        navigation: GoalDecision | None = None,
        recovery: RecoveryDecision | None = None,
        collision: CollisionState | None = None,
        mission: MissionState | None = None,
        observation: ObservationDecision | None = None,
        ego_context: EgoContextSnapshot | None = None,
    ) -> np.ndarray:
        width = self.config.window_width
        height = self.config.window_height
        canvas = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
        header_height = max(48, int(height * 0.06))
        image_bottom = int(height * 0.565)
        half = width // 2

        self._header(canvas, report, display_fps, header_height, paused)
        self._image_panel(
            canvas,
            (0, header_height, half, image_bottom),
            "LIVE RGB / FORWARD CAMERA",
            self._rgb_image(frame, state, half, image_bottom - header_height, grid_overlay),
        )
        self._image_panel(
            canvas,
            (half, header_height, width, image_bottom),
            "RELATIVE PSEUDO-DEPTH - NOT METRIC DISTANCE",
            self._depth_image(proximity, state, half, image_bottom - header_height, depth_enabled),
        )

        bottom_height = height - image_bottom
        row_height = bottom_height // 2
        risk_width = int(width * 0.34)
        ego_width = int(width * 0.66)
        risk = render_risk_panel(risk_width, row_height, state, depth_enabled)
        canvas[image_bottom : image_bottom + row_height, 0:risk_width] = risk
        self._text_panel(
            canvas,
            (risk_width, image_bottom, ego_width, image_bottom + row_height),
            "FLIGHT / EGO STATE",
            format_ego_lines(state),
        )
        self._safety_panel(
            canvas,
            (ego_width, image_bottom, width, image_bottom + row_height),
            report,
            depth_enabled,
        )
        self._text_panel(
            canvas,
            (0, image_bottom + row_height, half, height),
            "VISUAL HEALTH",
            format_visual_lines(state),
        )
        if navigation is not None:
            quarter = width // 4
            self._text_panel(
                canvas,
                (0, image_bottom + row_height, quarter, height),
                "VISUAL HEALTH",
                format_visual_lines(state),
            )
            self._text_panel(
                canvas,
                (quarter, image_bottom + row_height, quarter * 2, height),
                "DEPTH HEALTH",
                format_depth_lines(state, depth_enabled),
            )
            self._text_panel(
                canvas,
                (quarter * 2, image_bottom + row_height, width, height),
                (
                    "MISSION / LOCAL GOAL / RECOVERY / COLLISION"
                    if mission is not None
                    else "LOCAL GOAL / ACTIVE PERCEPTION / COLLISION"
                ),
                (
                    format_mission_lines(
                        mission,
                        navigation,
                        recovery,
                        collision,
                        avoidance,
                        observation=observation,
                        ego_context=ego_context,
                    )
                    if mission is not None
                    else format_navigation_lines(navigation, recovery, collision, avoidance)
                ),
            )
        elif avoidance is None:
            self._text_panel(
                canvas,
                (half, image_bottom + row_height, width, height),
                "DEPTH HEALTH / SPATIAL SUMMARY",
                format_depth_lines(state, depth_enabled),
            )
        else:
            lower_third = width // 3
            self._text_panel(
                canvas,
                (0, image_bottom + row_height, lower_third, height),
                "VISUAL HEALTH",
                format_visual_lines(state),
            )
            self._text_panel(
                canvas,
                (lower_third, image_bottom + row_height, lower_third * 2, height),
                "DEPTH HEALTH / SPATIAL SUMMARY",
                format_depth_lines(state, depth_enabled),
            )
            self._text_panel(
                canvas,
                (lower_third * 2, image_bottom + row_height, width, height),
                "REACTIVE AVOIDANCE",
                format_avoidance_lines(avoidance),
            )

        if paused:
            self._overlay_banner(canvas, "DISPLAY PAUSED - PERCEPTION AND SAFETY STILL RUNNING", ORANGE)
        if help_visible:
            self._help_overlay(canvas)
        hard_override_banner = avoidance_hard_override_banner(avoidance)
        if hard_override_banner is not None:
            self._overlay_banner(canvas, hard_override_banner, RED)
        if collision is not None and collision.collision_detected:
            self._overlay_banner(
                canvas,
                f"COLLISION DETECTED - {collision.latest_contacted_entity}",
                RED,
            )
        return canvas

    def _header(
        self,
        canvas: np.ndarray,
        report: ConfidenceReport,
        display_fps: float,
        height: int,
        paused: bool,
    ) -> None:
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], height), PANEL_ALT, -1)
        cv2.line(canvas, (0, height - 1), (canvas.shape[1], height - 1), BORDER, 1)
        title = "PROJECT POLARIS"
        _text(canvas, title, (18, 34), 0.85, TEXT, 2)
        title_width = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)[0][0]
        _text(canvas, "OPERATOR VIEW", (34 + title_width, 34), 0.62, CYAN, 2)
        status = report.mode.name
        color = mode_color(report.mode)
        badge_left = canvas.shape[1] - 410
        cv2.rectangle(canvas, (badge_left, 9), (badge_left + 210, height - 9), color, -1)
        _text(canvas, f"SAFETY  {status}", (badge_left + 14, 35), 0.58, TEXT, 2)
        _text(
            canvas,
            f"VIEW {display_fps:4.1f} FPS  {'PAUSED' if paused else 'LIVE'}",
            (badge_left + 228, 34),
            0.48,
            MUTED,
            1,
        )

    def _rgb_image(
        self,
        frame: CameraFrame | None,
        state: NavigationState,
        width: int,
        height: int,
        grid_overlay: bool,
    ) -> np.ndarray:
        content_height = height - 32
        if frame is None:
            return _placeholder_image(width, content_height, "WAITING FOR RGB CAMERA", "Auto-discovering X500 topic")
        image = _letterbox(frame.image_bgr, width, content_height)
        image_height, image_width = image.shape[:2]
        centre = (image_width // 2, image_height // 2)
        cv2.line(image, (centre[0] - 14, centre[1]), (centre[0] + 14, centre[1]), TEXT, 1)
        cv2.line(image, (centre[0], centre[1] - 14), (centre[0], centre[1] + 14), TEXT, 1)
        if grid_overlay:
            for fraction in (1 / 3, 2 / 3):
                x = int(image_width * fraction)
                y = int(image_height * fraction)
                cv2.line(image, (x, 0), (x, image_height), (210, 210, 210), 1)
                cv2.line(image, (0, y), (image_width, y), (210, 210, 210), 1)
        _overlay_text(
            image,
            f"RGB {state.visual.fps:.1f} FPS  age {_age(state.visual.frame_age_ms)}",
            (12, 25),
        )
        if state.visual.stale or not state.visual.camera_connected:
            _warning_overlay(image, "RGB STALE / DISCONNECTED")
        return image

    def _depth_image(
        self,
        proximity: np.ndarray | None,
        state: NavigationState,
        width: int,
        height: int,
        depth_enabled: bool,
    ) -> np.ndarray:
        content_height = height - 32
        if not depth_enabled:
            return _placeholder_image(width, content_height, "DEPTH DISABLED", "RGB and Ego State remain live")
        depth = state.depth
        if proximity is None or not depth.available or depth.stale:
            label = "DEPTH STALE - MAP HIDDEN" if depth.stale and depth.inference_timestamp_monotonic else "WAITING FOR DEPTH"
            return _placeholder_image(width, content_height, label, _first_reason(depth.reasons), danger=depth.stale)
        risk = cv2.resize(proximity, (width, content_height), interpolation=cv2.INTER_CUBIC)
        color = cv2.applyColorMap(np.uint8(np.clip(risk, 0, 1) * 255), cv2.COLORMAP_TURBO)
        _overlay_text(color, "BLUE = FAR / CLEAR    RED = NEAR / CONCERN", (12, 25))
        _overlay_text(
            color,
            f"{depth.inference_latency_ms:.0f} ms  {depth.inference_fps:.2f} FPS  stability {depth.temporal_stability:.2f}",
            (12, 51),
        )
        for fraction in (1 / 3, 2 / 3):
            x = int(width * fraction)
            y = int(content_height * fraction)
            cv2.line(color, (x, 0), (x, content_height), TEXT, 1)
            cv2.line(color, (0, y), (width, y), TEXT, 1)
        return color

    def _image_panel(
        self,
        canvas: np.ndarray,
        bounds: tuple[int, int, int, int],
        title: str,
        image: np.ndarray,
    ) -> None:
        x0, y0, x1, y1 = bounds
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), PANEL, -1)
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), BORDER, 1)
        _title(canvas, title, x0 + 14, y0 + 23)
        target_width = x1 - x0 - 2
        target_height = y1 - y0 - 33
        fitted = _letterbox(image, target_width, target_height)
        canvas[y0 + 32 : y1 - 1, x0 + 1 : x1 - 1] = fitted

    def _text_panel(
        self,
        canvas: np.ndarray,
        bounds: tuple[int, int, int, int],
        title: str,
        lines: list[str],
    ) -> None:
        x0, y0, x1, y1 = bounds
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), PANEL, -1)
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), BORDER, 1)
        _title(canvas, title, x0 + 14, y0 + 25)
        available = max(1, y1 - y0 - 43)
        spacing = max(12, min(25, available // max(1, len(lines))))
        scale = max(0.36, min(0.50, (x1 - x0) / 1000.0))
        for index, line in enumerate(lines):
            _text(canvas, line, (x0 + 14, y0 + 48 + index * spacing), scale, TEXT, 1)

    def _safety_panel(
        self,
        canvas: np.ndarray,
        bounds: tuple[int, int, int, int],
        report: ConfidenceReport,
        depth_enabled: bool,
    ) -> None:
        x0, y0, x1, y1 = bounds
        color = mode_color(report.mode)
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), PANEL, -1)
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y0 + 36), color, -1)
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), BORDER, 1)
        _text(canvas, f"SAFETY / CONFIDENCE   {report.mode.name}", (x0 + 14, y0 + 25), 0.50, TEXT, 2)
        panel_height = y1 - y0
        lines = format_safety_lines(report, depth_enabled)
        compact = panel_height < 175
        line_start = 50 if compact else 55
        line_spacing = 13 if compact else 18
        line_scale = 0.28 if compact else 0.36
        for index, line in enumerate(lines):
            _text(canvas, line, (x0 + 14, y0 + line_start + index * line_spacing), line_scale, TEXT, 1)
        reasons = report.critical_overrides or report.reasons
        reason_text = "None" if not reasons else "; ".join(reasons)
        reason_top = line_start + len(lines) * line_spacing + (3 if compact else 8)
        remaining = panel_height - reason_top - 8
        max_reason_lines = max(1, min(3, (remaining - 18) // 15))
        wrapped = textwrap.wrap(reason_text, width=max(28, (x1 - x0) // 8))[:max_reason_lines]
        _text(canvas, "HARD OVERRIDES / REASONS", (x0 + 14, y0 + reason_top), 0.27 if compact else 0.34, color, 1)
        for index, line in enumerate(wrapped):
            _text(
                canvas,
                line,
                (x0 + 14, y0 + reason_top + (13 if compact else 17) + index * (12 if compact else 15)),
                0.25 if compact else 0.31,
                TEXT,
                1,
            )

    def _overlay_banner(self, canvas: np.ndarray, text: str, color: tuple[int, int, int]) -> None:
        y0 = canvas.shape[0] // 2 - 30
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, y0), (canvas.shape[1], y0 + 60), color, -1)
        cv2.addWeighted(overlay, 0.85, canvas, 0.15, 0, canvas)
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]
        _text(canvas, text, ((canvas.shape[1] - size[0]) // 2, y0 + 39), 0.75, TEXT, 2)

    def _help_overlay(self, canvas: np.ndarray) -> None:
        width = min(720, canvas.shape[1] - 80)
        height = 250
        x0 = (canvas.shape[1] - width) // 2
        y0 = (canvas.shape[0] - height) // 2
        cv2.rectangle(canvas, (x0, y0), (x0 + width, y0 + height), BACKGROUND, -1)
        cv2.rectangle(canvas, (x0, y0), (x0 + width, y0 + height), CYAN, 2)
        _text(canvas, "OPERATOR VIEW KEYBOARD CONTROLS", (x0 + 24, y0 + 38), 0.66, TEXT, 2)
        controls = (
            "q / Escape    Quit cleanly",
            "p             Pause display rendering only",
            "s             Save one timestamped screenshot",
            "h             Toggle this help overlay",
            "g             Toggle 3x3 lines over RGB",
        )
        for index, line in enumerate(controls):
            _text(canvas, line, (x0 + 28, y0 + 78 + index * 30), 0.52, TEXT, 1)


def mode_color(mode: SafetyMode) -> tuple[int, int, int]:
    return {
        SafetyMode.NORMAL: GREEN,
        SafetyMode.CAUTIOUS: YELLOW,
        SafetyMode.DEGRADED: ORANGE,
        SafetyMode.EMERGENCY: RED,
    }[mode]


def _letterbox(image: np.ndarray, width: int, height: int) -> np.ndarray:
    output = np.full((height, width, 3), (8, 12, 20), dtype=np.uint8)
    if image.size == 0:
        return output
    scale = min(width / image.shape[1], height / image.shape[0])
    resized_width = max(1, int(image.shape[1] * scale))
    resized_height = max(1, int(image.shape[0] * scale))
    interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=interpolation)
    x = (width - resized_width) // 2
    y = (height - resized_height) // 2
    output[y : y + resized_height, x : x + resized_width] = resized
    return output


def _placeholder_image(
    width: int,
    height: int,
    title: str,
    detail: str,
    danger: bool = False,
) -> np.ndarray:
    image = np.full((height, width, 3), PANEL_ALT, dtype=np.uint8)
    color = RED if danger else MUTED
    title_size = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.72, 2)[0]
    _text(image, title, ((width - title_size[0]) // 2, height // 2 - 8), 0.72, color, 2)
    detail = textwrap.shorten(detail, width=80, placeholder="...")
    detail_size = cv2.getTextSize(detail, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0]
    _text(image, detail, ((width - detail_size[0]) // 2, height // 2 + 26), 0.42, TEXT, 1)
    return image


def _placeholder(panel: np.ndarray, title: str, detail: str) -> None:
    size = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2)[0]
    _text(panel, title, ((panel.shape[1] - size[0]) // 2, panel.shape[0] // 2), 0.58, RED, 2)
    detail = textwrap.shorten(detail, width=52, placeholder="...")
    detail_size = cv2.getTextSize(detail, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)[0]
    _text(panel, detail, ((panel.shape[1] - detail_size[0]) // 2, panel.shape[0] // 2 + 26), 0.36, TEXT, 1)


def _warning_overlay(image: np.ndarray, text: str) -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (image.shape[1], image.shape[0]), RED, -1)
    cv2.addWeighted(overlay, 0.35, image, 0.65, 0, image)
    size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
    _text(image, text, ((image.shape[1] - size[0]) // 2, image.shape[0] // 2), 0.8, TEXT, 2)


def _overlay_text(image: np.ndarray, text: str, origin: tuple[int, int]) -> None:
    _text(image, text, origin, 0.48, (0, 0, 0), 3)
    _text(image, text, origin, 0.48, TEXT, 1)


def _title(image: np.ndarray, text: str, x: int, y: int) -> None:
    _text(image, text, (x, y), 0.46, CYAN, 1)


def _text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    scale: float,
    color: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _age(value: float) -> str:
    return "never" if not np.isfinite(value) else f"{value:.0f} ms"


def _first_reason(reasons: tuple[str, ...]) -> str:
    return reasons[0] if reasons else "No usable depth inference"
