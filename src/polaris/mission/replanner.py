from __future__ import annotations

import math
from dataclasses import replace

from polaris.control.models import RiskSummary
from polaris.navigation.models import LocalGoal

from .models import (
    FailureAction,
    MissionOrchestratorConfig,
    MissionWaypoint,
    ReplanDecision,
)


class BoundedMissionReplanner:
    def __init__(self, config: MissionOrchestratorConfig) -> None:
        self.config = config

    def generate_lateral_staging(
        self,
        failed: MissionWaypoint,
        *,
        current_north_offset_m: float,
        current_east_offset_m: float,
        current_yaw_deg: float,
        risk: RiskSummary,
        replan_index: int,
        existing_ids: set[str],
    ) -> ReplanDecision:
        if replan_index >= self.config.maximum_replans:
            return ReplanDecision(False, "Replan limit exhausted", None, "NONE")
        if not risk.valid:
            return ReplanDecision(False, "Risk grid is invalid", None, "NONE")
        direction = -1.0 if risk.left < risk.right else 1.0
        yaw = math.radians(current_yaw_deg)
        right_north = -math.sin(yaw)
        right_east = math.cos(yaw)
        offset = self.config.lateral_replan_offset_m * direction
        north = current_north_offset_m + right_north * offset
        east = current_east_offset_m + right_east * offset
        down = failed.goal.target_down_m
        if not self.config.world_bounds.contains(north, east, down):
            return ReplanDecision(False, "Generated staging point is outside world bounds", None, "LATERAL")
        route_offset = self._distance_to_segment(
            north,
            east,
            current_north_offset_m,
            current_east_offset_m,
            failed.goal.target_north_m,
            failed.goal.target_east_m,
        )
        if route_offset > self.config.maximum_generated_waypoint_offset_m:
            return ReplanDecision(False, "Generated staging point exceeds route-offset bound", None, "LATERAL")
        identifier = f"generated-{failed.waypoint_id}-r{replan_index + 1}"
        if identifier in existing_ids:
            return ReplanDecision(False, "Generated waypoint ID is not unique", None, "LATERAL")
        goal = LocalGoal(
            goal_id=identifier,
            target_north_m=north,
            target_east_m=east,
            target_down_m=down,
            horizontal_tolerance_m=failed.goal.horizontal_tolerance_m,
            vertical_tolerance_m=failed.goal.vertical_tolerance_m,
            timeout_s=self.config.generated_waypoint_timeout_s,
            maximum_path_length_m=min(
                failed.goal.maximum_path_length_m,
                self.config.maximum_generated_waypoint_offset_m * 2.0,
            ),
            maximum_displacement_m=self.config.maximum_generated_waypoint_offset_m * 2.0,
            completion_hold_s=failed.goal.completion_hold_s,
            source="generated:lateral-staging:MISSION_ORIGIN_NED_OFFSET",
        )
        waypoint = MissionWaypoint(
            waypoint_id=identifier,
            goal=goal,
            required=True,
            skippable=False,
            maximum_attempts=1,
            verification_hold_s=failed.verification_hold_s,
            minimum_visual_health=failed.minimum_visual_health,
            minimum_depth_health=failed.minimum_depth_health,
            failure_action=FailureAction.ABORT_MISSION,
            metadata={
                "generated_for": failed.waypoint_id,
                "strategy": "LATERAL_STAGING",
                "side": "LEFT" if direction < 0 else "RIGHT",
            },
            generated=True,
        )
        waypoint.validate(self.config.world_bounds)
        return ReplanDecision(True, "Bounded lateral staging generated", waypoint, "LATERAL_STAGING")

    def generate_previous_waypoint_retreat(
        self,
        previous: MissionWaypoint | None,
        failed: MissionWaypoint,
        *,
        replan_index: int,
        existing_ids: set[str],
    ) -> ReplanDecision:
        if previous is None:
            return ReplanDecision(False, "No completed waypoint is available", None, "RETREAT")
        identifier = f"generated-retreat-{failed.waypoint_id}-r{replan_index + 1}"
        if identifier in existing_ids:
            return ReplanDecision(False, "Generated waypoint ID is not unique", None, "RETREAT")
        waypoint = replace(
            previous,
            waypoint_id=identifier,
            goal=replace(
                previous.goal,
                goal_id=identifier,
                timeout_s=self.config.generated_waypoint_timeout_s,
                source="generated:previous-waypoint-retreat:MISSION_ORIGIN_NED_OFFSET",
            ),
            maximum_attempts=1,
            failure_action=FailureAction.ABORT_MISSION,
            metadata={"generated_for": failed.waypoint_id, "strategy": "RETREAT_PREVIOUS"},
            generated=True,
        )
        waypoint.validate(self.config.world_bounds)
        return ReplanDecision(True, "Previous waypoint retreat generated", waypoint, "RETREAT_PREVIOUS")

    def generate_healthier_viewpoint_staging(
        self,
        failed: MissionWaypoint,
        *,
        current_north_offset_m: float,
        current_east_offset_m: float,
        selected_heading_deg: float | None,
        replan_index: int,
        existing_ids: set[str],
    ) -> ReplanDecision:
        if selected_heading_deg is None:
            return ReplanDecision(False, "No selected recovery heading exists", None, "HEALTHIER_VIEW")
        if replan_index >= self.config.maximum_replans:
            return ReplanDecision(False, "Replan limit exhausted", None, "HEALTHIER_VIEW")
        distance = min(1.0, self.config.maximum_generated_waypoint_offset_m)
        heading = math.radians(selected_heading_deg)
        north = current_north_offset_m + math.cos(heading) * distance
        east = current_east_offset_m + math.sin(heading) * distance
        down = failed.goal.target_down_m
        if not self.config.world_bounds.contains(north, east, down):
            return ReplanDecision(False, "Healthier-viewpoint staging is outside bounds", None, "HEALTHIER_VIEW")
        identifier = f"generated-view-{failed.waypoint_id}-r{replan_index + 1}"
        if identifier in existing_ids:
            return ReplanDecision(False, "Generated waypoint ID is not unique", None, "HEALTHIER_VIEW")
        goal = LocalGoal(
            identifier,
            north,
            east,
            down,
            timeout_s=self.config.generated_waypoint_timeout_s,
            maximum_path_length_m=distance * 2.0,
            maximum_displacement_m=distance * 2.0,
            source="generated:healthier-viewpoint:MISSION_ORIGIN_NED_OFFSET",
        )
        waypoint = MissionWaypoint(
            identifier,
            goal,
            required=True,
            skippable=False,
            maximum_attempts=1,
            failure_action=FailureAction.ABORT_MISSION,
            metadata={"generated_for": failed.waypoint_id, "strategy": "HEALTHIER_VIEWPOINT"},
            generated=True,
        )
        waypoint.validate(self.config.world_bounds)
        return ReplanDecision(True, "Healthier-viewpoint staging generated", waypoint, "HEALTHIER_VIEWPOINT")

    @staticmethod
    def _distance_to_segment(
        point_n: float,
        point_e: float,
        start_n: float,
        start_e: float,
        end_n: float,
        end_e: float,
    ) -> float:
        line_n = end_n - start_n
        line_e = end_e - start_e
        length_squared = line_n * line_n + line_e * line_e
        if length_squared == 0:
            return math.hypot(point_n - start_n, point_e - start_e)
        projection = (
            (point_n - start_n) * line_n + (point_e - start_e) * line_e
        ) / length_squared
        projection = max(0.0, min(1.0, projection))
        nearest_n = start_n + projection * line_n
        nearest_e = start_e + projection * line_e
        return math.hypot(point_n - nearest_n, point_e - nearest_e)
