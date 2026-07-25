from __future__ import annotations

from polaris.control.models import RiskSummary
from polaris.ego_state.models import NavigationState
from polaris.evaluation.viewpoint import shortest_yaw_delta

from .models import ActivePerceptionConfig, ViewpointScore


class ViewpointScorer:
    def __init__(self, config: ActivePerceptionConfig) -> None:
        self.config = config

    def score(
        self,
        navigation: NavigationState,
        heading_deg: float,
        goal_heading_deg: float,
        base_heading_deg: float,
        failed_headings: tuple[float, ...] = (),
    ) -> ViewpointScore:
        risk = RiskSummary.from_grid(navigation.depth.risk_grid)
        visual = navigation.visual
        depth = navigation.depth
        feature_quality = min(
            1.0,
            0.5 * visual.feature_count / self.config.feature_count_reference
            + 0.5 * visual.feature_coverage,
        )
        exposure_quality = max(
            0.0,
            1.0 - max(visual.overexposed_ratio, visual.underexposed_ratio),
        )
        goal_alignment = max(
            0.0,
            1.0 - abs(shortest_yaw_delta(goal_heading_deg, heading_deg)) / 180.0,
        )
        yaw_cost = min(
            1.0,
            abs(shortest_yaw_delta(heading_deg, base_heading_deg)) / 180.0,
        )
        failure_count = sum(
            abs(shortest_yaw_delta(heading_deg, failed))
            <= self.config.failure_heading_radius_deg
            for failed in failed_headings
        )
        failure_penalty = min(1.0, failure_count * self.config.failure_history_penalty)
        forward_clearance = 0.0 if not risk.valid else 1.0 - risk.forward
        score = (
            self.config.visual_weight * visual.visual_health_score
            + self.config.depth_weight * depth.depth_health_score
            + self.config.stability_weight * depth.temporal_stability
            + self.config.clearance_weight * forward_clearance
            + self.config.feature_weight * feature_quality
            + self.config.exposure_weight * exposure_quality
            + self.config.goal_alignment_weight * goal_alignment
            + self.config.yaw_change_weight * (1.0 - yaw_cost)
            - failure_penalty
        )
        reasons: list[str] = []
        if visual.visual_health_score < self.config.minimum_visual_health:
            reasons.append("Visual health below viewpoint minimum")
        if depth.depth_health_score < self.config.minimum_depth_health:
            reasons.append("Depth health below viewpoint minimum")
        if depth.temporal_stability < self.config.minimum_temporal_stability:
            reasons.append("Depth stability below viewpoint minimum")
        if not risk.valid:
            reasons.append("Risk grid invalid")
        acceptable = not reasons and score >= self.config.minimum_acceptable_score
        return ViewpointScore(
            heading_deg=heading_deg,
            score=max(0.0, min(1.0, score)),
            acceptable=acceptable,
            visual_health=visual.visual_health_score,
            depth_health=depth.depth_health_score,
            temporal_stability=depth.temporal_stability,
            forward_clearance=forward_clearance,
            feature_quality=feature_quality,
            exposure_quality=exposure_quality,
            goal_alignment=goal_alignment,
            yaw_change_cost=yaw_cost,
            failure_penalty=failure_penalty,
            reasons=tuple(reasons),
        )
