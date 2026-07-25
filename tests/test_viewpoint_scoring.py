from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

from polaris.navigation import (  # noqa: E402
    ActivePerceptionConfig,
    ViewpointScorer,
)
from test_reactive_avoidance import CLEAR, navigation  # noqa: E402


class ViewpointScoringTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ActivePerceptionConfig()
        self.scorer = ViewpointScorer(self.config)

    def test_goal_alignment_influences_selection(self) -> None:
        state = navigation(CLEAR, 1)
        aligned = self.scorer.score(state, 30.0, 30.0, 0.0)
        away = self.scorer.score(state, -30.0, 30.0, 0.0)
        self.assertGreater(aligned.score, away.score)

    def test_glare_heavy_heading_is_penalized(self) -> None:
        healthy = navigation(CLEAR, 1)
        glare = navigation(CLEAR, 1)
        glare.visual = replace(
            glare.visual,
            visual_health_score=0.55,
            overexposed_ratio=0.8,
        )
        healthy_score = self.scorer.score(healthy, 0.0, 0.0, 0.0)
        glare_score = self.scorer.score(glare, 0.0, 0.0, 0.0)
        self.assertGreater(healthy_score.score, glare_score.score)

    def test_clear_but_visually_unreliable_heading_is_rejected(self) -> None:
        state = navigation(CLEAR, 1)
        state.visual = replace(state.visual, visual_health_score=0.2)
        score = self.scorer.score(state, 0.0, 0.0, 0.0)
        self.assertFalse(score.acceptable)
        self.assertIn("Visual health below viewpoint minimum", score.reasons)

    def test_recent_failure_history_penalizes_heading(self) -> None:
        state = navigation(CLEAR, 1)
        normal = self.scorer.score(state, 15.0, 15.0, 0.0)
        failed = self.scorer.score(state, 15.0, 15.0, 0.0, (15.0,))
        self.assertGreater(normal.score, failed.score)


if __name__ == "__main__":
    unittest.main()
