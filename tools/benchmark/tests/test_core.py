from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


BENCHMARK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BENCHMARK_ROOT))

from vif_benchmark.core import evaluate_scenario, write_summary_artifacts  # noqa: E402


def transition(state, timestamp, pose_valid=False):
    return {
        "state": state,
        "monotonic_sec": timestamp,
        "score": 0.9,
        "pose_valid": pose_valid,
        "reasons": [],
    }


def fault(event, timestamp, fault_type="visual_occlusion"):
    return {
        "event": event,
        "monotonic_sec": timestamp,
        "fault_type": fault_type,
    }


class ScenarioEvaluationTests(unittest.TestCase):
    def test_clean_requires_stable_single_map_tracking(self):
        result = evaluate_scenario(
            "clean",
            state_transitions=[
                transition("INITIALIZING", 0.0),
                transition("HEALTHY", 2.0, True),
            ],
            map_ids=[0],
            fault_events=[],
            relocalization_events=[],
            observation_started_sec=0.0,
            observation_ended_sec=12.5,
            clean_hold_sec=10.0,
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["metrics"]["stable_healthy_sec"], 10.5)

    def test_verified_visual_occlusion_is_same_map_recovery(self):
        result = evaluate_scenario(
            "visual_occlusion",
            state_transitions=[
                transition("INITIALIZING", 0.0),
                transition("HEALTHY", 14.9, True),
                transition("RELOCALIZING", 59.059),
                transition("RECOVERED", 60.409, True),
                transition("HEALTHY", 60.908, True),
            ],
            map_ids=[0],
            fault_events=[fault("STARTED", 59.0), fault("ENDED", 59.989)],
            relocalization_events=[],
            observation_started_sec=0.0,
            observation_ended_sec=61.0,
            clean_hold_sec=10.0,
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["metrics"]["detection_delay_sec"], 0.059)
        self.assertEqual(
            result["metrics"]["recovery_after_fault_end_sec"], 0.42
        )
        self.assertFalse(result["metrics"]["map_changed"])

    def test_blackout_requires_reset_new_map_and_no_recovered_state(self):
        result = evaluate_scenario(
            "camera_blackout",
            state_transitions=[
                transition("INITIALIZING", 0.0),
                transition("HEALTHY", 3.0, True),
                transition("LOST", 45.55),
                transition("RESETTING", 46.10),
                transition("INITIALIZING", 46.20),
                transition("HEALTHY", 51.0, True),
            ],
            map_ids=[0, 1],
            fault_events=[
                fault("STARTED", 45.0, "camera_blackout"),
                fault("ENDED", 46.0, "camera_blackout"),
            ],
            relocalization_events=[],
            observation_started_sec=0.0,
            observation_ended_sec=51.5,
            clean_hold_sec=10.0,
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["metrics"]["map_count"], 2)
        self.assertEqual(result["metrics"]["reset_delay_sec"], 1.1)

    def test_false_recovery_after_map_change_fails(self):
        result = evaluate_scenario(
            "camera_blackout",
            state_transitions=[
                transition("HEALTHY", 0.0, True),
                transition("LOST", 1.1),
                transition("RECOVERED", 2.1, True),
            ],
            map_ids=[0, 1],
            fault_events=[
                fault("STARTED", 1.0, "camera_blackout"),
                fault("ENDED", 2.0, "camera_blackout"),
            ],
            relocalization_events=[],
            observation_started_sec=0.0,
            observation_ended_sec=3.0,
            clean_hold_sec=1.0,
        )
        self.assertFalse(result["passed"])
        self.assertTrue(any("forbidden" in item for item in result["errors"]))


class ReportTests(unittest.TestCase):
    def test_writes_machine_and_human_readable_artifacts(self):
        result = evaluate_scenario(
            "clean",
            state_transitions=[transition("HEALTHY", 0.0, True)],
            map_ids=[0],
            fault_events=[],
            relocalization_events=[],
            observation_started_sec=0.0,
            observation_ended_sec=2.0,
            clean_hold_sec=1.0,
        )
        summary = {
            "run_id": "test-run",
            "generated_at_utc": "2026-08-07T00:00:00+00:00",
            "all_passed": True,
            "configuration": {"dataset_path": "/data/MH_01_easy"},
            "scenarios": [result],
        }
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_summary_artifacts(summary, output)
            parsed = json.loads((output / "summary.json").read_text())
            self.assertEqual(parsed["run_id"], "test-run")
            with (output / "summary.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["status"], "PASS")
            report = (output / "report.md").read_text()
            self.assertIn("Overall result: **PASS**", report)


if __name__ == "__main__":
    unittest.main()
