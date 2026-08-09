"""Pure benchmark validation and report generation.

This module deliberately has no ROS dependency.  Keeping acceptance logic pure makes
the benchmark criteria reviewable and testable in ordinary CI.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Iterable


SCENARIOS = ("clean", "visual_occlusion", "camera_blackout")

EXPECTED_PATHS = {
    "visual_occlusion": ["HEALTHY", "RELOCALIZING", "RECOVERED", "HEALTHY"],
    "camera_blackout": [
        "HEALTHY",
        "LOST",
        "RESETTING",
        "INITIALIZING",
        "HEALTHY",
    ],
}

FORBIDDEN_STATES = {
    "clean": {"LOST", "RELOCALIZING", "RESETTING"},
    "visual_occlusion": {"RESETTING"},
    "camera_blackout": {"RECOVERED"},
}

FAILURE_STATES = {"LOST", "RELOCALIZING", "RESETTING"}


def contains_ordered_path(states: Iterable[str], expected: list[str]) -> bool:
    """Return whether *expected* occurs in order, allowing intermediate states."""
    next_index = 0
    for state in states:
        if state == expected[next_index]:
            next_index += 1
            if next_index == len(expected):
                return True
    return False


def _first_transition_time(
    transitions: list[dict[str, Any]],
    states: set[str],
    *,
    after: float | None = None,
) -> float | None:
    for transition in transitions:
        timestamp = float(transition["monotonic_sec"])
        if transition["state"] in states and (after is None or timestamp >= after):
            return timestamp
    return None


def _fault_time(
    fault_events: list[dict[str, Any]], event_name: str
) -> float | None:
    for event in fault_events:
        if event["event"] == event_name:
            return float(event["monotonic_sec"])
    return None


def _rounded_delta(end: float | None, start: float | None) -> float | None:
    if end is None or start is None:
        return None
    return round(end - start, 6)


def _stable_healthy_duration(
    transitions: list[dict[str, Any]], observation_ended_sec: float
) -> float:
    healthy_start: float | None = None
    best = 0.0
    for transition in transitions:
        timestamp = float(transition["monotonic_sec"])
        if transition["state"] == "HEALTHY":
            healthy_start = timestamp
        elif healthy_start is not None:
            best = max(best, timestamp - healthy_start)
            healthy_start = None
    if healthy_start is not None:
        best = max(best, observation_ended_sec - healthy_start)
    return round(best, 6)


def evaluate_scenario(
    scenario: str,
    *,
    state_transitions: list[dict[str, Any]],
    map_ids: list[int],
    fault_events: list[dict[str, Any]],
    relocalization_events: list[dict[str, Any]],
    observation_started_sec: float,
    observation_ended_sec: float,
    clean_hold_sec: float,
    process_errors: list[str] | None = None,
) -> dict[str, Any]:
    """Evaluate one scenario against explicit state, map, and timing invariants."""
    if scenario not in SCENARIOS:
        raise ValueError(f"unsupported scenario: {scenario}")

    process_errors = list(process_errors or [])
    states = [item["state"] for item in state_transitions]
    errors = list(process_errors)
    forbidden = sorted(FORBIDDEN_STATES[scenario].intersection(states))
    if forbidden:
        errors.append(f"forbidden states observed: {', '.join(forbidden)}")

    invalid_failure_states = sorted(
        {
            item["state"]
            for item in state_transitions
            if item["state"] in FAILURE_STATES and bool(item["pose_valid"])
        }
    )
    if invalid_failure_states:
        errors.append(
            "pose remained valid during failure states: "
            + ", ".join(invalid_failure_states)
        )

    fault_start = _fault_time(fault_events, "STARTED")
    fault_end = _fault_time(fault_events, "ENDED")
    detection_time: float | None = None
    recovered_time: float | None = None
    healthy_after_fault_time: float | None = None
    reset_time: float | None = None
    stable_healthy_sec = _stable_healthy_duration(
        state_transitions, observation_ended_sec
    )

    if scenario == "clean":
        if fault_events:
            errors.append("clean scenario emitted a fault event")
        if "HEALTHY" not in states:
            errors.append("HEALTHY was never observed")
        if stable_healthy_sec < clean_hold_sec:
            errors.append(
                f"continuous HEALTHY period was {stable_healthy_sec:.3f}s; "
                f"required {clean_hold_sec:.3f}s"
            )
        if len(map_ids) != 1:
            errors.append("clean tracking requires exactly one observed map ID")
    else:
        if fault_start is None:
            errors.append("fault STARTED event was not observed")
        if fault_end is None:
            errors.append("fault ENDED event was not observed")
        if fault_start is not None and fault_end is not None and fault_end <= fault_start:
            errors.append("fault ENDED event did not follow STARTED")

        expected = EXPECTED_PATHS[scenario]
        if not contains_ordered_path(states, expected):
            errors.append("missing ordered state path: " + " -> ".join(expected))

        if scenario == "visual_occlusion":
            if len(map_ids) != 1:
                errors.append("same-map recovery requires exactly one observed map ID")
            detection_time = _first_transition_time(
                state_transitions, {"LOST", "RELOCALIZING"}, after=fault_start
            )
            recovered_time = _first_transition_time(
                state_transitions, {"RECOVERED"}, after=fault_end
            )
            healthy_after_fault_time = _first_transition_time(
                state_transitions, {"HEALTHY"}, after=fault_end
            )
        else:
            if len(map_ids) < 2:
                errors.append("blackout reset requires an observed map-ID change")
            detection_time = _first_transition_time(
                state_transitions,
                {"LOST", "RELOCALIZING", "RESETTING"},
                after=fault_start,
            )
            reset_time = _first_transition_time(
                state_transitions, {"RESETTING"}, after=fault_start
            )
            healthy_after_fault_time = _first_transition_time(
                state_transitions, {"HEALTHY"}, after=fault_end
            )

    metrics = {
        "observation_duration_sec": round(
            observation_ended_sec - observation_started_sec, 6
        ),
        "stable_healthy_sec": stable_healthy_sec,
        "fault_duration_sec": _rounded_delta(fault_end, fault_start),
        "detection_delay_sec": _rounded_delta(detection_time, fault_start),
        "recovery_after_fault_end_sec": _rounded_delta(recovered_time, fault_end),
        "healthy_after_fault_end_sec": _rounded_delta(
            healthy_after_fault_time, fault_end
        ),
        "reset_delay_sec": _rounded_delta(reset_time, fault_start),
        "map_count": len(map_ids),
        "map_changed": len(map_ids) > 1,
    }

    return {
        "scenario": scenario,
        "status": "PASS" if not errors else "FAIL",
        "passed": not errors,
        "errors": errors,
        "state_sequence": states,
        "map_ids": map_ids,
        "metrics": metrics,
        "state_transitions": state_transitions,
        "fault_events": fault_events,
        "relocalization_events": relocalization_events,
    }


def _format_metric(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def render_markdown_report(summary: dict[str, Any]) -> str:
    """Render a compact report suitable for a README, CI artifact, or review."""
    rows = []
    for result in summary["scenarios"]:
        metrics = result["metrics"]
        rows.append(
            "| {scenario} | {status} | {detect} | {recover} | {healthy} | {maps} |".format(
                scenario=result["scenario"],
                status=result["status"],
                detect=_format_metric(metrics["detection_delay_sec"]),
                recover=_format_metric(metrics["recovery_after_fault_end_sec"]),
                healthy=_format_metric(metrics["healthy_after_fault_end_sec"]),
                maps=", ".join(str(item) for item in result["map_ids"]) or "—",
            )
        )

    detail_sections = []
    for result in summary["scenarios"]:
        errors = "None" if not result["errors"] else "; ".join(result["errors"])
        detail_sections.append(
            "\n".join(
                [
                    f"### {result['scenario']} — {result['status']}",
                    "",
                    f"- State sequence: `{' → '.join(result['state_sequence'])}`",
                    f"- Map IDs: `{result['map_ids']}`",
                    f"- Acceptance errors: {errors}",
                ]
            )
        )

    return "\n".join(
        [
            "# VIF ORB-SLAM3 benchmark report",
            "",
            f"- Run ID: `{summary['run_id']}`",
            f"- Generated (UTC): `{summary['generated_at_utc']}`",
            f"- Overall result: **{'PASS' if summary['all_passed'] else 'FAIL'}**",
            f"- Dataset: `{summary['configuration']['dataset_path']}`",
            "",
            "| Scenario | Result | Detection delay (s) | Same-map recovery "
            "after fault (s) | Healthy after fault (s) | Map IDs |",
            "|---|---:|---:|---:|---:|---|",
            *rows,
            "",
            "Detection delay is measured from the published fault-start event to "
            "the first failure-state transition. Same-map recovery is reported "
            "only for visual occlusion; a camera blackout is expected to reset "
            "and reinitialize instead.",
            "",
            *detail_sections,
            "",
        ]
    )


def write_summary_artifacts(summary: dict[str, Any], output_dir: Path) -> None:
    """Write JSON, CSV, and Markdown summaries for one benchmark run."""
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    columns = [
        "scenario",
        "status",
        "detection_delay_sec",
        "fault_duration_sec",
        "recovery_after_fault_end_sec",
        "healthy_after_fault_end_sec",
        "reset_delay_sec",
        "map_count",
        "map_changed",
        "map_ids",
        "state_sequence",
        "errors",
    ]
    with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for result in summary["scenarios"]:
            metrics = result["metrics"]
            writer.writerow(
                {
                    "scenario": result["scenario"],
                    "status": result["status"],
                    "detection_delay_sec": metrics["detection_delay_sec"],
                    "fault_duration_sec": metrics["fault_duration_sec"],
                    "recovery_after_fault_end_sec": metrics[
                        "recovery_after_fault_end_sec"
                    ],
                    "healthy_after_fault_end_sec": metrics[
                        "healthy_after_fault_end_sec"
                    ],
                    "reset_delay_sec": metrics["reset_delay_sec"],
                    "map_count": metrics["map_count"],
                    "map_changed": metrics["map_changed"],
                    "map_ids": " ".join(str(item) for item in result["map_ids"]),
                    "state_sequence": " -> ".join(result["state_sequence"]),
                    "errors": "; ".join(result["errors"]),
                }
            )

    (output_dir / "report.md").write_text(
        render_markdown_report(summary), encoding="utf-8"
    )
