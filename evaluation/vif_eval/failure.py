from __future__ import annotations

import csv
import statistics
from collections import defaultdict
from pathlib import Path


def _rows(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _safe_stats(values: list[float], prefix: str) -> dict[str, float | None]:
    if not values:
        return {f"{prefix}_mean_sec": None, f"{prefix}_median_sec": None}
    return {
        f"{prefix}_mean_sec": float(statistics.mean(values)),
        f"{prefix}_median_sec": float(statistics.median(values)),
    }


def evaluate_failures(
    events_path: str | Path,
    labels_path: str | Path,
    detection_grace_sec: float = 1.0,
) -> dict[str, float | int | None]:
    """Evaluate REQUESTED detections against labelled [start_sec, end_sec] windows."""
    events = _rows(events_path)
    labels = _rows(labels_path)
    requested = sorted(
        float(row["timestamp_sec"]) for row in events if row["event"].upper() == "REQUESTED"
    )
    windows = sorted(
        (float(row["start_sec"]), float(row["end_sec"])) for row in labels
    )
    used_detection: set[int] = set()
    delays: list[float] = []
    true_positives = 0
    for start, end in windows:
        for index, timestamp in enumerate(requested):
            if index in used_detection:
                continue
            if start <= timestamp <= end + detection_grace_sec:
                used_detection.add(index)
                true_positives += 1
                delays.append(max(0.0, timestamp - start))
                break
    false_positives = len(requested) - true_positives
    false_negatives = len(windows) - true_positives
    precision = true_positives / len(requested) if requested else 0.0
    recall = true_positives / len(windows) if windows else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0

    attempts: dict[str, dict[str, float]] = defaultdict(dict)
    for row in events:
        attempt = row.get("attempt_id", "")
        event = row["event"].upper()
        if attempt and event in {"REQUESTED", "SUCCEEDED", "FAILED"}:
            attempts[attempt][event] = float(row["timestamp_sec"])
    recovery_times = [
        attempt["SUCCEEDED"] - attempt["REQUESTED"]
        for attempt in attempts.values()
        if "REQUESTED" in attempt and "SUCCEEDED" in attempt
    ]
    attempted_recoveries = sum("REQUESTED" in attempt for attempt in attempts.values())
    recovery_successes = len(recovery_times)

    all_times = requested + [value for window in windows for value in window]
    duration_sec = max(all_times) - min(all_times) if len(all_times) >= 2 else 0.0
    result: dict[str, float | int | None] = {
        "labelled_failures": len(windows),
        "detections": len(requested),
        "true_positives": true_positives,
        "false_positives": false_positives,
        "false_negatives": false_negatives,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "false_alarms_per_minute": false_positives / (duration_sec / 60.0)
        if duration_sec > 0
        else 0.0,
        "recovery_attempts": attempted_recoveries,
        "recovery_successes": recovery_successes,
        "relocalization_success_rate": recovery_successes / attempted_recoveries
        if attempted_recoveries
        else 0.0,
    }
    result.update(_safe_stats(delays, "detection_delay"))
    result.update(_safe_stats(recovery_times, "recovery_time"))
    return result
