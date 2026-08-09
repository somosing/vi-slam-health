#!/usr/bin/env python3
"""Run reproducible clean, same-map recovery, and blackout-reset benchmarks."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from vif_interfaces.msg import (
    FaultEvent,
    LocalizationHealth,
    RelocalizationEvent,
    TrackingStatus,
)

from vif_benchmark.core import (
    EXPECTED_PATHS,
    SCENARIOS,
    contains_ordered_path,
    evaluate_scenario,
    write_summary_artifacts,
)


STATE_NAMES = {
    LocalizationHealth.INITIALIZING: "INITIALIZING",
    LocalizationHealth.HEALTHY: "HEALTHY",
    LocalizationHealth.DEGRADED: "DEGRADED",
    LocalizationHealth.LOST: "LOST",
    LocalizationHealth.RELOCALIZING: "RELOCALIZING",
    LocalizationHealth.RECOVERED: "RECOVERED",
    LocalizationHealth.RESETTING: "RESETTING",
}

FAULT_EVENT_NAMES = {
    FaultEvent.STARTED: "STARTED",
    FaultEvent.ENDED: "ENDED",
}

RELOCALIZATION_EVENT_NAMES = {
    RelocalizationEvent.REQUESTED: "REQUESTED",
    RelocalizationEvent.STARTED: "STARTED",
    RelocalizationEvent.SUCCEEDED: "SUCCEEDED",
    RelocalizationEvent.FAILED: "FAILED",
    RelocalizationEvent.RESET_REQUESTED: "RESET_REQUESTED",
    RelocalizationEvent.NEW_MAP_REQUESTED: "NEW_MAP_REQUESTED",
}

VIF_RUNTIME_NODES = {
    "euroc_player",
    "fault_injector",
    "health_monitor",
    "orbslam3",
    "relocalization_manager",
    "sensor_guard",
}


class BenchmarkObserver(Node):
    """Collect benchmark evidence without feeding anything into the detector."""

    def __init__(self, scenario: str) -> None:
        super().__init__(f"vif_benchmark_observer_{os.getpid()}")
        self.scenario = scenario
        self.observation_started_sec = time.monotonic()
        self.state_transitions: list[dict[str, Any]] = []
        self.map_ids: list[int] = []
        self.fault_events: list[dict[str, Any]] = []
        self.relocalization_events: list[dict[str, Any]] = []
        self.create_subscription(
            LocalizationHealth, "/vif/localization/health", self._on_health, 10
        )
        self.create_subscription(
            TrackingStatus, "/vif/slam/tracking", self._on_tracking, 10
        )
        self.create_subscription(FaultEvent, "/vif/fault/events", self._on_fault, 10)
        self.create_subscription(
            RelocalizationEvent,
            "/vif/relocalization/events",
            self._on_relocalization,
            20,
        )

    @property
    def states(self) -> list[str]:
        return [item["state"] for item in self.state_transitions]

    @property
    def current_state(self) -> str | None:
        return self.states[-1] if self.state_transitions else None

    @property
    def last_transition_sec(self) -> float | None:
        if not self.state_transitions:
            return None
        return float(self.state_transitions[-1]["monotonic_sec"])

    def runtime_node_names(self) -> set[str]:
        return {
            name.lstrip("/")
            for name, _namespace in self.get_node_names_and_namespaces()
            if name.lstrip("/") in VIF_RUNTIME_NODES
        }

    def _on_health(self, message: LocalizationHealth) -> None:
        state = STATE_NAMES.get(message.state, f"UNKNOWN({message.state})")
        if self.state_transitions and self.state_transitions[-1]["state"] == state:
            return
        transition = {
            "state": state,
            "monotonic_sec": time.monotonic(),
            "ros_stamp_sec": message.header.stamp.sec
            + message.header.stamp.nanosec / 1e9,
            "score": round(float(message.score), 6),
            "pose_valid": bool(message.pose_valid),
            "reasons": list(message.reasons),
        }
        self.state_transitions.append(transition)
        self.get_logger().info(
            f"[{self.scenario}] state={state} score={message.score:.3f} "
            f"pose_valid={message.pose_valid}"
        )

    def _on_tracking(self, message: TrackingStatus) -> None:
        map_id = int(message.map_id)
        if map_id not in self.map_ids:
            self.map_ids.append(map_id)
            self.get_logger().info(f"[{self.scenario}] observed map_id={map_id}")

    def _on_fault(self, message: FaultEvent) -> None:
        event_name = FAULT_EVENT_NAMES.get(message.event, f"UNKNOWN({message.event})")
        event = {
            "event": event_name,
            "fault_type": message.fault_type,
            "monotonic_sec": time.monotonic(),
            "ros_stamp_sec": message.header.stamp.sec
            + message.header.stamp.nanosec / 1e9,
            "injector_elapsed_sec": round(float(message.elapsed_sec), 6),
        }
        self.fault_events.append(event)
        self.get_logger().info(
            f"[{self.scenario}] fault={message.fault_type} event={event_name}"
        )

    def _on_relocalization(self, message: RelocalizationEvent) -> None:
        event_name = RELOCALIZATION_EVENT_NAMES.get(
            message.event, f"UNKNOWN({message.event})"
        )
        self.relocalization_events.append(
            {
                "event": event_name,
                "attempt_id": int(message.attempt_id),
                "monotonic_sec": time.monotonic(),
                "health_score": round(float(message.health_score), 6),
                "elapsed_sec": round(float(message.elapsed_sec), 6),
                "detail": message.detail,
            }
        )


class ManagedProcess:
    """Own one process group and its durable log file."""

    def __init__(self, name: str, command: list[str], log_path: Path) -> None:
        self.name = name
        self.command = command
        self.log_path = log_path
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = log_path.open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen(
                command,
                stdout=self._stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
        except Exception:
            self._stream.close()
            raise

    def poll(self) -> int | None:
        return self.process.poll()

    def stop(self) -> None:
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5)
        self._stream.flush()
        self._stream.close()


def _spin_until(
    node: BenchmarkObserver,
    predicate: Callable[[], bool],
    timeout_sec: float,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def _scenario_complete(
    node: BenchmarkObserver,
    scenario: str,
    clean_hold_sec: float,
) -> bool:
    if scenario == "clean":
        return (
            node.current_state == "HEALTHY"
            and node.last_transition_sec is not None
            and time.monotonic() - node.last_transition_sec >= clean_hold_sec
        )
    return (
        contains_ordered_path(node.states, EXPECTED_PATHS[scenario])
        and any(item["event"] == "ENDED" for item in node.fault_events)
        and (
            (scenario == "visual_occlusion" and len(node.map_ids) == 1)
            or (scenario == "camera_blackout" and len(node.map_ids) >= 2)
        )
    )


def _command_output(command: list[str], cwd: Path | None = None) -> str | None:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def _validate_runtime(args: argparse.Namespace) -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError("ros2 is not on PATH; source ROS 2 Jazzy first")
    if not args.dataset.is_dir() or not (args.dataset / "mav0").is_dir():
        raise RuntimeError(f"dataset is not an extracted EuRoC sequence: {args.dataset}")
    for label, path in (("vocabulary", args.vocabulary), ("settings", args.settings)):
        if not path.is_file():
            raise RuntimeError(f"{label} file does not exist: {path}")
    for package in (
        "vif_bringup",
        "vif_euroc_player",
        "vif_fault_injection",
        "vif_interfaces",
        "vif_orbslam3",
    ):
        if _command_output(["ros2", "pkg", "prefix", package]) is None:
            raise RuntimeError(
                f"ROS package {package} is unavailable; rebuild and source install/setup.bash"
            )


def _backend_command(args: argparse.Namespace) -> list[str]:
    return [
        "ros2",
        "launch",
        "vif_bringup",
        "orbslam3.launch.py",
        f"vocabulary_path:={args.vocabulary}",
        f"settings_path:={args.settings}",
        "use_viewer:=false",
    ]


def _injector_command(args: argparse.Namespace, scenario: str) -> list[str]:
    enabled = scenario != "clean"
    fault_type = scenario if enabled else "visual_occlusion"
    return [
        "ros2",
        "run",
        "vif_fault_injection",
        "fault_injector",
        "--ros-args",
        "-p",
        f"enabled:={'true' if enabled else 'false'}",
        "-p",
        f"fault_type:={fault_type}",
        "-p",
        f"start_after_sec:={args.fault_start_sec}",
        "-p",
        f"duration_sec:={args.fault_duration_sec}",
    ]


def _player_command(args: argparse.Namespace) -> list[str]:
    return [
        "ros2",
        "run",
        "vif_euroc_player",
        "euroc_player",
        "--ros-args",
        "-p",
        f"dataset_path:={args.dataset}",
        "-p",
        f"duration_sec:={args.playback_duration_sec}",
        "-p",
        f"playback_rate:={args.playback_rate}",
        "-p",
        "left_topic:=/vif/raw/camera/left/image_raw",
        "-p",
        "right_topic:=/vif/raw/camera/right/image_raw",
        "-p",
        "imu_topic:=/vif/raw/imu",
        "-p",
        "required_subscriber_count:=1",
    ]


def run_scenario(
    args: argparse.Namespace, scenario: str, output_dir: Path
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    observer = BenchmarkObserver(scenario)
    processes: list[ManagedProcess] = []
    process_errors: list[str] = []
    observation_ended_sec = observer.observation_started_sec

    try:
        for _ in range(5):
            rclpy.spin_once(observer, timeout_sec=0.1)
        existing = observer.runtime_node_names()
        if existing:
            _spin_until(
                observer,
                lambda: not observer.runtime_node_names(),
                10.0,
            )
            existing = observer.runtime_node_names()
        if existing:
            process_errors.append(
                "VIF nodes already running before scenario: " + ", ".join(sorted(existing))
            )
            observation_ended_sec = time.monotonic()
        else:
            backend = ManagedProcess(
                "backend", _backend_command(args), output_dir / "backend.log"
            )
            processes.append(backend)
            ready = _spin_until(
                observer,
                lambda: {
                    "health_monitor",
                    "orbslam3",
                    "relocalization_manager",
                    "sensor_guard",
                }.issubset(observer.runtime_node_names()),
                args.backend_ready_timeout_sec,
            )
            if not ready:
                process_errors.append("ORB-SLAM3 stack did not become ready before timeout")
            elif backend.poll() is not None:
                process_errors.append(
                    f"ORB-SLAM3 backend exited with code {backend.poll()} during startup"
                )
            else:
                injector = ManagedProcess(
                    "injector",
                    _injector_command(args, scenario),
                    output_dir / "injector.log",
                )
                processes.append(injector)
                injector_ready = _spin_until(
                    observer,
                    lambda: "fault_injector" in observer.runtime_node_names(),
                    15.0,
                )
                if not injector_ready:
                    process_errors.append("fault injector did not become ready")
                else:
                    player = ManagedProcess(
                        "player", _player_command(args), output_dir / "player.log"
                    )
                    processes.append(player)
                    deadline = time.monotonic() + args.scenario_timeout_sec
                    while time.monotonic() < deadline:
                        rclpy.spin_once(observer, timeout_sec=0.1)
                        if _scenario_complete(observer, scenario, args.clean_hold_sec):
                            break
                        if backend.poll() is not None:
                            process_errors.append(
                                f"ORB-SLAM3 backend exited with code {backend.poll()}"
                            )
                            break
                        if injector.poll() is not None:
                            process_errors.append(
                                f"fault injector exited with code {injector.poll()}"
                            )
                            break
                        if player.poll() is not None:
                            process_errors.append(
                                "EuRoC playback ended before acceptance criteria were met"
                            )
                            break
                    else:
                        process_errors.append(
                            "scenario timed out before acceptance criteria were met"
                        )

                    if not process_errors:
                        _spin_until(observer, lambda: False, 0.25)
                    observation_ended_sec = time.monotonic()
    except Exception as error:  # report unexpected orchestration errors as evidence
        process_errors.append(f"benchmark orchestration error: {error}")
        observation_ended_sec = time.monotonic()
    finally:
        for process in reversed(processes):
            process.stop()
        observer.destroy_node()

    result = evaluate_scenario(
        scenario,
        state_transitions=observer.state_transitions,
        map_ids=observer.map_ids,
        fault_events=observer.fault_events,
        relocalization_events=observer.relocalization_events,
        observation_started_sec=observer.observation_started_sec,
        observation_ended_sec=observation_ended_sec,
        clean_hold_sec=args.clean_hold_sec,
        process_errors=process_errors,
    )
    result["commands"] = {
        process.name: process.command for process in processes
    }
    (output_dir / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def _metadata(args: argparse.Namespace, project_root: Path) -> dict[str, Any]:
    orb_root = args.orbslam3_root or args.vocabulary.resolve().parent.parent
    return {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "default"),
        "project_git_commit": _command_output(
            ["git", "rev-parse", "HEAD"], cwd=project_root
        ),
        "orbslam3_git_commit": _command_output(
            ["git", "rev-parse", "HEAD"], cwd=orb_root
        ),
    }


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run portfolio-grade VIF/ORB-SLAM3 recovery benchmarks."
    )
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--vocabulary", type=Path, required=True)
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument("--orbslam3-root", type=Path)
    parser.add_argument(
        "--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS)
    )
    parser.add_argument("--fault-start-sec", type=float, default=45.0)
    parser.add_argument("--fault-duration-sec", type=float, default=1.0)
    parser.add_argument("--playback-duration-sec", type=float, default=75.0)
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--clean-hold-sec", type=float, default=10.0)
    parser.add_argument("--backend-ready-timeout-sec", type=float, default=180.0)
    parser.add_argument("--scenario-timeout-sec", type=float, default=100.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.fault_start_sec <= 0.0 or args.fault_duration_sec <= 0.0:
        parser.error("fault timing must be positive")
    if args.playback_duration_sec <= args.fault_start_sec + args.fault_duration_sec:
        parser.error("playback duration must extend beyond the fault window")
    if args.playback_rate <= 0.0 or args.clean_hold_sec <= 0.0:
        parser.error("playback rate and clean hold must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    _validate_runtime(args)
    project_root = Path(__file__).resolve().parents[2]
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = (args.output or project_root / "results" / run_id).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    results: list[dict[str, Any]] = []
    try:
        for scenario in args.scenarios:
            print(f"\n=== Running {scenario} ===", flush=True)
            result = run_scenario(args, scenario, output_dir / scenario)
            results.append(result)
            print(
                f"{scenario}: {result['status']} — "
                f"{'no errors' if not result['errors'] else '; '.join(result['errors'])}",
                flush=True,
            )
            time.sleep(1.0)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    summary = {
        "schema_version": 1,
        "run_id": run_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "project_version": (project_root / "VERSION").read_text(encoding="utf-8").strip(),
        "all_passed": all(item["passed"] for item in results),
        "configuration": {
            "dataset_path": str(args.dataset.resolve()),
            "vocabulary_path": str(args.vocabulary.resolve()),
            "settings_path": str(args.settings.resolve()),
            "scenarios": args.scenarios,
            "fault_start_sec": args.fault_start_sec,
            "fault_duration_sec": args.fault_duration_sec,
            "playback_duration_sec": args.playback_duration_sec,
            "playback_rate": args.playback_rate,
            "clean_hold_sec": args.clean_hold_sec,
        },
        "environment": _metadata(args, project_root),
        "scenarios": results,
    }
    write_summary_artifacts(summary, output_dir)
    print(f"\nReport: {output_dir / 'report.md'}", flush=True)
    print(f"Overall: {'PASS' if summary['all_passed'] else 'FAIL'}", flush=True)
    return 0 if summary["all_passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("benchmark interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print(f"benchmark failed before completion: {error}", file=sys.stderr)
        raise SystemExit(2)
