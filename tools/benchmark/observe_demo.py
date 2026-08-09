#!/usr/bin/env python3
"""Observe and validate an ordered localization-health transition path."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from vif_interfaces.msg import LocalizationHealth, TrackingStatus


NAMES = {
    LocalizationHealth.INITIALIZING: "INITIALIZING",
    LocalizationHealth.HEALTHY: "HEALTHY",
    LocalizationHealth.DEGRADED: "DEGRADED",
    LocalizationHealth.LOST: "LOST",
    LocalizationHealth.RELOCALIZING: "RELOCALIZING",
    LocalizationHealth.RECOVERED: "RECOVERED",
    LocalizationHealth.RESETTING: "RESETTING",
}


class Observer(Node):
    def __init__(self) -> None:
        super().__init__("vif_demo_observer")
        self.states: list[str] = []
        self.map_ids: set[int] = set()
        self.create_subscription(
            LocalizationHealth, "/vif/localization/health", self._on_health, 10
        )
        self.create_subscription(
            TrackingStatus, "/vif/slam/tracking", self._on_tracking, 10
        )

    def _on_health(self, message: LocalizationHealth) -> None:
        name = NAMES.get(message.state, f"UNKNOWN({message.state})")
        if not self.states or self.states[-1] != name:
            self.states.append(name)
            self.get_logger().info(
                f"state={name} score={message.score:.3f} pose_valid={message.pose_valid}"
            )

    def _on_tracking(self, message: TrackingStatus) -> None:
        self.map_ids.add(message.map_id)


EXPECTED_PATHS = {
    "mock-recovery": [
        "HEALTHY",
        "LOST",
        "RELOCALIZING",
        "RECOVERED",
        "HEALTHY",
    ],
    "same-map-recovery": ["HEALTHY", "RELOCALIZING", "RECOVERED", "HEALTHY"],
    "blackout-reset": ["HEALTHY", "LOST", "RESETTING", "INITIALIZING", "HEALTHY"],
}

FORBIDDEN_STATES = {
    "mock-recovery": {"RESETTING"},
    "same-map-recovery": {"RESETTING"},
    "blackout-reset": {"RECOVERED"},
}


def contains_ordered_path(states: list[str], expected: list[str]) -> bool:
    next_index = 0
    for state in states:
        if state == expected[next_index]:
            next_index += 1
            if next_index == len(expected):
                return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=18.0)
    parser.add_argument(
        "--expected-path",
        choices=sorted(EXPECTED_PATHS),
        default="mock-recovery",
    )
    args = parser.parse_args()
    rclpy.init()
    node = Observer()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if contains_ordered_path(node.states, EXPECTED_PATHS[args.expected_path]):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"observed state sequence: {' -> '.join(node.states)}")
    print(f"observed map IDs: {sorted(node.map_ids)}")
    forbidden = FORBIDDEN_STATES[args.expected_path].intersection(node.states)
    if forbidden:
        print(f"FAIL: forbidden states observed: {sorted(forbidden)}")
        return 1
    expected = EXPECTED_PATHS[args.expected_path]
    if not contains_ordered_path(node.states, expected):
        print(f"FAIL: expected ordered path: {' -> '.join(expected)}")
        return 1
    if args.expected_path == "same-map-recovery" and len(node.map_ids) != 1:
        print("FAIL: same-map recovery requires exactly one observed map ID")
        return 1
    if args.expected_path == "blackout-reset" and len(node.map_ids) < 2:
        print("FAIL: blackout reset requires an observed map-ID change")
        return 1
    print(f"PASS: {args.expected_path} path observed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
