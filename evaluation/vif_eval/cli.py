from __future__ import annotations

import argparse
import json
from pathlib import Path

from .failure import evaluate_failures
from .trajectory import evaluate_trajectory, read_tum


def _write(result: dict, output: str | None) -> None:
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        Path(output).write_text(rendered, encoding="utf-8")
    print(rendered, end="")


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(prog="vif-eval")
    commands = root.add_subparsers(dest="command", required=True)

    trajectory = commands.add_parser("trajectory", help="Compute aligned ATE and RPE")
    trajectory.add_argument("--estimate", required=True)
    trajectory.add_argument("--ground-truth", required=True)
    trajectory.add_argument("--max-difference-sec", type=float, default=0.02)
    trajectory.add_argument("--rpe-delta-sec", type=float, default=1.0)
    trajectory.add_argument("--align-scale", action="store_true")
    trajectory.add_argument("--output")

    failures = commands.add_parser("failures", help="Evaluate loss and recovery events")
    failures.add_argument("--events", required=True)
    failures.add_argument("--labels", required=True)
    failures.add_argument("--detection-grace-sec", type=float, default=1.0)
    failures.add_argument("--output")
    return root


def main(argv: list[str] | None = None) -> None:
    args = parser().parse_args(argv)
    if args.command == "trajectory":
        result = evaluate_trajectory(
            read_tum(args.estimate),
            read_tum(args.ground_truth),
            max_difference_sec=args.max_difference_sec,
            rpe_delta_sec=args.rpe_delta_sec,
            align_scale=args.align_scale,
        )
    else:
        result = evaluate_failures(
            args.events, args.labels, detection_grace_sec=args.detection_grace_sec
        )
    _write(result, args.output)


if __name__ == "__main__":
    main()
