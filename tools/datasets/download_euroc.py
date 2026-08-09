#!/usr/bin/env python3
"""Download one EuRoC ASL-format ZIP from its official host, without extracting it."""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path


BASE = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
SEQUENCES = {
    "MH_01_easy": f"{BASE}/machine_hall/MH_01_easy/MH_01_easy.zip",
    "MH_02_easy": f"{BASE}/machine_hall/MH_02_easy/MH_02_easy.zip",
    "MH_03_medium": f"{BASE}/machine_hall/MH_03_medium/MH_03_medium.zip",
    "MH_04_difficult": f"{BASE}/machine_hall/MH_04_difficult/MH_04_difficult.zip",
    "MH_05_difficult": f"{BASE}/machine_hall/MH_05_difficult/MH_05_difficult.zip",
    "V1_01_easy": f"{BASE}/vicon_room1/V1_01_easy/V1_01_easy.zip",
    "V1_02_medium": f"{BASE}/vicon_room1/V1_02_medium/V1_02_medium.zip",
    "V1_03_difficult": f"{BASE}/vicon_room1/V1_03_difficult/V1_03_difficult.zip",
    "V2_01_easy": f"{BASE}/vicon_room2/V2_01_easy/V2_01_easy.zip",
    "V2_02_medium": f"{BASE}/vicon_room2/V2_02_medium/V2_02_medium.zip",
    "V2_03_difficult": f"{BASE}/vicon_room2/V2_03_difficult/V2_03_difficult.zip",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(sequence: str, destination: Path, force: bool = False) -> Path:
    destination.mkdir(parents=True, exist_ok=True)
    output = destination / f"{sequence}.zip"
    temporary = destination / f".{sequence}.zip.part"
    if output.exists() and not force:
        if not zipfile.is_zipfile(output):
            raise RuntimeError(f"existing file is not a valid ZIP: {output}")
        return output
    request = urllib.request.Request(
        SEQUENCES[sequence], headers={"User-Agent": "vi-slam-health/0.3 dataset helper"}
    )
    try:
        with urllib.request.urlopen(request, timeout=60) as response, temporary.open("wb") as sink:
            shutil.copyfileobj(response, sink, length=1024 * 1024)
        if not zipfile.is_zipfile(temporary):
            raise RuntimeError("downloaded file is not a valid ZIP")
        temporary.replace(output)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence", choices=sorted(SEQUENCES))
    parser.add_argument("--destination", type=Path, default=Path("data/euroc"))
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    try:
        output = download(args.sequence, args.destination, args.force)
    except Exception as error:
        print(f"download failed: {error}", file=sys.stderr)
        return 1
    print(f"file: {output}")
    print(f"sha256: {sha256(output)}")
    print("Review the EuRoC terms on the official dataset page before use.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
