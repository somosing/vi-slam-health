#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST_ROOT="$ROOT/src/third_party"
VENDOR_PIXI_OVERLAY="$ROOT/simulation/mujoco/pixi/mujoco_vendor.toml"
mkdir -p "$DEST_ROOT"

checkout_pinned() {
  local repo_url="$1"
  local destination="$2"
  local revision="$3"

  # Pixi needs a package-local mapping for patchelf, which is a system tool in
  # the upstream manifest rather than a ROS package. Remove our known overlay
  # before checking the pinned Git checkout, then restore it below.
  if [[ "$destination" == "$DEST_ROOT/mujoco_vendor" && -f "$destination/pixi.toml" ]]; then
    if cmp -s "$VENDOR_PIXI_OVERLAY" "$destination/pixi.toml"; then
      rm "$destination/pixi.toml"
    else
      echo "Refusing to replace modified Pixi overlay: $destination/pixi.toml" >&2
      return 1
    fi
  fi

  if [[ -d "$destination/.git" ]]; then
    if [[ -n "$(git -C "$destination" status --porcelain)" ]]; then
      echo "Refusing to move modified checkout: $destination" >&2
      return 1
    fi
    if [[ "$(git -C "$destination" rev-parse HEAD)" == "$revision" ]]; then
      echo "Already pinned: $destination"
      if [[ "$destination" == "$DEST_ROOT/mujoco_vendor" ]]; then
        cp "$VENDOR_PIXI_OVERLAY" "$destination/pixi.toml"
      fi
      return 0
    fi
    git -C "$destination" fetch --depth=1 origin "$revision"
  else
    if [[ -e "$destination" ]]; then
      echo "Refusing to replace non-git path: $destination" >&2
      return 1
    fi
    git clone --filter=blob:none --no-checkout "$repo_url" "$destination"
    git -C "$destination" fetch --depth=1 origin "$revision"
  fi

  git -C "$destination" checkout --detach "$revision"
  if [[ "$destination" == "$DEST_ROOT/mujoco_vendor" ]]; then
    cp "$VENDOR_PIXI_OVERLAY" "$destination/pixi.toml"
  fi
}

checkout_pinned \
  "https://github.com/ros-controls/mujoco_ros2_control.git" \
  "$DEST_ROOT/mujoco_ros2_control" \
  "6838a7fe45a4fc0e9a36ea013032c9d2f9537be7"

checkout_pinned \
  "https://github.com/pal-robotics/mujoco_vendor.git" \
  "$DEST_ROOT/mujoco_vendor" \
  "1c46fbb511b39977aede38aef7c9d335b7f2eb77"

echo "MuJoCo ROS 2 sources are pinned under src/third_party."
