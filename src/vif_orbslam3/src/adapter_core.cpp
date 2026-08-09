// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vif_orbslam3/adapter_core.hpp"

#include "vif_interfaces/msg/tracking_status.hpp"

namespace vif_orbslam3
{

namespace
{
constexpr int kSystemNotReady = -1;
constexpr int kNoImagesYet = 0;
constexpr int kNotInitialized = 1;
constexpr int kOk = 2;
constexpr int kRecentlyLost = 3;
constexpr int kLost = 4;
constexpr int kOkKlt = 5;
}  // namespace

std::uint8_t ToVifTrackingState(int orb_state)
{
  using TrackingStatus = vif_interfaces::msg::TrackingStatus;
  switch (orb_state) {
    case kSystemNotReady:
    case kNoImagesYet:
      return TrackingStatus::NOT_INITIALIZED;
    case kNotInitialized:
      return TrackingStatus::INITIALIZING;
    case kOk:
    case kOkKlt:
      return TrackingStatus::TRACKING;
    case kRecentlyLost:
      return TrackingStatus::RELOCALIZING;
    case kLost:
    default:
      return TrackingStatus::LOST;
  }
}

bool RecoveryPulseDetector::Observe(int orb_state)
{
  if (orb_state == kRecentlyLost || orb_state == kLost) {
    recovery_seen_ = true;
    return false;
  }
  if (recovery_seen_ && (orb_state == kOk || orb_state == kOkKlt)) {
    recovery_seen_ = false;
    return true;
  }
  return false;
}

void RecoveryPulseDetector::Reset()
{
  recovery_seen_ = false;
}

}  // namespace vif_orbslam3
