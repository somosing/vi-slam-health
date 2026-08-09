// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include <gtest/gtest.h>

#include "vif_interfaces/msg/tracking_status.hpp"
#include "vif_orbslam3/adapter_core.hpp"

namespace vo = vif_orbslam3;
using TrackingStatus = vif_interfaces::msg::TrackingStatus;

TEST(AdapterCore, MapsPinnedOrbStatesToContract)
{
  EXPECT_EQ(vo::ToVifTrackingState(-1), TrackingStatus::NOT_INITIALIZED);
  EXPECT_EQ(vo::ToVifTrackingState(0), TrackingStatus::NOT_INITIALIZED);
  EXPECT_EQ(vo::ToVifTrackingState(1), TrackingStatus::INITIALIZING);
  EXPECT_EQ(vo::ToVifTrackingState(2), TrackingStatus::TRACKING);
  EXPECT_EQ(vo::ToVifTrackingState(3), TrackingStatus::RELOCALIZING);
  EXPECT_EQ(vo::ToVifTrackingState(4), TrackingStatus::LOST);
  EXPECT_EQ(vo::ToVifTrackingState(5), TrackingStatus::TRACKING);
  EXPECT_EQ(vo::ToVifTrackingState(99), TrackingStatus::LOST);
}

TEST(AdapterCore, PulsesOnceAfterRecovery)
{
  vo::RecoveryPulseDetector detector;
  EXPECT_FALSE(detector.Observe(2));
  EXPECT_FALSE(detector.Observe(3));
  EXPECT_FALSE(detector.Observe(1));
  EXPECT_TRUE(detector.Observe(2));
  EXPECT_FALSE(detector.Observe(2));
}

TEST(AdapterCore, ResetCancelsPendingPulse)
{
  vo::RecoveryPulseDetector detector;
  EXPECT_FALSE(detector.Observe(4));
  detector.Reset();
  EXPECT_FALSE(detector.Observe(2));
}
