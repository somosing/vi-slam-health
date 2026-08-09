// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later


#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "vif_health_monitor/health_core.hpp"

namespace vh = vif_health_monitor;

namespace
{

vh::Input GoodInput()
{
  vh::Input input;
  input.tracking_state = vh::TrackingState::kTracking;
  input.tracked_features = 150;
  input.inlier_ratio = 0.85;
  input.reprojection_error_px = 0.8;
  input.processing_time_ms = 20.0;
  input.camera_ok = true;
  input.imu_ok = true;
  input.timestamps_ok = true;
  input.imu_gap = false;
  return input;
}

TEST(HealthCore, HealthyInputBecomesValid)
{
  vh::HealthCore core;
  vh::Output output;
  for (int i = 0; i < 5; ++i) {
    output = core.Update(GoodInput());
  }
  EXPECT_EQ(output.state, vh::HealthState::kHealthy);
  EXPECT_TRUE(output.pose_valid);
  EXPECT_GT(output.score, 0.90);
}

TEST(HealthCore, BackendLossImmediatelyInvalidatesPose)
{
  vh::HealthCore core;
  for (int i = 0; i < 5; ++i) {
    core.Update(GoodInput());
  }
  auto input = GoodInput();
  input.tracking_state = vh::TrackingState::kLost;
  const auto output = core.Update(input);
  EXPECT_EQ(output.state, vh::HealthState::kLost);
  EXPECT_FALSE(output.pose_valid);
  EXPECT_NE(std::find(output.reasons.begin(), output.reasons.end(), "backend_tracking_lost"),
            output.reasons.end());
}

TEST(HealthCore, RecoveryRequiresStableSamples)
{
  vh::Parameters p;
  p.recovery_samples = 3;
  p.recovered_hold_samples = 2;
  vh::HealthCore core(p);
  for (int i = 0; i < 5; ++i) {
    core.Update(GoodInput());
  }
  auto lost = GoodInput();
  lost.tracking_state = vh::TrackingState::kLost;
  core.Update(lost);

  auto output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kRelocalizing);
  output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kRelocalizing);
  output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kRecovered);
  EXPECT_TRUE(output.pose_valid);
  core.Update(GoodInput());
  output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kHealthy);
}

TEST(HealthCore, BackendInitializationDoesNotEraseRecoveryContext)
{
  vh::Parameters p;
  p.recovery_samples = 2;
  vh::HealthCore core(p);
  for (int i = 0; i < 5; ++i) {
    core.Update(GoodInput());
  }

  auto input = GoodInput();
  input.tracking_state = vh::TrackingState::kLost;
  core.Update(input);
  input.tracking_state = vh::TrackingState::kInitializing;
  auto output = core.Update(input);
  EXPECT_EQ(output.state, vh::HealthState::kInitializing);
  EXPECT_FALSE(output.pose_valid);

  output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kRelocalizing);
  output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kRecovered);
}

TEST(HealthCore, MapChangeReportsResetAndCannotMasqueradeAsRecovery)
{
  vh::Parameters p;
  p.transition_samples = 2;
  p.recovery_samples = 2;
  vh::HealthCore core(p);
  for (int i = 0; i < 5; ++i) {
    core.Update(GoodInput());
  }

  auto input = GoodInput();
  input.tracking_state = vh::TrackingState::kLost;
  core.Update(input);
  input.tracking_state = vh::TrackingState::kInitializing;
  input.map_changed = true;
  auto output = core.Update(input);
  EXPECT_EQ(output.state, vh::HealthState::kResetting);
  EXPECT_FALSE(output.pose_valid);
  EXPECT_NE(std::find(output.reasons.begin(), output.reasons.end(), "backend_map_changed"),
            output.reasons.end());

  input.map_changed = false;
  output = core.Update(input);
  EXPECT_EQ(output.state, vh::HealthState::kInitializing);
  output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kInitializing);
  output = core.Update(GoodInput());
  EXPECT_EQ(output.state, vh::HealthState::kHealthy);
}

TEST(HealthCore, UnavailableBackendSignalsAreExcludedFromScore)
{
  vh::HealthCore core;
  auto input = GoodInput();
  input.inlier_ratio = std::numeric_limits<double>::quiet_NaN();
  input.reprojection_error_px = std::numeric_limits<double>::quiet_NaN();

  vh::Output output;
  for (int i = 0; i < 5; ++i) {
    output = core.Update(input);
  }

  EXPECT_EQ(output.state, vh::HealthState::kHealthy);
  EXPECT_TRUE(output.pose_valid);
  EXPECT_TRUE(std::isnan(output.inlier_quality));
  EXPECT_TRUE(std::isnan(output.reprojection_quality));
  EXPECT_NE(std::find(output.reasons.begin(), output.reasons.end(),
                      "inlier_ratio_unavailable"), output.reasons.end());
  EXPECT_NE(std::find(output.reasons.begin(), output.reasons.end(),
                      "reprojection_error_unavailable"), output.reasons.end());
}

}  // namespace
