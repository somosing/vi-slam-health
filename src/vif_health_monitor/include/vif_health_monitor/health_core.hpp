// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace vif_health_monitor
{

enum class TrackingState : std::uint8_t
{
  kNotInitialized = 0,
  kInitializing = 1,
  kTracking = 2,
  kDegraded = 3,
  kLost = 4,
  kRelocalizing = 5,
};

enum class HealthState : std::uint8_t
{
  kInitializing = 0,
  kHealthy = 1,
  kDegraded = 2,
  kLost = 3,
  kRelocalizing = 4,
  kRecovered = 5,
  kResetting = 6,
};

struct Parameters
{
  double feature_min{20.0};
  double feature_nominal{120.0};
  double inlier_bad{0.25};
  double inlier_good{0.70};
  double reprojection_good_px{1.0};
  double reprojection_bad_px{5.0};
  double processing_budget_ms{50.0};
  double weight_features{0.25};
  double weight_inliers{0.25};
  double weight_reprojection{0.15};
  double weight_sensors{0.25};
  double weight_realtime{0.10};
  double ewma_alpha{0.35};
  double healthy_threshold{0.70};
  double degraded_threshold{0.45};
  double lost_threshold{0.25};
  double recovery_threshold{0.65};
  int transition_samples{3};
  int recovery_samples{8};
  int recovered_hold_samples{10};
};

struct Input
{
  TrackingState tracking_state{TrackingState::kNotInitialized};
  double tracked_features{0.0};
  double inlier_ratio{std::numeric_limits<double>::quiet_NaN()};
  double reprojection_error_px{std::numeric_limits<double>::quiet_NaN()};
  double processing_time_ms{std::numeric_limits<double>::quiet_NaN()};
  bool camera_ok{false};
  bool imu_ok{false};
  bool timestamps_ok{true};
  bool imu_gap{false};
  bool map_changed{false};
};

struct Output
{
  HealthState state{HealthState::kInitializing};
  double score{0.0};
  bool pose_valid{false};
  double feature_quality{std::numeric_limits<double>::quiet_NaN()};
  double inlier_quality{std::numeric_limits<double>::quiet_NaN()};
  double reprojection_quality{std::numeric_limits<double>::quiet_NaN()};
  double sensor_quality{0.0};
  double realtime_quality{std::numeric_limits<double>::quiet_NaN()};
  std::vector<std::string> reasons;
};

class HealthCore {
public:
  explicit HealthCore(Parameters parameters = {});
  Output Update(const Input & input);
  void Reset();

private:
  static double Clamp01(double value);
  static double IncreasingQuality(double value, double bad, double good);
  static double DecreasingQuality(double value, double good, double bad);
  void TransitionFromScore(double score);

  Parameters parameters_;
  HealthState state_{HealthState::kInitializing};
  bool score_initialized_{false};
  double filtered_score_{0.0};
  int healthy_count_{0};
  int degraded_count_{0};
  int lost_count_{0};
  int recovery_count_{0};
  int recovered_hold_count_{0};
  bool recovery_active_{false};
};

}  // namespace vif_health_monitor
