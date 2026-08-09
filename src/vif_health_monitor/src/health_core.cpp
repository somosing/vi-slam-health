// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vif_health_monitor/health_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vif_health_monitor
{

HealthCore::HealthCore(Parameters parameters)
: parameters_(parameters)
{
  const double weight_sum = parameters_.weight_features + parameters_.weight_inliers +
    parameters_.weight_reprojection + parameters_.weight_sensors +
    parameters_.weight_realtime;
  if (weight_sum <= 0.0) {
    throw std::invalid_argument("health quality weights must have a positive sum");
  }
  if (parameters_.ewma_alpha <= 0.0 || parameters_.ewma_alpha > 1.0) {
    throw std::invalid_argument("ewma_alpha must be in (0, 1]");
  }
}

double HealthCore::Clamp01(double value) {return std::clamp(value, 0.0, 1.0);}

double HealthCore::IncreasingQuality(double value, double bad, double good)
{
  if (good <= bad) {return value >= good ? 1.0 : 0.0;}
  return Clamp01((value - bad) / (good - bad));
}

double HealthCore::DecreasingQuality(double value, double good, double bad)
{
  if (bad <= good) {return value <= good ? 1.0 : 0.0;}
  return 1.0 - Clamp01((value - good) / (bad - good));
}

Output HealthCore::Update(const Input & input)
{
  Output output;
  const bool features_available = std::isfinite(input.tracked_features);
  const bool inliers_available = std::isfinite(input.inlier_ratio);
  const bool reprojection_available = std::isfinite(input.reprojection_error_px);
  const bool realtime_available = std::isfinite(input.processing_time_ms);

  if (features_available) {
    output.feature_quality = IncreasingQuality(
        input.tracked_features, parameters_.feature_min, parameters_.feature_nominal);
  }
  if (inliers_available) {
    output.inlier_quality =
      IncreasingQuality(input.inlier_ratio, parameters_.inlier_bad, parameters_.inlier_good);
  }
  if (reprojection_available) {
    output.reprojection_quality = DecreasingQuality(input.reprojection_error_px,
                                                    parameters_.reprojection_good_px,
                                                    parameters_.reprojection_bad_px);
  }
  output.sensor_quality =
    (input.camera_ok ? 0.5 : 0.0) + (input.imu_ok ? 0.3 : 0.0) +
    (input.timestamps_ok ? 0.1 : 0.0) + (!input.imu_gap ? 0.1 : 0.0);
  if (realtime_available) {
    output.realtime_quality = DecreasingQuality(
        input.processing_time_ms, 0.6 * parameters_.processing_budget_ms,
        parameters_.processing_budget_ms);
  }

  double weighted_quality = parameters_.weight_sensors * output.sensor_quality;
  double available_weight = parameters_.weight_sensors;
  const auto add_available = [&weighted_quality, &available_weight](
    bool available, double quality, double weight) {
      if (available && weight > 0.0) {
        weighted_quality += weight * quality;
        available_weight += weight;
      }
    };
  add_available(features_available, output.feature_quality, parameters_.weight_features);
  add_available(inliers_available, output.inlier_quality, parameters_.weight_inliers);
  add_available(
      reprojection_available, output.reprojection_quality, parameters_.weight_reprojection);
  add_available(realtime_available, output.realtime_quality, parameters_.weight_realtime);
  double raw_score = available_weight > 0.0 ? weighted_quality / available_weight : 0.0;

  if (input.tracking_state == TrackingState::kLost) {raw_score = std::min(raw_score, 0.05);}
  if (!input.camera_ok || !input.imu_ok) {raw_score = std::min(raw_score, 0.20);}
  if (!score_initialized_) {
    filtered_score_ = raw_score;
    score_initialized_ = true;
  } else {
    filtered_score_ = parameters_.ewma_alpha * raw_score +
      (1.0 - parameters_.ewma_alpha) * filtered_score_;
  }

  if (!features_available) {
    output.reasons.emplace_back("feature_count_unavailable");
  } else if (output.feature_quality < 0.35) {
    output.reasons.emplace_back("low_feature_count");
  }
  if (!inliers_available) {
    output.reasons.emplace_back("inlier_ratio_unavailable");
  } else if (output.inlier_quality < 0.35) {
    output.reasons.emplace_back("low_inlier_ratio");
  }
  if (!reprojection_available) {
    output.reasons.emplace_back("reprojection_error_unavailable");
  } else if (output.reprojection_quality < 0.35) {
    output.reasons.emplace_back("high_reprojection_error");
  }
  if (!input.camera_ok) {output.reasons.emplace_back("camera_unavailable");}
  if (!input.imu_ok) {output.reasons.emplace_back("imu_unavailable");}
  if (!input.timestamps_ok) {output.reasons.emplace_back("timestamp_fault");}
  if (input.imu_gap) {output.reasons.emplace_back("imu_gap");}
  if (!realtime_available) {
    output.reasons.emplace_back("processing_time_unavailable");
  } else if (output.realtime_quality < 0.35) {
    output.reasons.emplace_back("processing_overrun");
  }

  const bool backend_initializing =
    input.tracking_state == TrackingState::kNotInitialized ||
    input.tracking_state == TrackingState::kInitializing;

  if (input.map_changed) {
    state_ = HealthState::kResetting;
    recovery_active_ = false;
    healthy_count_ = degraded_count_ = lost_count_ = recovery_count_ =
      recovered_hold_count_ = 0;
    output.reasons.emplace_back("backend_map_changed");
  } else if (backend_initializing) {
    state_ = HealthState::kInitializing;
    healthy_count_ = degraded_count_ = lost_count_ = 0;
  } else if (input.tracking_state == TrackingState::kLost) {
    state_ = HealthState::kLost;
    recovery_active_ = true;
    recovery_count_ = 0;
    output.reasons.emplace_back("backend_tracking_lost");
  } else if (input.tracking_state == TrackingState::kRelocalizing) {
    state_ = HealthState::kRelocalizing;
    recovery_active_ = true;
    recovery_count_ = 0;
    output.reasons.emplace_back("backend_relocalizing");
  } else if (state_ == HealthState::kResetting) {
    state_ = HealthState::kInitializing;
  } else if (recovery_active_) {
    if (filtered_score_ >= parameters_.recovery_threshold) {
      ++recovery_count_;
      if (recovery_count_ >= parameters_.recovery_samples) {
        state_ = HealthState::kRecovered;
        recovery_active_ = false;
        recovered_hold_count_ = 0;
      } else {
        state_ = HealthState::kRelocalizing;
      }
    } else {
      recovery_count_ = 0;
    }
  } else if (state_ == HealthState::kRecovered) {
    ++recovered_hold_count_;
    if (recovered_hold_count_ >= parameters_.recovered_hold_samples) {
      state_ = HealthState::kHealthy;
      recovery_active_ = false;
    }
  } else {
    TransitionFromScore(filtered_score_);
    if (state_ == HealthState::kLost) {
      recovery_active_ = true;
      recovery_count_ = 0;
    }
  }

  output.state = state_;
  output.score = Clamp01(filtered_score_);
  output.pose_valid = state_ == HealthState::kHealthy || state_ == HealthState::kDegraded ||
    state_ == HealthState::kRecovered;
  return output;
}

void HealthCore::TransitionFromScore(double score)
{
  healthy_count_ = score >= parameters_.healthy_threshold ? healthy_count_ + 1 : 0;
  degraded_count_ = score < parameters_.degraded_threshold ? degraded_count_ + 1 : 0;
  lost_count_ = score < parameters_.lost_threshold ? lost_count_ + 1 : 0;

  if (lost_count_ >= parameters_.transition_samples) {
    state_ = HealthState::kLost;
  } else if (degraded_count_ >= parameters_.transition_samples) {
    state_ = HealthState::kDegraded;
  } else if (healthy_count_ >= parameters_.transition_samples) {
    state_ = HealthState::kHealthy;
  }
}

void HealthCore::Reset()
{
  state_ = HealthState::kInitializing;
  recovery_active_ = false;
  score_initialized_ = false;
  filtered_score_ = 0.0;
  healthy_count_ = degraded_count_ = lost_count_ = recovery_count_ = recovered_hold_count_ = 0;
}

}  // namespace vif_health_monitor
