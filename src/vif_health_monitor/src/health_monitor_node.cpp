// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vif_health_monitor/health_core.hpp"
#include "vif_interfaces/msg/localization_health.hpp"
#include "vif_interfaces/msg/sensor_health.hpp"
#include "vif_interfaces/msg/tracking_status.hpp"

namespace vif_health_monitor
{

class HealthMonitorNode final : public rclcpp::Node {
public:
  HealthMonitorNode()
  : Node("health_monitor"), core_(declareParameters())
  {
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.5);
    sensor_sub_ = create_subscription<vif_interfaces::msg::SensorHealth>(
        "/vif/sensors/health", rclcpp::QoS(10).reliable(),
      [this](const vif_interfaces::msg::SensorHealth::ConstSharedPtr msg) {
        std::scoped_lock lock(mutex_);
        sensor_ = *msg;
        sensor_received_ = now();
        have_sensor_ = true;
        });
    tracking_sub_ = create_subscription<vif_interfaces::msg::TrackingStatus>(
        "/vif/slam/tracking", rclcpp::QoS(10).reliable(),
      [this](const vif_interfaces::msg::TrackingStatus::ConstSharedPtr msg) {
        std::scoped_lock lock(mutex_);
        if (have_tracking_ && msg->map_id != tracking_.map_id) {
          map_changed_pending_ = true;
        }
        tracking_ = *msg;
        tracking_received_ = now();
        have_tracking_ = true;
        });
    health_pub_ = create_publisher<vif_interfaces::msg::LocalizationHealth>(
        "/vif/localization/health", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&HealthMonitorNode::evaluate, this));
  }

private:
  Parameters declareParameters()
  {
    Parameters p;
    p.feature_min = declare_parameter<double>("feature_min", p.feature_min);
    p.feature_nominal = declare_parameter<double>("feature_nominal", p.feature_nominal);
    p.inlier_bad = declare_parameter<double>("inlier_bad", p.inlier_bad);
    p.inlier_good = declare_parameter<double>("inlier_good", p.inlier_good);
    p.reprojection_good_px =
      declare_parameter<double>("reprojection_good_px", p.reprojection_good_px);
    p.reprojection_bad_px =
      declare_parameter<double>("reprojection_bad_px", p.reprojection_bad_px);
    p.processing_budget_ms =
      declare_parameter<double>("processing_budget_ms", p.processing_budget_ms);
    p.weight_features = declare_parameter<double>("weight_features", p.weight_features);
    p.weight_inliers = declare_parameter<double>("weight_inliers", p.weight_inliers);
    p.weight_reprojection =
      declare_parameter<double>("weight_reprojection", p.weight_reprojection);
    p.weight_sensors = declare_parameter<double>("weight_sensors", p.weight_sensors);
    p.weight_realtime = declare_parameter<double>("weight_realtime", p.weight_realtime);
    p.ewma_alpha = declare_parameter<double>("ewma_alpha", p.ewma_alpha);
    p.healthy_threshold =
      declare_parameter<double>("healthy_threshold", p.healthy_threshold);
    p.degraded_threshold =
      declare_parameter<double>("degraded_threshold", p.degraded_threshold);
    p.lost_threshold = declare_parameter<double>("lost_threshold", p.lost_threshold);
    p.recovery_threshold =
      declare_parameter<double>("recovery_threshold", p.recovery_threshold);
    p.transition_samples =
      declare_parameter<int>("transition_samples", p.transition_samples);
    p.recovery_samples = declare_parameter<int>("recovery_samples", p.recovery_samples);
    p.recovered_hold_samples =
      declare_parameter<int>("recovered_hold_samples", p.recovered_hold_samples);
    return p;
  }

  void evaluate()
  {
    Input input;
    const auto current = now();
    {
      std::scoped_lock lock(mutex_);
      const bool sensor_fresh = have_sensor_ &&
        (current - sensor_received_).seconds() <= input_timeout_sec_;
      const bool tracking_fresh = have_tracking_ &&
        (current - tracking_received_).seconds() <= input_timeout_sec_;
      input.camera_ok = sensor_fresh && !sensor_.camera_stale && sensor_.stereo_synchronized;
      input.imu_ok = sensor_fresh && !sensor_.imu_stale;
      input.timestamps_ok = sensor_fresh && sensor_.timestamps_monotonic;
      input.imu_gap = !sensor_fresh || sensor_.imu_gap;
      input.map_changed = map_changed_pending_;
      map_changed_pending_ = false;
      if (tracking_fresh) {
        input.tracking_state = static_cast<TrackingState>(tracking_.state);
        input.tracked_features = tracking_.tracked_features;
        input.inlier_ratio = tracking_.inlier_ratio;
        input.reprojection_error_px = tracking_.mean_reprojection_error_px;
        input.processing_time_ms = tracking_.processing_time_ms;
      } else if (have_tracking_) {
        input.tracking_state = TrackingState::kLost;
      }
    }

    const Output output = core_.Update(input);
    vif_interfaces::msg::LocalizationHealth msg;
    msg.header.stamp = current;
    msg.header.frame_id = "map";
    msg.state = static_cast<std::uint8_t>(output.state);
    msg.score = static_cast<float>(output.score);
    msg.pose_valid = output.pose_valid;
    msg.relocalization_requested = output.state == HealthState::kLost;
    msg.feature_quality = static_cast<float>(output.feature_quality);
    msg.inlier_quality = static_cast<float>(output.inlier_quality);
    msg.reprojection_quality = static_cast<float>(output.reprojection_quality);
    msg.sensor_quality = static_cast<float>(output.sensor_quality);
    msg.realtime_quality = static_cast<float>(output.realtime_quality);
    msg.reasons = output.reasons;
    health_pub_->publish(msg);
  }

  HealthCore core_;
  double input_timeout_sec_{0.5};
  std::mutex mutex_;
  bool have_sensor_{false};
  bool have_tracking_{false};
  bool map_changed_pending_{false};
  rclcpp::Time sensor_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tracking_received_{0, 0, RCL_ROS_TIME};
  vif_interfaces::msg::SensorHealth sensor_;
  vif_interfaces::msg::TrackingStatus tracking_;
  rclcpp::Subscription<vif_interfaces::msg::SensorHealth>::SharedPtr sensor_sub_;
  rclcpp::Subscription<vif_interfaces::msg::TrackingStatus>::SharedPtr tracking_sub_;
  rclcpp::Publisher<vif_interfaces::msg::LocalizationHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vif_health_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vif_health_monitor::HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
