// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vif_interfaces/msg/sensor_health.hpp"

namespace vif_sensor_frontend
{

class SensorGuardNode final : public rclcpp::Node {
public:
  using Image = sensor_msgs::msg::Image;
  using StereoPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  SensorGuardNode()
  : Node("sensor_guard")
  {
    const auto left_topic = declare_parameter<std::string>(
        "left_topic", "/vif/camera/left/image_raw");
    const auto right_topic = declare_parameter<std::string>(
        "right_topic", "/vif/camera/right/image_raw");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/vif/imu");
    camera_stale_sec_ = declare_parameter<double>("camera_stale_sec", 0.35);
    imu_stale_sec_ = declare_parameter<double>("imu_stale_sec", 0.10);
    max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.025);
    max_stereo_skew_sec_ = declare_parameter<double>("max_stereo_skew_sec", 0.005);

    left_sub_.subscribe(this, left_topic, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, right_topic, rmw_qos_profile_sensor_data);
    stereo_sync_ = std::make_shared<message_filters::Synchronizer<StereoPolicy>>(
        StereoPolicy(20), left_sub_, right_sub_);
    stereo_sync_->registerCallback(
        std::bind(&SensorGuardNode::onStereo, this, std::placeholders::_1,
                  std::placeholders::_2));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&SensorGuardNode::onImu, this, std::placeholders::_1));
    health_pub_ = create_publisher<vif_interfaces::msg::SensorHealth>(
        "/vif/sensors/health", rclcpp::QoS(10).reliable());

    rate_window_start_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&SensorGuardNode::publishHealth, this));
  }

private:
  void onStereo(const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right)
  {
    const rclcpp::Time left_stamp(left->header.stamp);
    const rclcpp::Time right_stamp(right->header.stamp);
    const auto arrival = now();

    std::scoped_lock lock(mutex_);
    if (last_camera_stamp_.nanoseconds() != 0 && left_stamp <= last_camera_stamp_) {
      timestamps_monotonic_ = false;
    }
    last_camera_stamp_ = left_stamp;
    last_camera_arrival_ = arrival;
    stereo_synchronized_ =
      std::abs((left_stamp - right_stamp).seconds()) <= max_stereo_skew_sec_;
    ++camera_count_;
  }

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    const auto arrival = now();

    std::scoped_lock lock(mutex_);
    if (last_imu_stamp_.nanoseconds() != 0) {
      const double gap = (stamp - last_imu_stamp_).seconds();
      if (gap <= 0.0) {
        timestamps_monotonic_ = false;
      } else {
        max_observed_imu_gap_sec_ = std::max(max_observed_imu_gap_sec_, gap);
        imu_gap_ = imu_gap_ || gap > max_imu_gap_sec_;
      }
    }
    last_imu_stamp_ = stamp;
    last_imu_arrival_ = arrival;
    ++imu_count_;
  }

  void publishHealth()
  {
    const auto current = now();
    vif_interfaces::msg::SensorHealth msg;
    msg.header.stamp = current;
    msg.header.frame_id = "imu_link";

    std::scoped_lock lock(mutex_);
    const bool no_camera = last_camera_arrival_.nanoseconds() == 0;
    const bool no_imu = last_imu_arrival_.nanoseconds() == 0;
    msg.camera_stale = no_camera || (current - last_camera_arrival_).seconds() > camera_stale_sec_;
    msg.imu_stale = no_imu || (current - last_imu_arrival_).seconds() > imu_stale_sec_;
    msg.stereo_synchronized = stereo_synchronized_ && !msg.camera_stale;
    msg.timestamps_monotonic = timestamps_monotonic_;
    msg.imu_gap = imu_gap_;
    msg.max_imu_gap_ms = static_cast<float>(1000.0 * max_observed_imu_gap_sec_);

    const double elapsed = (current - rate_window_start_).seconds();
    if (elapsed >= 1.0) {
      camera_rate_hz_ = static_cast<double>(camera_count_) / elapsed;
      imu_rate_hz_ = static_cast<double>(imu_count_) / elapsed;
      camera_count_ = 0;
      imu_count_ = 0;
      rate_window_start_ = current;
      max_observed_imu_gap_sec_ = 0.0;
      imu_gap_ = false;
    }
    msg.camera_rate_hz = static_cast<float>(camera_rate_hz_);
    msg.imu_rate_hz = static_cast<float>(imu_rate_hz_);

    if (msg.camera_stale) {msg.reasons.emplace_back("camera_stale");}
    if (msg.imu_stale) {msg.reasons.emplace_back("imu_stale");}
    if (!msg.stereo_synchronized) {msg.reasons.emplace_back("stereo_not_synchronized");}
    if (!msg.timestamps_monotonic) {msg.reasons.emplace_back("non_monotonic_timestamp");}
    if (msg.imu_gap) {msg.reasons.emplace_back("imu_gap");}
    health_pub_->publish(msg);
  }

  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<StereoPolicy>> stereo_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<vif_interfaces::msg::SensorHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  rclcpp::Time last_camera_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_camera_arrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_arrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Time rate_window_start_{0, 0, RCL_ROS_TIME};
  bool stereo_synchronized_{false};
  bool timestamps_monotonic_{true};
  bool imu_gap_{false};
  double camera_stale_sec_{0.35};
  double imu_stale_sec_{0.10};
  double max_imu_gap_sec_{0.025};
  double max_stereo_skew_sec_{0.005};
  double max_observed_imu_gap_sec_{0.0};
  double camera_rate_hz_{0.0};
  double imu_rate_hz_{0.0};
  std::size_t camera_count_{0};
  std::size_t imu_count_{0};
};

}  // namespace vif_sensor_frontend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vif_sensor_frontend::SensorGuardNode>());
  rclcpp::shutdown();
  return 0;
}
