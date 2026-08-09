// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vif_interfaces/msg/sensor_health.hpp"
#include "vif_interfaces/msg/tracking_status.hpp"
#include "vif_interfaces/srv/manage_slam.hpp"

namespace vif_mock_slam
{

class MockSlamNode final : public rclcpp::Node {
public:
  MockSlamNode()
  : Node("mock_slam")
  {
    initialization_sec_ = declare_parameter<double>("initialization_sec", 2.0);
    relocalization_sec_ = declare_parameter<double>("relocalization_sec", 1.5);
    trajectory_radius_m_ = declare_parameter<double>("trajectory_radius_m", 2.0);
    angular_speed_rad_s_ = declare_parameter<double>("angular_speed_rad_s", 0.12);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    sensor_sub_ = create_subscription<vif_interfaces::msg::SensorHealth>(
        "/vif/sensors/health", rclcpp::QoS(10).reliable(),
        std::bind(&MockSlamNode::onSensor, this, std::placeholders::_1));
    tracking_pub_ = create_publisher<vif_interfaces::msg::TrackingStatus>(
        "/vif/slam/tracking", rclcpp::QoS(10).reliable());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/vif/slam/odometry", rclcpp::QoS(10).reliable());
    manage_service_ = create_service<vif_interfaces::srv::ManageSlam>(
        "/vif/slam/manage",
        std::bind(&MockSlamNode::onManage, this, std::placeholders::_1,
                  std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    start_time_ = now();
    state_entered_ = start_time_;
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&MockSlamNode::tick, this));
    RCLCPP_WARN(get_logger(), "Mock backend active: output is synthetic and not real SLAM");
  }

private:
  using SensorHealth = vif_interfaces::msg::SensorHealth;
  using TrackingStatus = vif_interfaces::msg::TrackingStatus;
  using ManageSlam = vif_interfaces::srv::ManageSlam;

  void onSensor(const SensorHealth::ConstSharedPtr msg)
  {
    have_sensor_ = true;
    sensor_ok_ = !msg->camera_stale && !msg->imu_stale && msg->stereo_synchronized &&
      msg->timestamps_monotonic && !msg->imu_gap;
  }

  void onManage(
    const std::shared_ptr<ManageSlam::Request> request,
    std::shared_ptr<ManageSlam::Response> response)
  {
    response->accepted = true;
    response->map_id = map_id_;
    if (request->command == ManageSlam::Request::REQUEST_RELOCALIZATION) {
      recovery_requested_ = true;
      response->message = "relocalization scheduled when sensors are healthy";
    } else if (request->command == ManageSlam::Request::RESET_ACTIVE_MAP) {
      recovery_requested_ = false;
      ++map_id_;
      response->map_id = map_id_;
      setState(TrackingStatus::INITIALIZING);
      response->message = "active mock map reset";
    } else if (request->command == ManageSlam::Request::START_NEW_MAP) {
      recovery_requested_ = false;
      ++map_id_;
      response->map_id = map_id_;
      setState(TrackingStatus::INITIALIZING);
      response->message = "new mock map started";
    } else {
      response->accepted = false;
      response->message = "unknown command";
    }
    RCLCPP_INFO(get_logger(), "Manage command %u: %s", request->command,
                request->reason.c_str());
  }

  void setState(std::uint8_t state)
  {
    if (state_ != state) {
      state_ = state;
      state_entered_ = now();
      RCLCPP_INFO(get_logger(), "Mock tracking state changed to %u", state_);
    }
  }

  void updateState()
  {
    const double state_elapsed = (now() - state_entered_).seconds();
    if (!have_sensor_ || !sensor_ok_) {
      setState(TrackingStatus::LOST);
      return;
    }
    if (state_ == TrackingStatus::NOT_INITIALIZED || state_ == TrackingStatus::INITIALIZING) {
      if (state_elapsed >= initialization_sec_) {setState(TrackingStatus::TRACKING);}
      return;
    }
    if (state_ == TrackingStatus::LOST && recovery_requested_) {
      setState(TrackingStatus::RELOCALIZING);
      return;
    }
    if (state_ == TrackingStatus::RELOCALIZING && state_elapsed >= relocalization_sec_) {
      recovery_requested_ = false;
      relocalization_pulse_ = true;
      setState(TrackingStatus::TRACKING);
    }
  }

  void tick()
  {
    updateState();
    const auto stamp = now();
    TrackingStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = map_frame_;
    status.state = state_;
    status.map_id = map_id_;
    status.backend = "deterministic_mock";
    status.relocalization_event = relocalization_pulse_;
    relocalization_pulse_ = false;

    if (state_ == TrackingStatus::TRACKING) {
      status.tracked_features = 145;
      status.inlier_ratio = 0.86F;
      status.mean_reprojection_error_px = 0.75F;
      status.processing_time_ms = 18.0F;
    } else if (state_ == TrackingStatus::RELOCALIZING) {
      status.tracked_features = 70;
      status.inlier_ratio = 0.55F;
      status.mean_reprojection_error_px = 2.2F;
      status.processing_time_ms = 30.0F;
    } else {
      status.tracked_features = 0;
      status.inlier_ratio = 0.0F;
      status.mean_reprojection_error_px = 10.0F;
      status.processing_time_ms = 5.0F;
    }
    tracking_pub_->publish(status);
    publishOdometry(stamp);
  }

  void publishOdometry(const rclcpp::Time & stamp)
  {
    if (state_ == TrackingStatus::TRACKING || state_ == TrackingStatus::RELOCALIZING) {
      const double t = (stamp - start_time_).seconds();
      const double angle = angular_speed_rad_s_ * t;
      x_ = trajectory_radius_m_ * std::cos(angle);
      y_ = trajectory_radius_m_ * std::sin(angle);
      yaw_ = angle + 1.5707963267948966;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw_);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw_);
    const double covariance = state_ == TrackingStatus::TRACKING ? 0.01 : 1.0e6;
    odom.pose.covariance[0] = covariance;
    odom.pose.covariance[7] = covariance;
    odom.pose.covariance[14] = covariance;
    odom.pose.covariance[21] = covariance;
    odom.pose.covariance[28] = covariance;
    odom.pose.covariance[35] = covariance;
    odom_pub_->publish(odom);

    if (state_ == TrackingStatus::TRACKING) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  double initialization_sec_{2.0};
  double relocalization_sec_{1.5};
  double trajectory_radius_m_{2.0};
  double angular_speed_rad_s_{0.12};
  std::string map_frame_;
  std::string base_frame_;
  bool have_sensor_{false};
  bool sensor_ok_{false};
  bool recovery_requested_{false};
  bool relocalization_pulse_{false};
  std::uint8_t state_{TrackingStatus::INITIALIZING};
  std::uint32_t map_id_{0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_entered_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<SensorHealth>::SharedPtr sensor_sub_;
  rclcpp::Publisher<TrackingStatus>::SharedPtr tracking_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<ManageSlam>::SharedPtr manage_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vif_mock_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vif_mock_slam::MockSlamNode>());
  rclcpp::shutdown();
  return 0;
}
