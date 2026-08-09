// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vif_interfaces/msg/localization_health.hpp"
#include "vif_interfaces/msg/relocalization_event.hpp"
#include "vif_interfaces/srv/manage_slam.hpp"

namespace vif_relocalization
{

class RelocalizationManagerNode final : public rclcpp::Node {
public:
  RelocalizationManagerNode()
  : Node("relocalization_manager")
  {
    relocalization_timeout_sec_ = declare_parameter<double>("relocalization_timeout_sec", 8.0);
    failure_timeout_sec_ = declare_parameter<double>("failure_timeout_sec", 16.0);
    cooldown_sec_ = declare_parameter<double>("cooldown_sec", 5.0);
    service_name_ = declare_parameter<std::string>("manage_service", "/vif/slam/manage");

    health_sub_ = create_subscription<vif_interfaces::msg::LocalizationHealth>(
        "/vif/localization/health", rclcpp::QoS(10).reliable(),
        std::bind(&RelocalizationManagerNode::onHealth, this, std::placeholders::_1));
    event_pub_ = create_publisher<vif_interfaces::msg::RelocalizationEvent>(
        "/vif/relocalization/events", rclcpp::QoS(20).reliable());
    client_ = create_client<vif_interfaces::srv::ManageSlam>(service_name_);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&RelocalizationManagerNode::tick, this));
  }

private:
  using Health = vif_interfaces::msg::LocalizationHealth;
  using Event = vif_interfaces::msg::RelocalizationEvent;
  using ManageSlam = vif_interfaces::srv::ManageSlam;

  static bool containsReason(const Health & health, const std::string & reason)
  {
    return std::find(health.reasons.cbegin(), health.reasons.cend(), reason) !=
           health.reasons.cend();
  }

  void onHealth(const Health::ConstSharedPtr msg)
  {
    latest_score_ = msg->score;
    const auto state = msg->state;
    const auto current = now();
    const bool camera_unavailable = containsReason(*msg, "camera_unavailable");

    if (state == Health::LOST && !attempt_active_ &&
      (last_attempt_end_.nanoseconds() == 0 ||
      (current - last_attempt_end_).seconds() >= cooldown_sec_))
    {
      attempt_active_ = true;
      relocalization_sent_ = false;
      reset_sent_ = false;
      immediate_reset_required_ = camera_unavailable;
      started_event_sent_ = false;
      attempt_start_ = current;
      ++attempt_id_;
      publishEvent(Event::REQUESTED, "health monitor reported tracking loss");
    }

    if (attempt_active_ && state == Health::LOST && camera_unavailable) {
      immediate_reset_required_ = true;
    }

    if (attempt_active_ && state == Health::RELOCALIZING && !started_event_sent_) {
      started_event_sent_ = true;
      publishEvent(Event::STARTED, "backend entered relocalization");
    }

    if (attempt_active_ && state == Health::RECOVERED && relocalization_sent_ &&
      !immediate_reset_required_)
    {
      publishEvent(Event::SUCCEEDED, "stable localization recovered");
      finishAttempt();
    }

    if (attempt_active_ && state == Health::RESETTING) {
      publishEvent(
        Event::FAILED,
        immediate_reset_required_ ? "camera outage required active-map reset" :
        reset_sent_ ? "relocalization failed; requested active-map reset observed" :
        "backend reset the active map before relocalization succeeded");
      finishAttempt();
    }
  }

  void tick()
  {
    if (!attempt_active_) {return;}
    const double elapsed = (now() - attempt_start_).seconds();
    if (immediate_reset_required_ && !reset_sent_ && !request_in_flight_) {
      reset_sent_ = requestCommand(
        ManageSlam::Request::RESET_ACTIVE_MAP, "camera input unavailable during tracking loss");
      if (reset_sent_) {
        publishEvent(Event::RESET_REQUESTED, "camera outage requires active-map reset");
      }
    }
    if (!immediate_reset_required_ && !relocalization_sent_ &&
      !request_in_flight_)
    {
      relocalization_sent_ = requestCommand(ManageSlam::Request::REQUEST_RELOCALIZATION,
                                            "tracking loss detected");
    }
    if (!immediate_reset_required_ && elapsed >= relocalization_timeout_sec_ &&
      !reset_sent_ && !request_in_flight_)
    {
      reset_sent_ = requestCommand(ManageSlam::Request::RESET_ACTIVE_MAP,
                                   "relocalization timeout");
      if (reset_sent_) {publishEvent(Event::RESET_REQUESTED, "relocalization timed out");}
    }
    if (elapsed >= failure_timeout_sec_) {
      publishEvent(Event::FAILED, "recovery deadline exceeded");
      finishAttempt();
    }
  }

  bool requestCommand(std::uint8_t command, const std::string & reason)
  {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for SLAM management service %s", service_name_.c_str());
      return false;
    }
    auto request = std::make_shared<ManageSlam::Request>();
    request->command = command;
    request->reason = reason;
    request_in_flight_ = true;
    client_->async_send_request(
        request, [this](rclcpp::Client<ManageSlam>::SharedFuture future) {
        request_in_flight_ = false;
        const auto response = future.get();
        if (!response->accepted) {
          RCLCPP_ERROR(get_logger(), "SLAM rejected recovery command: %s",
                         response->message.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "SLAM accepted recovery command: %s",
                        response->message.c_str());
        }
        });
    return true;
  }

  void publishEvent(std::uint8_t event, const std::string & detail)
  {
    Event msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.event = event;
    msg.attempt_id = attempt_id_;
    msg.health_score = static_cast<float>(latest_score_);
    msg.elapsed_sec =
      attempt_active_ ? static_cast<float>((now() - attempt_start_).seconds()) : 0.0F;
    msg.detail = detail;
    event_pub_->publish(msg);
  }

  void finishAttempt()
  {
    attempt_active_ = false;
    request_in_flight_ = false;
    last_attempt_end_ = now();
  }

  double relocalization_timeout_sec_{8.0};
  double failure_timeout_sec_{16.0};
  double cooldown_sec_{5.0};
  std::string service_name_;
  bool attempt_active_{false};
  bool relocalization_sent_{false};
  bool reset_sent_{false};
  bool immediate_reset_required_{false};
  bool started_event_sent_{false};
  bool request_in_flight_{false};
  std::uint32_t attempt_id_{0};
  double latest_score_{0.0};
  rclcpp::Time attempt_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attempt_end_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Publisher<Event>::SharedPtr event_pub_;
  rclcpp::Client<ManageSlam>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vif_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vif_relocalization::RelocalizationManagerNode>());
  rclcpp::shutdown();
  return 0;
}
