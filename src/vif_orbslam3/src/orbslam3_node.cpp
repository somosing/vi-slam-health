// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "ImuTypes.h"
#include "System.h"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vif_interfaces/msg/tracking_status.hpp"
#include "vif_interfaces/srv/manage_slam.hpp"
#include "vif_orbslam3/adapter_core.hpp"

namespace vif_orbslam3
{

namespace
{

constexpr int kOrbOk = 2;
constexpr int kOrbRecentlyLost = 3;
constexpr int kOrbOkKlt = 5;

Eigen::Isometry3d ReadRigidTransform(
  const std::vector<double> & row_major, const std::string & parameter_name)
{
  if (row_major.size() != 16U) {
    throw std::invalid_argument(parameter_name + " must contain 16 row-major values");
  }

  Eigen::Matrix4d matrix;
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      matrix(row, column) = row_major[static_cast<std::size_t>(4 * row + column)];
    }
  }

  const Eigen::Vector4d expected_last_row(0.0, 0.0, 0.0, 1.0);
  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  if (!matrix.allFinite() ||
    !matrix.row(3).transpose().isApprox(expected_last_row, 1.0e-9) ||
    !rotation.transpose().isApprox(rotation.inverse(), 1.0e-6) ||
    std::abs(rotation.determinant() - 1.0) > 1.0e-6)
  {
    throw std::invalid_argument(parameter_name + " must be a finite rigid transform");
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.matrix() = matrix;
  return transform;
}

std::uint32_t SaturatingUint32(std::uint64_t value)
{
  return static_cast<std::uint32_t>(
    std::min<std::uint64_t>(value, std::numeric_limits<std::uint32_t>::max()));
}

bool ImuValuesFinite(const sensor_msgs::msg::Imu & msg)
{
  return std::isfinite(msg.linear_acceleration.x) &&
         std::isfinite(msg.linear_acceleration.y) &&
         std::isfinite(msg.linear_acceleration.z) &&
         std::isfinite(msg.angular_velocity.x) &&
         std::isfinite(msg.angular_velocity.y) &&
         std::isfinite(msg.angular_velocity.z);
}

}  // namespace

class OrbSlam3Node final : public rclcpp::Node {
public:
  using Image = sensor_msgs::msg::Image;
  using ImagePairPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using TrackingStatus = vif_interfaces::msg::TrackingStatus;
  using ManageSlam = vif_interfaces::srv::ManageSlam;

  OrbSlam3Node()
  : Node("orbslam3")
  {
    const std::string vocabulary_path =
      declare_parameter<std::string>("vocabulary_path", "");
    const std::string settings_path = declare_parameter<std::string>("settings_path", "");
    const bool use_viewer = declare_parameter<bool>("use_viewer", false);
    sensor_mode_ = declare_parameter<std::string>("sensor_mode", "stereo_imu");
    const auto left_topic = declare_parameter<std::string>(
      "left_topic", "/vif/camera/left/image_raw");
    const auto right_topic = declare_parameter<std::string>(
      "right_topic", "/vif/camera/right/image_raw");
    const auto rgb_topic = declare_parameter<std::string>(
      "rgb_topic", "/vif/camera/color/image_raw");
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/vif/camera/depth/image_raw");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/vif/imu");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>(
      "base_frame", "camera_left_optical_frame");
    max_stereo_skew_sec_ = declare_parameter<double>("max_stereo_skew_sec", 0.005);
    imu_wait_timeout_sec_ = declare_parameter<double>("imu_wait_timeout_sec", 0.02);
    max_imu_buffer_size_ = static_cast<std::size_t>(
      std::max<std::int64_t>(
        100, declare_parameter<std::int64_t>("max_imu_buffer_size", 4000)));
    publish_frozen_pose_on_loss_ =
      declare_parameter<bool>("publish_frozen_pose_on_loss", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    tracking_covariance_ = declare_parameter<double>("tracking_covariance", 0.01);
    invalid_covariance_ = declare_parameter<double>("invalid_covariance", 1.0e6);

    const std::vector<double> default_orb_world_to_map{
      0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0};
    const std::vector<double> identity{
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0};
    orb_world_to_map_ = ReadRigidTransform(
      declare_parameter<std::vector<double>>(
        "orb_world_to_map", default_orb_world_to_map),
      "orb_world_to_map");
    base_to_camera_ = ReadRigidTransform(
      declare_parameter<std::vector<double>>("base_to_camera", identity),
      "base_to_camera");

    if (sensor_mode_ != "stereo_imu" && sensor_mode_ != "rgbd_imu") {
      throw std::invalid_argument("sensor_mode must be stereo_imu or rgbd_imu");
    }
    if (!std::filesystem::is_regular_file(vocabulary_path)) {
      throw std::invalid_argument("vocabulary_path is not a readable file: " + vocabulary_path);
    }
    if (!std::filesystem::is_regular_file(settings_path)) {
      throw std::invalid_argument("settings_path is not a readable file: " + settings_path);
    }
    if (max_stereo_skew_sec_ <= 0.0 || imu_wait_timeout_sec_ < 0.0 ||
      tracking_covariance_ < 0.0 ||
      invalid_covariance_ <= tracking_covariance_)
    {
      throw std::invalid_argument("invalid synchronization or covariance parameters");
    }

    const auto sensor = sensor_mode_ == "rgbd_imu" ?
      ORB_SLAM3::System::IMU_RGBD : ORB_SLAM3::System::IMU_STEREO;
    system_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_path, settings_path, sensor, use_viewer);

    tracking_pub_ = create_publisher<TrackingStatus>(
      "/vif/slam/tracking", rclcpp::QoS(10).reliable());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/vif/slam/odometry", rclcpp::QoS(10).reliable());
    manage_service_ = create_service<ManageSlam>(
      "/vif/slam/manage",
      std::bind(&OrbSlam3Node::OnManage, this, std::placeholders::_1,
        std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto first_image_topic = sensor_mode_ == "rgbd_imu" ? rgb_topic : left_topic;
    const auto second_image_topic = sensor_mode_ == "rgbd_imu" ? depth_topic : right_topic;
    left_sub_.subscribe(this, first_image_topic, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, second_image_topic, rmw_qos_profile_sensor_data);
    image_pair_sync_ = std::make_shared<message_filters::Synchronizer<ImagePairPolicy>>(
      ImagePairPolicy(20), left_sub_, right_sub_);
    image_pair_sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(max_stereo_skew_sec_));
    image_pair_sync_->registerCallback(
      std::bind(&OrbSlam3Node::OnImagePair, this, std::placeholders::_1,
        std::placeholders::_2));

    imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&OrbSlam3Node::OnImu, this, std::placeholders::_1), imu_options);

#ifdef VIF_ORB_SLAM3_HAS_DIAGNOSTICS
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 adapter started with patched diagnostics");
#else
    RCLCPP_WARN(
      get_logger(),
      "ORB-SLAM3 diagnostics patch is disabled; inlier ratio and reprojection error are NaN");
#endif
  }

  ~OrbSlam3Node() override
  {
    std::scoped_lock lock(backend_mutex_);
    if (system_ && !system_->isShutDown()) {
      system_->Shutdown();
    }
  }

private:
  struct BufferedImu
  {
    double stamp_sec;
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
  };

  void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    if (!std::isfinite(stamp_sec) || !ImuValuesFinite(*msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-finite IMU measurement");
      return;
    }

    std::scoped_lock lock(imu_mutex_);
    if (have_imu_stamp_ && stamp_sec <= last_imu_stamp_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-monotonic IMU timestamp");
      return;
    }
    have_imu_stamp_ = true;
    last_imu_stamp_sec_ = stamp_sec;
    imu_buffer_.push_back(BufferedImu{
        stamp_sec,
        static_cast<float>(msg->linear_acceleration.x),
        static_cast<float>(msg->linear_acceleration.y),
        static_cast<float>(msg->linear_acceleration.z),
        static_cast<float>(msg->angular_velocity.x),
        static_cast<float>(msg->angular_velocity.y),
        static_cast<float>(msg->angular_velocity.z)});
    while (imu_buffer_.size() > max_imu_buffer_size_) {
      imu_buffer_.pop_front();
      ++dropped_imu_samples_;
    }
    imu_condition_.notify_all();
  }

  std::vector<ORB_SLAM3::IMU::Point> CollectImu(double frame_stamp_sec)
  {
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    std::unique_lock lock(imu_mutex_);
    const bool imu_caught_up = imu_condition_.wait_for(
      lock, std::chrono::duration<double>(imu_wait_timeout_sec_),
      [this, frame_stamp_sec]() {
        return (have_imu_stamp_ && last_imu_stamp_sec_ >= frame_stamp_sec) ||
               !rclcpp::ok();
      });
    if (!imu_caught_up) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU did not catch up to the stereo timestamp before the adapter deadline");
    }
    const double lower_bound = have_frame_stamp_ ? last_frame_stamp_sec_ :
      -std::numeric_limits<double>::infinity();
    while (!imu_buffer_.empty() && imu_buffer_.front().stamp_sec <= frame_stamp_sec) {
      const BufferedImu sample = imu_buffer_.front();
      imu_buffer_.pop_front();
      if (sample.stamp_sec > lower_bound) {
        measurements.emplace_back(
          sample.acceleration_x, sample.acceleration_y, sample.acceleration_z,
          sample.angular_velocity_x, sample.angular_velocity_y, sample.angular_velocity_z,
          sample.stamp_sec);
      }
    }
    return measurements;
  }

  void OnImagePair(const Image::ConstSharedPtr & first, const Image::ConstSharedPtr & second)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    const rclcpp::Time first_stamp(first->header.stamp);
    const rclcpp::Time second_stamp(second->header.stamp);
    const double frame_stamp_sec = first_stamp.seconds();
    const double pair_skew_sec = std::abs((first_stamp - second_stamp).seconds());

    if (!std::isfinite(frame_stamp_sec) || pair_skew_sec > max_stereo_skew_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected unsynchronized image pair");
      return;
    }
    if (have_frame_stamp_ && frame_stamp_sec <= last_frame_stamp_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-monotonic image timestamp");
      return;
    }

    cv::Mat first_image;
    cv::Mat second_image;
    try {
      if (sensor_mode_ == "rgbd_imu") {
        first_image = cv_bridge::toCvCopy(
          first, sensor_msgs::image_encodings::RGB8)->image;
        // ORB-SLAM3 reads the depth scale from the camera settings. Preserve the
        // incoming 16-bit or 32-bit depth representation for TrackRGBD.
        second_image = cv_bridge::toCvCopy(second)->image;
        if (second_image.type() != CV_16UC1 && second_image.type() != CV_32FC1) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "RGB-D depth image must be 16UC1 or 32FC1 (received %s)",
            second->encoding.c_str());
          return;
        }
      } else {
        first_image = cv_bridge::toCvCopy(
          first, sensor_msgs::image_encodings::MONO8)->image;
        second_image = cv_bridge::toCvCopy(
          second, sensor_msgs::image_encodings::MONO8)->image;
      }
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(get_logger(), "Image conversion failed: %s", error.what());
      return;
    }

    const auto imu_measurements = CollectImu(frame_stamp_sec);
    Sophus::SE3f camera_from_world;
    int orb_state = -1;
    std::uint32_t tracked_features = 0;
    std::uint32_t published_map_id = 0;
    float inlier_ratio = std::numeric_limits<float>::quiet_NaN();
    float reprojection_error_px = std::numeric_limits<float>::quiet_NaN();

    try {
      std::scoped_lock lock(backend_mutex_);
      if (sensor_mode_ == "rgbd_imu") {
        camera_from_world = system_->TrackRGBD(
          first_image, second_image, frame_stamp_sec, imu_measurements);
      } else {
        camera_from_world = system_->TrackStereo(
          first_image, second_image, frame_stamp_sec, imu_measurements);
      }
      orb_state = system_->GetTrackingState();

#ifdef VIF_ORB_SLAM3_HAS_DIAGNOSTICS
      const auto diagnostics = system_->GetTrackingDiagnostics();
      const auto correspondence_count = std::max(0, diagnostics.matched_correspondences);
      const auto pose_inliers = std::max(0, diagnostics.pose_inliers);
      tracked_features = SaturatingUint32(static_cast<std::uint64_t>(pose_inliers));
      if (correspondence_count > 0) {
        inlier_ratio = std::clamp(
          static_cast<float>(pose_inliers) / static_cast<float>(correspondence_count),
          0.0F, 1.0F);
      }
      if (diagnostics.mean_reprojection_error_px >= 0.0F &&
        std::isfinite(diagnostics.mean_reprojection_error_px))
      {
        reprojection_error_px = diagnostics.mean_reprojection_error_px;
      }
      const std::uint32_t backend_map_id = SaturatingUint32(diagnostics.active_map_id);
      if (have_backend_map_id_ && backend_map_id != backend_map_id_) {
        ++public_map_id_;
        recovery_detector_.Reset();
        have_last_pose_ = false;
      }
      backend_map_id_ = backend_map_id;
      have_backend_map_id_ = true;
#else
      const auto tracked_points = system_->GetTrackedMapPoints();
      tracked_features = SaturatingUint32(static_cast<std::uint64_t>(
            std::count_if(tracked_points.begin(), tracked_points.end(),
            [](const auto * point) {return point != nullptr;})));
#endif
      published_map_id = public_map_id_;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "ORB-SLAM3 camera update failed: %s", error.what());
      return;
    }

    have_frame_stamp_ = true;
    last_frame_stamp_sec_ = frame_stamp_sec;
    const auto processing_end = std::chrono::steady_clock::now();
    const float processing_time_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(processing_end - processing_start).count());

    TrackingStatus status;
    status.header.stamp = first->header.stamp;
    status.header.frame_id = map_frame_;
    status.state = ToVifTrackingState(orb_state);
    status.tracked_features = tracked_features;
    status.inlier_ratio = inlier_ratio;
    status.mean_reprojection_error_px = reprojection_error_px;
    status.processing_time_ms = processing_time_ms;
    status.relocalization_event = recovery_detector_.Observe(orb_state);
    status.map_id = published_map_id;
    status.backend = "orb_slam3_4452a3c";
    tracking_pub_->publish(status);

    PublishPose(camera_from_world, orb_state, first->header.stamp);
    const std::size_t dropped_imu_samples = dropped_imu_samples_.load();
    if (dropped_imu_samples > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU buffer overflow has dropped %zu samples", dropped_imu_samples);
    }
  }

  void PublishPose(
    const Sophus::SE3f & camera_from_world, int orb_state,
    const builtin_interfaces::msg::Time & stamp)
  {
    const bool backend_has_pose =
      orb_state == kOrbOk || orb_state == kOrbOkKlt || orb_state == kOrbRecentlyLost;
    const bool tracking = orb_state == kOrbOk || orb_state == kOrbOkKlt;
    Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
    bool publish_pose = false;

    if (backend_has_pose && camera_from_world.matrix().allFinite()) {
      Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
      world_from_camera.matrix() = camera_from_world.inverse().matrix().cast<double>();
      map_from_base = orb_world_to_map_ * world_from_camera * base_to_camera_;
      publish_pose = map_from_base.matrix().allFinite();
      if (tracking && publish_pose) {
        last_map_from_base_ = map_from_base;
        have_last_pose_ = true;
      }
    } else if (publish_frozen_pose_on_loss_ && have_last_pose_) {
      map_from_base = last_map_from_base_;
      publish_pose = true;
    }

    if (!publish_pose) {
      return;
    }

    Eigen::Quaterniond orientation(map_from_base.rotation());
    orientation.normalize();
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = map_from_base.translation().x();
    odometry.pose.pose.position.y = map_from_base.translation().y();
    odometry.pose.pose.position.z = map_from_base.translation().z();
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    const double covariance = tracking ? tracking_covariance_ : invalid_covariance_;
    for (std::size_t index : {0U, 7U, 14U, 21U, 28U, 35U}) {
      odometry.pose.covariance[index] = covariance;
    }
    odom_pub_->publish(odometry);

    if (tracking && publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = map_from_base.translation().x();
      transform.transform.translation.y = map_from_base.translation().y();
      transform.transform.translation.z = map_from_base.translation().z();
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void OnManage(
    const std::shared_ptr<ManageSlam::Request> request,
    std::shared_ptr<ManageSlam::Response> response)
  {
    std::scoped_lock lock(backend_mutex_);
    response->accepted = false;
    response->map_id = public_map_id_;
    if (!system_ || system_->isShutDown()) {
      response->message = "ORB-SLAM3 is not running";
      return;
    }

    if (request->command == ManageSlam::Request::REQUEST_RELOCALIZATION) {
      response->accepted = true;
      response->message =
        "ORB-SLAM3 automatic place-recognition recovery remains active while frames arrive";
    } else if (request->command == ManageSlam::Request::RESET_ACTIVE_MAP) {
      system_->ResetActiveMap();
      ++public_map_id_;
      ResetTrackingContext();
      response->accepted = true;
      response->map_id = public_map_id_;
      response->message = "active ORB-SLAM3 map reset scheduled";
    } else if (request->command == ManageSlam::Request::START_NEW_MAP) {
      system_->ChangeDataset();
      ++public_map_id_;
      ResetTrackingContext();
      response->accepted = true;
      response->map_id = public_map_id_;
      response->message = "new ORB-SLAM3 atlas map requested";
    } else {
      response->message = "unknown management command";
    }

    RCLCPP_INFO(
      get_logger(), "Manage command %u (%s): accepted=%s",
      request->command, request->reason.c_str(), response->accepted ? "true" : "false");
  }

  void ResetTrackingContext()
  {
    recovery_detector_.Reset();
    have_last_pose_ = false;
    have_frame_stamp_ = false;
    have_backend_map_id_ = false;
    std::scoped_lock lock(imu_mutex_);
    imu_buffer_.clear();
  }

  std::unique_ptr<ORB_SLAM3::System> system_;
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<ImagePairPolicy>> image_pair_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Publisher<TrackingStatus>::SharedPtr tracking_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<ManageSlam>::SharedPtr manage_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex backend_mutex_;
  std::mutex imu_mutex_;
  std::condition_variable imu_condition_;
  std::deque<BufferedImu> imu_buffer_;
  std::size_t max_imu_buffer_size_{4000};
  std::atomic<std::size_t> dropped_imu_samples_{0};
  bool have_imu_stamp_{false};
  double last_imu_stamp_sec_{0.0};
  bool have_frame_stamp_{false};
  double last_frame_stamp_sec_{0.0};
  double max_stereo_skew_sec_{0.005};
  double imu_wait_timeout_sec_{0.02};

  std::string sensor_mode_{"stereo_imu"};
  std::string map_frame_;
  std::string base_frame_;
  Eigen::Isometry3d orb_world_to_map_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d base_to_camera_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_map_from_base_{Eigen::Isometry3d::Identity()};
  bool have_last_pose_{false};
  bool publish_frozen_pose_on_loss_{true};
  bool publish_tf_{true};
  double tracking_covariance_{0.01};
  double invalid_covariance_{1.0e6};
  // VIF-visible generation; ORB-SLAM3 may reuse its internal map ID after reset.
  std::uint32_t public_map_id_{0};
  std::uint32_t backend_map_id_{0};
  bool have_backend_map_id_{false};
  RecoveryPulseDetector recovery_detector_;
};

}  // namespace vif_orbslam3

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vif_orbslam3::OrbSlam3Node>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
