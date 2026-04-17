// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "opennav_docking/simple_charging_dock.hpp"

namespace opennav_docking
{

void SimpleChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  is_charging_ = false;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Optionally use battery info to check when charging, else say charging if docked
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_battery_status", rclcpp::ParameterValue(true));

  // Parameters for optional external detection of dock pose
  // nav2_util::declare_parameter_if_not_declared(
  //   node_, name + ".use_external_detection_pose", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(-0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(-1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".filter_coef", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".camera_aruco_max", rclcpp::ParameterValue(0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".camera_aruco_min", rclcpp::ParameterValue(0.2));

  // Charging threshold from BatteryState message
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".charging_threshold", rclcpp::ParameterValue(0.5));

  // Optionally determine if docked via stall detection using joint_states
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_stall_detection", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_velocity_threshold", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_effort_threshold", rclcpp::ParameterValue(1.0));

  // If not using stall detection, this is how close robot should get to pose
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_debounce", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".xy_debounce_threshold", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".yaw_debounce_threshold", rclcpp::ParameterValue(5));

  // Staging pose configuration
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_dynamic_offset", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.35));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_y_offset", rclcpp::ParameterValue(-0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));

  // Base frame for docking
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".base_frame", rclcpp::ParameterValue("base_link"));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".dock_pose_history_file_path", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".dock_pose_history_suffix_datetime", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".pose_match_max_delta_ms", rclcpp::ParameterValue(100));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".pose_sync_timer_period_ms", rclcpp::ParameterValue(20));

  node_->get_parameter(name + ".use_battery_status", use_battery_status_);
  // node_->get_parameter(name + ".use_external_detection_pose", use_external_detection_pose_);
  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);
  external_detection_rotation_.setEuler(pitch, roll, yaw);
  node_->get_parameter(name + ".charging_threshold", charging_threshold_);
  node_->get_parameter(name + ".stall_velocity_threshold", stall_velocity_threshold_);
  node_->get_parameter(name + ".stall_effort_threshold", stall_effort_threshold_);
  node_->get_parameter(name + ".docking_threshold", docking_threshold_);
  node_->get_parameter(name + ".use_debounce", use_debounce_);
  node_->get_parameter(name + ".xy_debounce_threshold", xy_debounce_threshold_);
  node_->get_parameter(name + ".yaw_debounce_threshold", yaw_debounce_threshold_);
  node_->get_parameter(name + ".use_dynamic_offset", use_dynamic_offset_);
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_y_offset", staging_y_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);
  node_->get_parameter(name + ".base_frame", base_frame_);
  node_->get_parameter(name + ".dock_pose_history_file_path", dock_pose_history_file_path_);
  node_->get_parameter(name + ".dock_pose_history_suffix_datetime", dock_pose_history_suffix_datetime_);
  node_->get_parameter(name + ".pose_match_max_delta_ms", pose_match_max_delta_ms_);
  node_->get_parameter(name + ".pose_sync_timer_period_ms", pose_sync_timer_period_ms_);

  pose_sync_timer_period_ms_ = std::clamp(
    pose_sync_timer_period_ms_,
    kPoseSyncTimerPeriodMinMs,
    kPoseSyncTimerPeriodMaxMs);

  if (dock_pose_history_file_.is_open()) {
    dock_pose_history_file_.flush();
    dock_pose_history_file_.close();
  }

  if (!dock_pose_history_file_path_.empty()) {
    std::string output_file_path = dock_pose_history_file_path_;
    if (dock_pose_history_suffix_datetime_) {
      const auto now = std::chrono::system_clock::now();
      const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
      std::tm tm_now;
#ifdef _WIN32
      localtime_s(&tm_now, &now_c);
#else
      localtime_r(&now_c, &tm_now);
#endif
      char time_buf[32];
      std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", &tm_now);

      const std::string suffix = std::string("_") + time_buf;
      const auto slash_pos = output_file_path.find_last_of("/\\");
      const auto dot_pos = output_file_path.find_last_of('.');
      if (dot_pos != std::string::npos && (slash_pos == std::string::npos || dot_pos > slash_pos)) {
        output_file_path.insert(dot_pos, suffix);
      } else {
        output_file_path += suffix;
      }
    }

    dock_pose_history_file_.open(output_file_path, std::ios::out | std::ios::trunc);
    if (!dock_pose_history_file_.is_open()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to open dock pose history file: %s",
        output_file_path.c_str());
    } else {
      dock_pose_history_file_path_ = output_file_path;
      dock_pose_history_file_ << "time_sec,time_ns,frame_id,x,y,z,yaw_rad\n";
      RCLCPP_INFO(
        node_->get_logger(),
        "Dock pose history logging enabled: %s",
        dock_pose_history_file_path_.c_str());
    }
  }

  // Setup filter
  double filter_coef;
  node_->get_parameter(name + ".filter_coef", filter_coef);
  filter_ = std::make_unique<PoseFilter>(filter_coef, external_detection_timeout_);
  
  // Get camera ArUco detection distance thresholds
  node_->get_parameter(name + ".camera_aruco_max", camera_aruco_max_);
  node_->get_parameter(name + ".camera_aruco_min", camera_aruco_min_);

  // Set up nav type selector
  nav_type_selector_ = std::make_unique<NavTypeSelector>(node_);

  offset_direction_ = 'x';
  dock_positive_ = true;
  dock_w_cam_ = false;
  dock_offset_z_ = 0.0;
  reset_flag_ = false;
  domain_id_ = 50;

  if (use_battery_status_) {
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", 1,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr state) {
        is_charging_ = state->current > charging_threshold_;
      });
  }


  const char* domain_id_str = std::getenv("ROS_DOMAIN_ID");
  if ( domain_id_str != nullptr ) {
    domain_id_ = std::atoi(domain_id_str);
    if ( 11 <= domain_id_ && domain_id_ <= 14 ) {
      RCLCPP_INFO(node_->get_logger(), "ROS_DOMAIN_ID: %d (expected range)", domain_id_);
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "ROS_DOMAIN_ID: %d (outside expected range 11-14)", domain_id_);
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "ROS_DOMAIN_ID not set, using default");
  }

  dock_pose_.header.stamp = rclcpp::Time(0);
  
  // Configure QoS for real-time camera detection with best-effort delivery
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();
  
  dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", qos,
    std::bind(&SimpleChargingDock::detectedDockPoseCallback, this, std::placeholders::_1));

  // Subscribe to dock controller selector
  dock_controller_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/dock_controller_type",
      rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        // bool was_cam_mode = dock_w_cam_;
        if ( msg->data == "Cam" || msg->data == "CamFront" ) {
          dock_w_cam_ = true;
          // if ( !was_cam_mode ) {
          //   resetDockPoseSubscription();
          // }
        }
        else {
          dock_w_cam_ = false;
          use_external_detection_pose_ = false;
          std::scoped_lock<std::mutex> lock(pose_sync_mutex_);
          detected_dock_pose_queue_.clear();
          final_pose_queue_.clear();
          has_processed_dock_pose_cached_ = false;
        }
        RCLCPP_INFO(node_->get_logger(), "Dock controller type changed to: %s, dock_w_cam_: %s",
          msg->data.c_str(), dock_w_cam_ ? "true" : "false");
  });

  bool use_stall_detection;
  node_->get_parameter(name + ".use_stall_detection", use_stall_detection);
  if (use_stall_detection) {
    is_stalled_ = false;
    node_->get_parameter(name + ".stall_joint_names", stall_joint_names_);
    if (stall_joint_names_.size() < 1) {
      RCLCPP_ERROR(node_->get_logger(), "stall_joint_names cannot be empty!");
    }
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1,
      std::bind(&SimpleChargingDock::jointStateCallback, this, std::placeholders::_1));
  }

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  filtered_dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);

  // Subscribe to final_pose_nav topic
  final_pose_nav_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/final_pose", 10,
    std::bind(&SimpleChargingDock::finalPoseNavCallback, this, std::placeholders::_1));

  pose_sync_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(pose_sync_timer_period_ms_),
    std::bind(&SimpleChargingDock::poseSyncTimerCallback, this));

  // Subscribe to dock_side topic
  dock_side_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
    "/robot/dock_side", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) {
      cam_side_ = msg->data;
      if ( cam_side_ < 0 || cam_side_ > 3 )
        RCLCPP_WARN(node_->get_logger(), "/robot/dock_side:%d is not in valid range: 0-3", cam_side_);
  });
}


void SimpleChargingDock::detectedDockPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  if (!pose) {
    return;
  }

  if (!dock_w_cam_) {
    return;
  }

  use_external_detection_pose_ = true;

  std::scoped_lock<std::mutex> lock(pose_sync_mutex_);
  if (!detected_dock_pose_queue_.empty()) {
    const auto last_stamp = rclcpp::Time(detected_dock_pose_queue_.back().header.stamp);
    const auto curr_stamp = rclcpp::Time(pose->header.stamp);
    if (curr_stamp <= last_stamp) {
      return;
    }
  }

  detected_dock_pose_ = *pose;
  detected_dock_pose_queue_.push_back(*pose);
  while (detected_dock_pose_queue_.size() > kPoseQueueCap) {
    detected_dock_pose_queue_.pop_front();
  }
}


void SimpleChargingDock::finalPoseNavCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  final_pose_nav_ = *msg;

  geometry_msgs::msg::PoseStamped final_pose;
  final_pose.header = msg->header;
  final_pose.pose = msg->pose.pose;

  std::scoped_lock<std::mutex> lock(pose_sync_mutex_);
  if (!final_pose_queue_.empty()) {
    const auto last_stamp = rclcpp::Time(final_pose_queue_.back().header.stamp);
    const auto curr_stamp = rclcpp::Time(final_pose.header.stamp);
    if (curr_stamp <= last_stamp) {
      return;
    }
  }

  final_pose_queue_.push_back(final_pose);
  while (final_pose_queue_.size() > kPoseQueueCap) {
    final_pose_queue_.pop_front();
  }
}


bool SimpleChargingDock::processMatchedPose(
  const geometry_msgs::msg::PoseStamped & detected_pose,
  const geometry_msgs::msg::PoseStamped & matched_final_pose)
{
  if (matched_final_pose.header.frame_id.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped detected = detected_pose;

  if (cam_side_ == 0) {
    detected.pose.position.y -= fabs(dock_offset_z_);
  } else if (cam_side_ == 1) {
    detected.pose.position.x -= fabs(dock_offset_z_);
  } else if (cam_side_ == 2) {
    detected.pose.position.y += fabs(dock_offset_z_);
  } else if (cam_side_ == 3) {
    detected.pose.position.x += fabs(dock_offset_z_);
  }

  const auto & target_frame = matched_final_pose.header.frame_id;
  if (detected.header.frame_id != target_frame) {
    try {
      if (!tf2_buffer_->canTransform(
          target_frame, detected.header.frame_id,
          detected.header.stamp, rclcpp::Duration::from_seconds(0.2)))
      {
        return false;
      }
      tf2_buffer_->transform(detected, detected, target_frame);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  detected = filter_->update(detected);
  filtered_dock_pose_pub_->publish(detected);

  geometry_msgs::msg::PoseStamped processed;
  processed.header = detected.header;
  processed.header.stamp = matched_final_pose.header.stamp;
  processed.pose.position = detected.pose.position;

  if (!ignore_detected_orientation_) {
    geometry_msgs::msg::PoseStamped just_orientation;
    just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.rotation = detected.pose.orientation;
    tf2::doTransform(just_orientation, just_orientation, transform);

    tf2::Quaternion orientation;
    orientation.setEuler(0.0, 0.0, tf2::getYaw(just_orientation.pose.orientation));
    processed.pose.orientation = tf2::toMsg(orientation);
  } else {
    processed.pose.orientation = matched_final_pose.pose.orientation;
  }

  {
    std::scoped_lock<std::mutex> lock(pose_sync_mutex_);
    processed_dock_pose_cached_ = processed;
    has_processed_dock_pose_cached_ = true;
  }

  return true;
}


void SimpleChargingDock::poseSyncTimerCallback()
{
  if (!dock_w_cam_) {
    return;
  }

  geometry_msgs::msg::PoseStamped newest_detected;
  geometry_msgs::msg::PoseStamped matched_final;

  {
    std::scoped_lock<std::mutex> lock(pose_sync_mutex_);

    if (detected_dock_pose_queue_.empty() || final_pose_queue_.empty()) {
      return;
    }

    const auto now = node_->now();
    const auto max_age = rclcpp::Duration::from_seconds(kPoseQueueMaxAgeSec);
    while (!detected_dock_pose_queue_.empty() &&
      (now - rclcpp::Time(detected_dock_pose_queue_.front().header.stamp)) > max_age)
    {
      detected_dock_pose_queue_.pop_front();
    }
    while (!final_pose_queue_.empty() &&
      (now - rclcpp::Time(final_pose_queue_.front().header.stamp)) > max_age)
    {
      final_pose_queue_.pop_front();
    }

    if (detected_dock_pose_queue_.empty() || final_pose_queue_.empty()) {
      return;
    }

    newest_detected = detected_dock_pose_queue_.back();
    const auto detected_stamp_ns = rclcpp::Time(newest_detected.header.stamp).nanoseconds();
    const int64_t max_delta_ns = static_cast<int64_t>(pose_match_max_delta_ms_) * 1000000LL;

    size_t best_idx = final_pose_queue_.size();
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    for (size_t i = 0; i < final_pose_queue_.size(); ++i) {
      const int64_t final_stamp_ns = rclcpp::Time(final_pose_queue_[i].header.stamp).nanoseconds();
      const int64_t delta_ns = std::llabs(final_stamp_ns - detected_stamp_ns);
      if (delta_ns < best_delta_ns) {
        best_delta_ns = delta_ns;
        best_idx = i;
      }
    }

    if (best_idx == final_pose_queue_.size() || best_delta_ns > max_delta_ns) {
      return;
    }

    matched_final = final_pose_queue_[best_idx];
    for (size_t i = 0; i <= best_idx && !final_pose_queue_.empty(); ++i) {
      final_pose_queue_.pop_front();
    }
  }

  processMatchedPose(newest_detected, matched_final);
}


void SimpleChargingDock::cleanup()
{
  if (pose_sync_timer_) {
    pose_sync_timer_->cancel();
    pose_sync_timer_.reset();
  }

  {
    std::scoped_lock<std::mutex> lock(pose_sync_mutex_);
    detected_dock_pose_queue_.clear();
    final_pose_queue_.clear();
    has_processed_dock_pose_cached_ = false;
  }

  if (dock_pose_history_file_.is_open()) {
    dock_pose_history_file_.flush();
    dock_pose_history_file_.close();
    if (node_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Dock pose history logging disabled: %s",
        dock_pose_history_file_path_.c_str());
    }
  }
}



void SimpleChargingDock::recordDockPoseHistory(const geometry_msgs::msg::PoseStamped & dock_pose)
{
  if (!dock_pose_history_file_.is_open()) {
    return;
  }

  geometry_msgs::msg::PoseStamped sample = dock_pose;
  const auto stamp = rclcpp::Time(sample.header.stamp);
  const double stamp_sec = stamp.seconds();
  const int64_t stamp_ns = stamp.nanoseconds();

  dock_pose_history_file_
    << std::fixed << std::setprecision(9)
    << stamp_sec << ","
    << stamp_ns << ","
    << sample.header.frame_id << ","
    << sample.pose.position.x << ","
    << sample.pose.position.y << ","
    << sample.pose.position.z << ","
    << tf2::getYaw(sample.pose.orientation) << "\n";
}


geometry_msgs::msg::PoseStamped SimpleChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame, const std::string & dock_type)
{
  {
    std::scoped_lock<std::mutex> lock(pose_sync_mutex_);
    detected_dock_pose_queue_.clear();
    final_pose_queue_.clear();
    has_processed_dock_pose_cached_ = false;
  }

  // reset_flag_ = false;
  // reset_timer_flag_ = false;
  if (dock_type.find("cam") != std::string::npos) {
    dock_w_cam_ = true;
  } else {
    dock_w_cam_ = false;
    use_external_detection_pose_ = false;
  }

  // ** If not using detection, set the dock pose as the given dock pose estimate
  if (!use_external_detection_pose_ || !dock_w_cam_ ) {
    // This gets called at the start of docking
    // Reset our internally tracked dock pose
    // dock_pose_.header.frame_id = frame;
    // dock_pose_.pose = pose;
    detected_dock_pose_prev_.header.frame_id = frame;
    detected_dock_pose_prev_.header.stamp = node_->now();
    detected_dock_pose_prev_.pose = pose;
  }

  // Compute the staging pose with given offsets
  const double yaw = tf2::getYaw(pose.orientation);
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();
  staging_pose.pose = pose;
  staging_pose.pose.orientation = pose.orientation;

  //** Store the original z-offset value for use in getRefinedPose
  dock_offset_z_ = pose.position.z;

  //** Apply x and y offsets
  if(use_dynamic_offset_) {
    if ( dock_w_cam_ ) {
      nav_type_selector_->setType(dock_type, offset_direction_, staging_pose, computeExternalDockingDist(pose.position.z) );
    }
    else {
      nav_type_selector_->setType(dock_type, offset_direction_, staging_pose, pose.position.z);
    }
    if ( pose.position.z > 0 ) dock_positive_ = true;
    else dock_positive_ = false;
  } else {
    staging_pose.pose.position.x += cos(yaw) * staging_x_offset_ - sin(yaw) * staging_y_offset_;
    staging_pose.pose.position.y += sin(yaw) * staging_x_offset_ + cos(yaw) * staging_y_offset_;

    tf2::Quaternion orientation;
    orientation.setEuler(0.0, 0.0, yaw + staging_yaw_offset_);
    staging_pose.pose.orientation = tf2::toMsg(orientation);
  }

  // Publish staging pose for debugging purposes

  staging_pose_pub_->publish(staging_pose);

  RCLCPP_INFO(node_->get_logger(), "Staging pose: frame=%s, pos=(%.3f, %.3f, %.3f), yaw=%.3f",
  staging_pose.header.frame_id.c_str(),
  staging_pose.pose.position.x,
  staging_pose.pose.position.y,
  staging_pose.pose.position.z,
  tf2::getYaw(staging_pose.pose.orientation));

return staging_pose;

}

bool SimpleChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  if ( !dock_w_cam_ ) {
    use_external_detection_pose_ = false;
  }

  if (dock_w_cam_) {
    geometry_msgs::msg::PoseStamped cached_pose;
    bool has_cached = false;
    {
      std::scoped_lock<std::mutex> lock(pose_sync_mutex_);
      has_cached = has_processed_dock_pose_cached_;
      if (has_cached) {
        cached_pose = processed_dock_pose_cached_;
      }
    }

    if (has_cached && (node_->now() - cached_pose.header.stamp) <=
      rclcpp::Duration::from_seconds(external_detection_timeout_))
    {
      dock_pose_ = cached_pose;
      detected_dock_pose_prev_ = cached_pose;
      dock_pose_pub_->publish(dock_pose_);
      pose = dock_pose_;
      recordDockPoseHistory(dock_pose_);
      return true;
    }

    use_external_detection_pose_ = false;
  }

  // ** If using not detection, set the dock pose to the static fixed-frame version
  if (!use_external_detection_pose_) {
    if(detected_dock_pose_prev_.header.frame_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "No frame for detected dock pose");
    }
      // RCLCPP_WARN(node_->get_logger(), "No f");
    dock_pose_pub_->publish(detected_dock_pose_prev_);
    dock_pose_ = detected_dock_pose_prev_;
    dock_pose_.header.stamp = node_->now();
    recordDockPoseHistory(dock_pose_);
    return true;
  }

  return false;
}

bool SimpleChargingDock::isDocked()
{
  if (joint_state_sub_) {
    // Using stall detection
    return is_stalled_;
  }

  if (dock_pose_.header.frame_id.empty()) {
    // Dock pose is not yet valid
    return false;
  }

  // Find base pose in target frame
  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.stamp = rclcpp::Time(0);
  base_pose.header.frame_id = base_frame_;
  base_pose.pose.orientation.w = 1.0;
  try {
    tf2_buffer_->transform(base_pose, base_pose, dock_pose_.header.frame_id);
  } catch (const tf2::TransformException & ex) {
    return false;
  }

  // If we are close enough, pretend we are charging
  double d = 0.0;
  if(offset_direction_ == 'x' && !dock_w_cam_ ) {
    d = fabs(base_pose.pose.position.x - dock_pose_.pose.position.x);
  } else if(offset_direction_ == 'y' && !dock_w_cam_ ) {
    d = fabs(base_pose.pose.position.y - dock_pose_.pose.position.y);
  } else {
    d = hypot(
      base_pose.pose.position.x - dock_pose_.pose.position.x,
      base_pose.pose.position.y - dock_pose_.pose.position.y);
  }

  if(use_debounce_) {
    if (d < docking_threshold_) {
      xy_debounce_counter_++;
    } else {
      xy_debounce_counter_ = 0;
    }
    return xy_debounce_counter_ > xy_debounce_threshold_;
  } else  return d < docking_threshold_;
}

bool SimpleChargingDock::isCharging()
{
  return use_battery_status_ ? is_charging_ : isDocked();
}

bool SimpleChargingDock::disableCharging()
{
  return true;
}

bool SimpleChargingDock::hasStoppedCharging()
{
  return !isCharging();
}

void SimpleChargingDock::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
  double velocity = 0.0;
  double effort = 0.0;
  for (size_t i = 0; i < state->name.size(); ++i) {
    for (auto & name : stall_joint_names_) {
      if (state->name[i] == name) {
        // Tracking this joint
        velocity += abs(state->velocity[i]);
        effort += abs(state->effort[i]);
      }
    }
  }

  // Take average
  effort /= stall_joint_names_.size();
  velocity /= stall_joint_names_.size();

  is_stalled_ = (velocity < stall_velocity_threshold_) && (effort > stall_effort_threshold_);
}

double SimpleChargingDock::computeExternalDockingDist(const double z)
{
  // Transform camera z-distance to docking distance
  // Linearly decreasing: closer marker (smaller z) -> larger staging distance for safety
  // Further marker (larger z) -> smaller staging distance
  // z should be larger than camera_aruco_min_, or else docking with camera become no that usefull theoretically
  
  

  // const double min_docking_dist = (camera_aruco_max_ - camera_aruco_min_) * 0.2;  // Minimum staging distance (when marker is far)
  // const double max_docking_dist = (camera_aruco_max_ - camera_aruco_min_) * 0.9;  // Maximum staging distance (when marker is very close)
  // const double z_far = (camera_aruco_max_ + camera_aruco_min_) / 2;              // Z distance considered "far"
  // const double z_close = camera_aruco_min_;           // Z distance considered "close"
  
  // // Linear mapping: staging_dist = max when z = z_close, min when z = z_far
  // double slope = (min_docking_dist - max_docking_dist) / (z_far - z_close);
  // double docking_dist = max_docking_dist + slope * (fabs(z) - z_close);
  
  // // Clamp to safe range
  // double  abs_result = std::clamp(docking_dist, 0.0, camera_aruco_max_ - fabs(z));
  // if ( z > 0 ) return abs_result;
  // else return -1.0*abs_result;

  // For simplicity, all dock for 0.2
  // but for side 0(only black), only dock for 0.045 since the camera is down
  if ( cam_side_ == 0 ) {
    if ( z > 0 ) return 0.045;
    else return -0.045;
  }
  else {
    if ( z > 0 ) return 0.22;
    else return -0.22;
  }
}

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::SimpleChargingDock, opennav_docking_core::ChargingDock)
