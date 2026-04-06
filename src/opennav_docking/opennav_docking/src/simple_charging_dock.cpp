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

  // Locked pose detection parameter
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".lock_threshold", rclcpp::ParameterValue(10));

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
  node_->get_parameter(name + ".lock_threshold", lock_threshold_);

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
  lock_counter_ = 0;
  is_locked_ = false;
  lock_sum_x_ = 0.0;
  lock_sum_y_ = 0.0;
  lock_sum_z_ = 0.0;
  lock_sum_sin_yaw_ = 0.0;
  lock_sum_cos_yaw_ = 0.0;
  lock_frame_id_.clear();
  lock_latest_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

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
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
      if (is_locked_) {
        use_external_detection_pose_ = true;
        return;
      }
      if (!dock_w_cam_) {
        return;
      }

      if (lock_counter_ == 0) {
        lock_frame_id_ = pose->header.frame_id;
      }

      // Reset accumulation if detector frame changes mid-window.
      if (!lock_frame_id_.empty() && pose->header.frame_id != lock_frame_id_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "detected_dock_pose frame changed during lock averaging (%s -> %s). Resetting average window.",
          lock_frame_id_.c_str(), pose->header.frame_id.c_str());
        lock_counter_ = 0;
        lock_sum_x_ = 0.0;
        lock_sum_y_ = 0.0;
        lock_sum_z_ = 0.0;
        lock_sum_sin_yaw_ = 0.0;
        lock_sum_cos_yaw_ = 0.0;
        lock_frame_id_ = pose->header.frame_id;
      }

      const double sample_yaw = tf2::getYaw(pose->pose.orientation);
      lock_sum_x_ += pose->pose.position.x;
      lock_sum_y_ += pose->pose.position.y;
      lock_sum_z_ += pose->pose.position.z;
      lock_sum_sin_yaw_ += std::sin(sample_yaw);
      lock_sum_cos_yaw_ += std::cos(sample_yaw);
      lock_counter_++;
      lock_latest_stamp_ = rclcpp::Time(pose->header.stamp);

      const int effective_lock_threshold = lock_threshold_ > 0 ? lock_threshold_ : 1;
      if (lock_counter_ >= effective_lock_threshold) {
        geometry_msgs::msg::PoseStamped averaged_pose;
        averaged_pose.header.frame_id = lock_frame_id_;
        averaged_pose.header.stamp = lock_latest_stamp_;

        const double sample_count = static_cast<double>(lock_counter_);
        averaged_pose.pose.position.x = lock_sum_x_ / sample_count;
        averaged_pose.pose.position.y = lock_sum_y_ / sample_count;
        averaged_pose.pose.position.z = lock_sum_z_ / sample_count;

        const double mean_yaw = std::atan2(lock_sum_sin_yaw_, lock_sum_cos_yaw_);
        tf2::Quaternion orientation;
        orientation.setEuler(0.0, 0.0, mean_yaw);
        averaged_pose.pose.orientation = tf2::toMsg(orientation);

        detected_dock_pose_ = averaged_pose;
        detected_dock_pose_prev_ = averaged_pose;
        use_external_detection_pose_ = true;
        is_locked_ = true;

        RCLCPP_INFO(
          node_->get_logger(),
          "Dock pose locked in after %d frames (averaged), frame=%s, x=%.3f, y=%.3f, yaw(rad)=%.3f",
          lock_counter_,
          detected_dock_pose_.header.frame_id.c_str(),
          detected_dock_pose_.pose.position.x,
          detected_dock_pose_.pose.position.y,
          tf2::getYaw(detected_dock_pose_.pose.orientation));
      }
  });

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

  reset_lock_state_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "/reset_dock_lock_state",
    std::bind(
      &SimpleChargingDock::resetDockLockStateService, this,
      std::placeholders::_1, std::placeholders::_2));

  // Subscribe to final_pose_nav topic
  final_pose_nav_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/final_pose", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      final_pose_nav_ = *msg;
  });

  // Subscribe to dock_side topic
  dock_side_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
    "/robot/dock_side", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) {
      cam_side_ = msg->data;
      if ( cam_side_ < 0 || cam_side_ > 3 )
        RCLCPP_WARN(node_->get_logger(), "/robot/dock_side:%d is not in valid range: 0-3", cam_side_);
  });
}


void SimpleChargingDock::resetLockState()
{
  lock_counter_ = 0;
  is_locked_ = false;
  use_external_detection_pose_ = false;
  lock_sum_x_ = 0.0;
  lock_sum_y_ = 0.0;
  lock_sum_z_ = 0.0;
  lock_sum_sin_yaw_ = 0.0;
  lock_sum_cos_yaw_ = 0.0;
  lock_frame_id_.clear();
  lock_latest_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}


geometry_msgs::msg::PoseStamped SimpleChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame, const std::string & dock_type)
{
  // reset_flag_ = false;
  // reset_timer_flag_ = false;
  resetLockState();

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
  // ** If using not detection, set the dock pose to the static fixed-frame version
  if (!use_external_detection_pose_) {
    if(detected_dock_pose_prev_.header.frame_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "No frame for detected dock pose");
    }
      // RCLCPP_WARN(node_->get_logger(), "No f");
    dock_pose_pub_->publish(detected_dock_pose_prev_);
    dock_pose_ = detected_dock_pose_prev_;
    return true;
  }

  // If using detections, get current detections, transform to frame, and apply offsets
  geometry_msgs::msg::PoseStamped detected = detected_dock_pose_;

  RCLCPP_INFO(node_->get_logger(), "raw Dock pose: frame=%s, pos=(%.3f, %.3f, %.3f), yaw=%.3f",
    detected.header.frame_id.c_str(),
    detected.pose.position.x,
    detected.pose.position.y,
    detected.pose.position.z,
    tf2::getYaw(detected.pose.orientation));

  // ** Validate that external pose is new enough
  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (node_->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(node_->get_logger(), "Lost detection or did not detect: timeout exceeded");
    use_external_detection_pose_ = false; // experimental
    dock_pose_pub_->publish(detected_dock_pose_prev_);
    dock_pose_ = detected_dock_pose_prev_;
    return true;
  }

  // Apply z-offset to move dock_pose away from detected pose before transform
  // Use stored dock_offset_z_ value from original goal
  if ( cam_side_ == 0 ) { // dock toward +y
    detected.pose.position.y -= fabs(dock_offset_z_);
  }
  else if ( cam_side_ == 1 ) { // +x
    detected.pose.position.x -= fabs(dock_offset_z_);
  }
  else if ( cam_side_ == 2 ) { // -y
    detected.pose.position.y += fabs(dock_offset_z_);
  }
  else if ( cam_side_ == 3 ) { // -x
    detected.pose.position.x += fabs(dock_offset_z_);
  }

  // Transform detected pose into fixed frame. Note that the argument pose
  // is the output of detection, but also acts as the initial estimate
  // and contains the frame_id of docking
  if (detected.header.frame_id != pose.header.frame_id) {
    try {
      if (!tf2_buffer_->canTransform(
          pose.header.frame_id, detected.header.frame_id,
          detected.header.stamp, rclcpp::Duration::from_seconds(0.2)))
      {
        RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
        return false;
      }
      tf2_buffer_->transform(detected, detected, pose.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
      return false;
    }
  }

  // Filter the detected pose
  detected = filter_->update(detected);
  filtered_dock_pose_pub_->publish(detected);

  // Construct dock_pose_ header and position

  dock_pose_.header = detected.header;
  dock_pose_.pose.position = detected.pose.position;

  // Process orientation unless flag is set to ignore it
  if (!ignore_detected_orientation_) {
    // Rotate the just the orientation, then remove roll/pitch
    geometry_msgs::msg::PoseStamped just_orientation;
    just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.rotation = detected.pose.orientation;
    tf2::doTransform(just_orientation, just_orientation, transform);

    tf2::Quaternion orientation;
    orientation.setEuler(0.0, 0.0, tf2::getYaw(just_orientation.pose.orientation));
    dock_pose_.pose.orientation = tf2::toMsg(orientation);
  } else {
    // Keep the original orientation from the initial pose estimate
    dock_pose_.pose.orientation = pose.pose.orientation;
  }



  // Publish & return dock pose for debugging purposes
  RCLCPP_INFO(node_->get_logger(), "transformed Dock pose: frame=%s, pos=(%.3f, %.3f, %.3f), yaw=%.3f",
    dock_pose_.header.frame_id.c_str(),
    dock_pose_.pose.position.x,
    dock_pose_.pose.position.y,
    dock_pose_.pose.position.z,
    tf2::getYaw(dock_pose_.pose.orientation));
  dock_pose_pub_->publish(dock_pose_);
  pose = dock_pose_;
  use_external_detection_pose_ = false;
  detected_dock_pose_prev_ = dock_pose_;  // Update with processed pose
  return true;
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

void SimpleChargingDock::resetDockLockStateService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  resetLockState();
  response->success = true;
  response->message = "SimpleChargingDock lock state reset";
  RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
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
