// Copyright (c) 2026
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

#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;

class CameraOffsetPublisher : public rclcpp::Node
{
public:
  CameraOffsetPublisher()
  : Node("camera_offset_publisher")
  {
    // Declare parameters
    this->declare_parameter("goal_x", 1.5);
    this->declare_parameter("goal_y", 1.0);
    this->declare_parameter("publish_rate", 100.0);  // Hz
    this->declare_parameter("dock_side", 0);  // Default dock side is 0
    
    // Get parameters
    this->get_parameter("goal_x", goal_x_);
    this->get_parameter("goal_y", goal_y_);
    double publish_rate;
    this->get_parameter("publish_rate", publish_rate);
    this->get_parameter("dock_side", dock_side_);
    
    // Initialize current robot pose
    current_robot_x_ = 0.0;
    current_robot_y_ = 0.0;
    current_robot_yaw_ = 0.0;
    pose_received_ = false;
    
    // Create publisher for detected dock pose
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detected_dock_pose", 
      rclcpp::QoS(10).durability_volatile()
    );
    
    // Create publisher for absolute velocity
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/absolute_velocity",
      rclcpp::QoS(10)
    );
    
    // Create publisher for dock side
    dock_side_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
      "/robot/dock_side",
      rclcpp::QoS(10).reliable().transient_local()
    );
    
    // Subscribe to robot pose
    pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/final_pose",
      rclcpp::QoS(10),
      std::bind(&CameraOffsetPublisher::pose_callback, this, std::placeholders::_1)
    );
    
    // Subscribe to cmd_vel
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::QoS(10),
      std::bind(&CameraOffsetPublisher::cmd_vel_callback, this, std::placeholders::_1)
    );
    
    // Create timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CameraOffsetPublisher::timer_callback, this)
    );
    
    // Publish dock_side value
    auto dock_side_msg = std_msgs::msg::Int16();
    dock_side_msg.data = dock_side_;
    dock_side_publisher_->publish(dock_side_msg);
    
    RCLCPP_INFO(this->get_logger(), "Camera Offset Publisher started");
    RCLCPP_INFO(this->get_logger(), "Goal position: (%.3f, %.3f)", goal_x_, goal_y_);
    RCLCPP_INFO(this->get_logger(), "Publishing at %.1f Hz", publish_rate);
    RCLCPP_INFO(this->get_logger(), "Dock side: %d", dock_side_);
  }

private:
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_robot_x_ = msg->pose.pose.position.x;
    current_robot_y_ = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    auto q = msg->pose.pose.orientation;
    current_robot_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
    
    pose_received_ = true;
  }
  
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Calculate absolute velocity (magnitude of linear velocity vector)
    double abs_velocity = std::sqrt(
      std::pow(msg->linear.x, 2) + 
      std::pow(msg->linear.y, 2) + 
      std::pow(msg->linear.z, 2)
    );
    
    // Publish absolute velocity
    auto velocity_msg = std_msgs::msg::Float64();
    velocity_msg.data = abs_velocity;
    velocity_publisher_->publish(velocity_msg);
  }

  void timer_callback()
  {
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.frame_id = "base_footprint";  // Dock position relative to robot
    message.header.stamp = this->now();
    
    if (!pose_received_) {
      // No pose received yet, don't publish anything
      return;
    } else {
      // Calculate offset in global frame
      double dx_global = goal_x_ - current_robot_x_;
      double dy_global = goal_y_ - current_robot_y_;
      
      // Transform to robot's local frame (base_footprint)
      // Rotate by -current_robot_yaw to get offset in robot frame
      double cos_yaw = std::cos(current_robot_yaw_);
      double sin_yaw = std::sin(current_robot_yaw_);
      message.pose.position.x = dx_global * cos_yaw + dy_global * sin_yaw;
      message.pose.position.y = -dx_global * sin_yaw + dy_global * cos_yaw;
      message.pose.position.z = 0.0;
      
      // Orientation remains the same (dock orientation in base_footprint)
      message.pose.orientation.w = 0.71;
      message.pose.orientation.x = 0.0;
      message.pose.orientation.y = 0.0;
      message.pose.orientation.z = 0.704;
    }

    // Publish dock_side value
    auto dock_side_msg = std_msgs::msg::Int16();
    dock_side_msg.data = dock_side_;
    dock_side_publisher_->publish(dock_side_msg);
    
    publisher_->publish(message);
    
    // Log every 100 messages (reduce spam)
      RCLCPP_INFO(
        this->get_logger(), 
        "Robot: (%.3f, %.3f) | Goal: (%.3f, %.3f) | Dock offset: (%.3f, %.3f) | Distance: %.3f", 
        current_robot_x_, current_robot_y_,
        goal_x_, goal_y_,
        message.pose.position.x, message.pose.position.y,
        std::sqrt(std::pow(goal_x_ - current_robot_x_, 2) + std::pow(goal_y_ - current_robot_y_, 2))
      );
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr dock_side_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  double goal_x_;
  double goal_y_;
  double current_robot_x_;
  double current_robot_y_;
  double current_robot_yaw_;
  bool pose_received_;
  int dock_side_;
  size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraOffsetPublisher>());
  rclcpp::shutdown();
  return 0;
}
