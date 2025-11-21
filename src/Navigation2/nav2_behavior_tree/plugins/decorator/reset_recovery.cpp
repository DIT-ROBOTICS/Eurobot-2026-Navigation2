// Copyright (c) 2025 DIT-ROBOTICS
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

#include "nav2_behavior_tree/plugins/decorator/reset_recovery.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

ResetRecovery::ResetRecovery(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(xml_tag_name, conf),
  goal_reached_(false)
{
  // Get the ROS node from the blackboard
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Create subscription to goal_reached topic
  if (node_) {
    goal_reached_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "goal_reached",
      10,
      std::bind(&ResetRecovery::goalReachedCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "ResetRecovery: Subscribed to /goal_reached topic");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ResetRecovery"), "Failed to get ROS node from blackboard");
  }

  // Initialize current_child_idx to 0
  setOutput("current_child_idx", 0);
}

BT::NodeStatus ResetRecovery::tick()
{
  // Reset current_child_idx if goal was reached (thread-safe)
  if (goal_reached_) {
    setOutput("current_child_idx", 0);
    goal_reached_ = false;
  }

  // Execute child node
  return child_node_->executeTick();
}

void ResetRecovery::goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Set flag when goal reached - actual reset happens in tick()
  goal_reached_ = msg->data;

  if (goal_reached_) {
    RCLCPP_INFO(node_->get_logger(), "ResetRecovery: Goal reached flag set");
  }
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ResetRecovery>(
        name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::ResetRecovery>("ResetRecovery", builder);
}