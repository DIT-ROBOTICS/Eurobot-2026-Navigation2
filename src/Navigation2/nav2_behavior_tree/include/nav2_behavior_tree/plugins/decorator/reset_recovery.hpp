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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RESET_RECOVERY_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RESET_RECOVERY_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A decorator node that resets recovery state when goal is reached
 *
 * This node monitors the /goal_reached topic and resets the current_child_idx
 * blackboard variable to 0 when a goal is reached, allowing recovery behaviors
 * to restart from the beginning for new goals.
 */
class ResetRecovery : public BT::DecoratorNode
{
public:
  /**
   * @brief Construct a new Reset Recovery decorator node
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ResetRecovery(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Declare ports for this node
   * @return BT::PortsList List of ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<int>("current_child_idx", "Child index for persistent sequences")
    };
  }

private:
  /**
   * @brief The main override required by a BT decorator
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback for goal_reached topic subscription
   * @param msg Bool message indicating goal reached status
   */
  void goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
  bool goal_reached_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RESET_RECOVERY_HPP_