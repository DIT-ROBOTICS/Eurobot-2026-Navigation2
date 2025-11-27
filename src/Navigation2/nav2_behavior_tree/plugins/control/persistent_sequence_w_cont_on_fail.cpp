// Copyright (c) 2025 Enjoy Robotics
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

// Modifications:
//   - Copyright (c) 2025 DIT-ROBOTICS
//   - Renamed include to match project filename and added provenance note.
//   - Based on code from the Navigation2 project (ros-planning/navigation2).
// SPDX-License-Identifier: Apache-2.0

#include "nav2_behavior_tree/plugins/control/persistent_sequence_w_cont_on_fail.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

PersistentSequenceWContOnFailNode::PersistentSequenceWContOnFailNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode(name, conf)
{
}

void PersistentSequenceWContOnFailNode::resetCurrentChild(int child_idx)
{
  if (child_idx >= 0 && child_idx < static_cast<int>(children_nodes_.size())) {
    // Use inherited haltChild method which halts if RUNNING and sets status to IDLE
    haltChild(child_idx);
    RCLCPP_DEBUG(
      rclcpp::get_logger("PersistentSequenceWContOnFail"),
      "Reset child %d to IDLE", child_idx);
  }
}

BT::NodeStatus PersistentSequenceWContOnFailNode::tick()
{
  const int children_count = children_nodes_.size();

  int current_child_idx;
  if (!getInput("current_child_idx", current_child_idx)) {
    throw BT::RuntimeError(
      "Missing required input [current_child_idx] in PersistentSequenceWContOnFail");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx < children_count) {
    TreeNode * current_child_node = children_nodes_[current_child_idx];
    const BT::NodeStatus child_status = current_child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        return child_status;

      case BT::NodeStatus::FAILURE:
        // On failure, reset current child and continue to next
        resetCurrentChild(current_child_idx);
        current_child_idx++;
        setOutput("current_child_idx", current_child_idx);
        continue;

      case BT::NodeStatus::SUCCESS:
        // On success, reset current child, increment index and return success
        resetCurrentChild(current_child_idx);
        current_child_idx++;
        setOutput("current_child_idx", current_child_idx);
        return child_status;

      case BT::NodeStatus::IDLE:
        throw std::runtime_error("A child node must never return IDLE");
    }
  }

  // All children completed - reset for next iteration
  if (current_child_idx >= children_count) {
    resetChildren();
    setOutput("current_child_idx", 0);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::PersistentSequenceWContOnFailNode>(
        name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::PersistentSequenceWContOnFailNode>(
    "PersistentSequenceWContOnFail", builder);
}