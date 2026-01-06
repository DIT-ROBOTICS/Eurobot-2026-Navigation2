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
//
// Modifications:
//   - Copyright (c) 2025 DIT-ROBOTICS
//   - Adapted filename, include-guard and comments to match project layout.
//   - Based on code from the Navigation2 project (ros-planning/navigation2).
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PERSISTENT_SEQUENCE_W_CONT_ON_FAIL_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PERSISTENT_SEQUENCE_W_CONT_ON_FAIL_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{

/**
 * @brief A persistent sequence node that continues to next child on failure
 *
 * The PersistentSequenceWContOnFailNode is similar to SequenceNode, but:
 * - Stores the index of the last running child in the blackboard (key: "current_child_idx")
 * - Does not reset the index when halted
 * - Continues to next child on FAILURE instead of resetting the sequence
 *
 * Behavior:
 * - If all children return SUCCESS, this node returns SUCCESS
 * - If a child returns RUNNING, this node returns RUNNING (same child ticked again)
 * - If a child returns FAILURE, continues to the next child
 */
class PersistentSequenceWContOnFailNode : public BT::ControlNode
{
public:
  /**
   * @brief Construct a new Persistent Sequence W Cont On Fail Node
   * @param name Name of the node
   * @param conf BT node configuration
   */
  PersistentSequenceWContOnFailNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  ~PersistentSequenceWContOnFailNode() override = default;

  /**
   * @brief Declare ports for this node
   * @return BT::PortsList List of ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<int>("current_child_idx", "Index of the current child")
    };
  }

private:
  /**
   * @brief The main override required by a BT control node
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Reset a specific child to IDLE state
   * @param child_idx Index of the child to reset
   */
  void resetCurrentChild(int child_idx);
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PERSISTENT_SEQUENCE_W_CONT_ON_FAIL_HPP_