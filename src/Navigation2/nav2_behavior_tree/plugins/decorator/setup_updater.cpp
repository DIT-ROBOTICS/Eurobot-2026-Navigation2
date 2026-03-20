#include "nav2_behavior_tree/plugins/decorator/setup_updater.hpp"

namespace nav2_behavior_tree
{
    SetupUpdater::SetupUpdater(
        const std::string & name,
        const BT::NodeConfiguration & conf)
        : BT::DecoratorNode(name, conf)
    {
        // Retrieve node from the blackboard
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        isGoalUpdated = false;
        waiting_for_service_ = false;
        // Create client to shrink's service
        shrink_client_ = node_->create_client<std_srvs::srv::SetBool>(
            "/shrink/doneShrink",
            rmw_qos_profile_services_default);
    }

    void SetupUpdater::requestShrinkBack()
    {
        if (!shrink_client_->wait_for_service(std::chrono::milliseconds(0))) {
            RCLCPP_ERROR(node_->get_logger(), "Shrink service not available");
            waiting_for_service_ = false;
        } else {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;  // Instruct shrink to call setToOriginal()
            future_result_ = shrink_client_->async_send_request(request).share();
            waiting_for_service_ = true;
            RCLCPP_INFO(node_->get_logger(), "Async shrink request sent...");
        }
    }

    bool SetupUpdater::goalUpdated(){
        config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", incomming_goal_list);
        config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", incomming_goal);
        if (current_goal != incomming_goal || current_goal_list != incomming_goal_list) {
            current_goal = incomming_goal;
            current_goal_list = incomming_goal_list;
            RCLCPP_INFO(node_->get_logger(), "\033[1;34m Goal updated \033[0m");
            return true;
        }

        return false;
    }

    inline BT::NodeStatus SetupUpdater::tick()
    {
        isGoalUpdated = goalUpdated();
        setOutput("goalUpdated", isGoalUpdated);
        if(isGoalUpdated){
            requestShrinkBack();
        }

        if (waiting_for_service_) {
            auto status = future_result_.wait_for(std::chrono::milliseconds(0));
            if (status == std::future_status::ready) {
                auto response = future_result_.get();
                if (response && response->success) {
                    RCLCPP_INFO(node_->get_logger(), "Shrink request finished successfully");
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Shrink service call failed or timed out");
                }
                waiting_for_service_ = false;
            } else {
                return BT::NodeStatus::RUNNING;
            }
        }

        return child_node_->executeTick();
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetupUpdater>("SetupUpdater");
}