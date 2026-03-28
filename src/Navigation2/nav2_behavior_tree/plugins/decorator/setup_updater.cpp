#include "nav2_behavior_tree/plugins/decorator/setup_updater.hpp"

namespace nav2_behavior_tree
{
    SetupUpdater::SetupUpdater(
        const std::string & name,
        const BT::NodeConfiguration & conf)
        : BT::DecoratorNode(name, conf),
          request_start_time_(0, 0, RCL_ROS_TIME)
    {
        // Retrieve node from the blackboard
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        isGoalUpdated = false;
        waiting_for_service_ = false;

        // Setup callback group and executor for independent processing
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        // Create client to shrink's service with the dedicated callback group
        shrink_client_ = node_->create_client<std_srvs::srv::SetBool>(
            "/shrink/doneShrink",
            rmw_qos_profile_services_default,
            callback_group_);
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
            request_start_time_ = node_->now();
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
        // Process callbacks for the client independently
        callback_group_executor_.spin_some();

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
                // Check for timeout
                if ((node_->now() - request_start_time_).seconds() > timeout_) {
                    RCLCPP_WARN(node_->get_logger(), "Shrink request timed out after %.1f seconds. Proceeding anyway.", timeout_);
                    waiting_for_service_ = false;
                } else {
                    // RCLCPP_DEBUG(node_->get_logger(), "SetupUpdater: Still waiting for shrink service...");
                    return BT::NodeStatus::RUNNING;
                }
            }
        }

        BT::NodeStatus child_status = child_node_->executeTick();
        
        // Log status for debugging
        if (child_status == BT::NodeStatus::RUNNING) {
            // RCLCPP_DEBUG(node_->get_logger(), "SetupUpdater returning RUNNING (child is running)");
        } else if (child_status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "\033[1;32mSetupUpdater returning SUCCESS\033[0m");
        } else if (child_status == BT::NodeStatus::FAILURE) {
            RCLCPP_INFO(node_->get_logger(), "\033[1;31mSetupUpdater returning FAILURE\033[0m");
        }

        return child_status;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetupUpdater>("SetupUpdater");
}