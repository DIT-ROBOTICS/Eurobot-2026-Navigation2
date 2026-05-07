#include "nav2_behaviors/plugins/shrink.hpp"

namespace nav2_behaviors
{
    Shrink::Shrink() : TimedBehavior<ShrinkAction>(){
    }

    Shrink::~Shrink(){
    }

    void Shrink::onConfigure()
    {
        times = 0;
        costmap_received.store(false);
        goal_received.store(false);
        pose_received.store(false);
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error("Failed to lock node");
        }
            
        radius_param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            node, 
            "/global_costmap/global_costmap"
        );

        sub_costmap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 
            rclcpp::QoS(10), 
            std::bind(&Shrink::costmapCallback, this, std::placeholders::_1)
        );

        goal_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 
            rclcpp::SystemDefaultsQoS(), 
            std::bind(&Shrink::goalPoseCallback, this, std::placeholders::_1)
        );
        
        shrinkBack = false;
        
        // Use the behavior name to get the namespaced parameters
        nav2_util::declare_parameter_if_not_declared(
            node, "shrink.costmap_tolerance", rclcpp::ParameterValue(50));
        nav2_util::declare_parameter_if_not_declared(
            node, "shrink.timer_duration", rclcpp::ParameterValue(20));
        nav2_util::declare_parameter_if_not_declared(
            node, "shrink.shrink_delay_cycles", rclcpp::ParameterValue(3));

        // Get the parameter values using the proper namespace
        node->get_parameter("shrink.costmap_tolerance", costmap_tolerance);
        node->get_parameter("shrink.timer_duration", timer_duration);
        node->get_parameter("shrink.shrink_delay_cycles", shrink_delay_cycles);
    
        shrinkCheck_srv = node->create_service<std_srvs::srv::SetBool>(
            "/shrink/doneShrink",
            std::bind(&Shrink::handleShrinkCheck, this, std::placeholders::_1, std::placeholders::_2)
        );

        setMode_rival_client = node->create_client<std_srvs::srv::SetBool>(
            "/rival_layer/set_mode", 
            rmw_qos_profile_services_default
        );

        setMode_object_client = node->create_client<std_srvs::srv::SetBool>(
            "/object_layer/set_mode", 
            rmw_qos_profile_services_default
        );

        setMode_inflation_client = node->create_client<std_srvs::srv::SetBool>(
            "/inflation_layer/set_mode", 
            rmw_qos_profile_services_default
        );

    }

    void Shrink::onCleanup()
    {
        auto node = node_.lock();
        if (!node) {
            return;
        }
        try {
            setToOriginal();
        } catch (const std::exception & e) {
            RCLCPP_WARN(logger_, "Exception while cleaning up Shrink: %s", e.what());
        }
    }

    Status Shrink::onRun(const std::shared_ptr<const ShrinkAction::Goal> command){
        times = 0;
        shrinkBack = false;
        if (!command) {
            RCLCPP_ERROR(logger_, "Shrink goal is null");
            pose_received.store(false);
            return Status::FAILED;
        }
        unused_shrink = command->shrink_to;

        // Get the current robot pose (already done)
        geometry_msgs::msg::PoseStamped current_pose;
        const bool got_pose = nav2_util::getCurrentPose(
            current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (got_pose) {
                robotPose = current_pose;
            }
        }
        pose_received.store(got_pose);

        if (!got_pose) {
            RCLCPP_WARN(logger_, "Failed to get current robot pose");
            return Status::FAILED;
        }

        return Status::SUCCEEDED;
    }

    Status Shrink::onCycleUpdate(){
        times++;
        if(times == shrink_delay_cycles){
            setToShrink();
        }

        geometry_msgs::msg::PoseStamped current_pose;
        const bool got_pose = nav2_util::getCurrentPose(
            current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (got_pose) {
                robotPose = current_pose;
            }
        }
        pose_received.store(got_pose);

        if (!pose_received.load() || !costmap_received.load() || !goal_received.load()) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "Shrink waiting for pose/costmap/goal (pose=%s costmap=%s goal=%s)",
                pose_received.load() ? "true" : "false",
                costmap_received.load() ? "true" : "false",
                goal_received.load() ? "true" : "false");
            return Status::RUNNING;
        }

        if(noCostInMiddle() && noCostAtGoal() && times > timer_duration){
            times = 0;
            shrinkBack = true;
            RCLCPP_INFO(logger_, "\033[1;32mShrink SUCCESSED\033[0m");  // Bold green
            return Status::SUCCEEDED;
        }
        else if(times > timer_duration){
            times = 0;
            shrinkBack = true;
            RCLCPP_INFO(logger_, "\033[1;31mShrink FAILED\033[0m");  // Bold red
            return Status::FAILED;
        }
        else return Status::RUNNING;
    }

    void Shrink::handleShrinkCheck(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if(request->data){
            response->success = true;
            response->message = "getting message from the service";
            setToOriginal();
        }
    }


    void Shrink::setToOriginal(){
        RCLCPP_INFO(logger_, "set the inflation radius to original");
        changeInflationLayer(false);
        changeRivalLayer(false);
        changeObjectLayer(false);
    }

    void Shrink::setToShrink(){
        RCLCPP_INFO(logger_, "shrink the inflation radius");
        changeInflationLayer(true);
        changeRivalLayer(true);
        changeObjectLayer(true);
    }

    void Shrink::worldToMap(double wx, double wy, int & mx, int & my){
        mx = static_cast<int>((wx - costmap.info.origin.position.x) / costmap.info.resolution);
        my = static_cast<int>((wy - costmap.info.origin.position.y) / costmap.info.resolution);
    }

    double Shrink::getOneGridCost(double x, double y){
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        if (!costmap_received.load()) {
            return -1.0;
        }
        if (costmap.info.resolution <= 0.0 || costmap.info.width == 0 || costmap.info.height == 0) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Invalid costmap metadata");
            return -1.0;
        }
        const size_t expected_size = static_cast<size_t>(costmap.info.width) * costmap.info.height;
        if (costmap.data.size() < expected_size) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Costmap data size is smaller than expected");
            return -1.0;
        }
        int map_x, map_y;
        worldToMap(x, y, map_x, map_y);
        if (map_x < 0 || map_y < 0 ||
            map_x >= static_cast<int>(costmap.info.width) ||
            map_y >= static_cast<int>(costmap.info.height)) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Requested point outside costmap bounds");
            return -1.0;
        }
        return costmap.data[map_y * costmap.info.width + map_x];
    }

    void Shrink::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap = msg;
        costmap_received.store(true);
    }

    void Shrink::goalPoseCallback(const geometry_msgs::msg::PoseStamped& msg){
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goalPose = msg;
        goal_received.store(true);
    }

    void Shrink::changeInflationLayer(bool doShrink) {
        if (!setMode_inflation_client) {
            RCLCPP_WARN(logger_, "Inflation layer client not initialized, skipping");
            return;
        }
        // Wait for service to be ready with a timeout
        if (!setMode_inflation_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(logger_, "Service is not ready for inflation layer, skipping");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;

        auto logger = logger_;
        setMode_inflation_client->async_send_request(
            request,
            [logger, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger, "Inflation layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger, "Failed to set inflation layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger, "Exception in inflation layer callback: %s", e.what());
                }
            });
    }

    void Shrink::changeRivalLayer(bool doShrink) {
        if (!setMode_rival_client) {
            RCLCPP_WARN(logger_, "Rival layer client not initialized, skipping");
            return;
        }
        // Wait for service to be ready with a timeout
        if (!setMode_rival_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(logger_, "Service is not ready for rival layer, skipping");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;
        
        auto logger = logger_;
        setMode_rival_client->async_send_request(
            request,
            [logger, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger, "Rival layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger, "Failed to set rival layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger, "Exception in rival layer callback: %s", e.what());
                }
            });        
    }

    void Shrink::changeObjectLayer(bool doShrink) {
        if (!setMode_object_client) {
            RCLCPP_WARN(logger_, "Object layer client not initialized, skipping");
            return;
        }
        // Wait for service to be ready with a timeout
        if (!setMode_object_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(logger_, "Service is not ready for object layer, skipping");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;
        
        // Use a callback-based approach instead of spin_until_future_complete
        auto logger = logger_;
        setMode_object_client->async_send_request(
            request,
            [logger, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger, "Object layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger, "Failed to set object layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger, "Exception in object layer callback: %s", e.what());
                }
            });        
    }

    bool Shrink::noCostInMiddle(){
        geometry_msgs::msg::PoseStamped pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose = robotPose;
        }
        const double cost = getOneGridCost(pose.pose.position.x, pose.pose.position.y);
        if (cost < 0) {
            return false;
        }
        return cost <= costmap_tolerance;
    }

    bool Shrink::noCostAtGoal(){
        geometry_msgs::msg::PoseStamped goal;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal = goalPose;
        }
        const double cost = getOneGridCost(goal.pose.position.x, goal.pose.position.y);
        if (cost < 0) {
            return false;
        }
        return cost <= costmap_tolerance;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Shrink, nav2_core::Behavior)