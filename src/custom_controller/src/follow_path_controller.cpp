#include <algorithm>
#include <string>
#include <memory>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

#include "follow_path_controller/follow_path_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

#include "tf2/LinearMath/Matrix3x3.h"

using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;

PLUGINLIB_EXPORT_CLASS(follow_path_controller::FollowPathController, nav2_core::Controller)

namespace follow_path_controller
{

static inline double normalize_angle(double a)
{
    while (a > M_PI) { a -= 2.0 * M_PI; }
    while (a < -M_PI) { a += 2.0 * M_PI; }
    return a;
}

void FollowPathController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, 
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf, 
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = parent.lock();

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    plugin_name_ = name;

    logger_ = node->get_logger();
    clock_ = node->get_clock();

    update_plan_ = true;

    declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".linear_kp", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".angular_kp", rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".look_ahead_distance", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".costmap_tolerance", rclcpp::ParameterValue(60));
    declare_parameter_if_not_declared(node, plugin_name_ + ".speed_decade", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".keep_planning", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".spin_delay_threshold", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(node, plugin_name_ + ".base_max_linear_vel", rclcpp::ParameterValue(0.7));

    declare_parameter_if_not_declared(node, plugin_name_ + ".costmap_topic", rclcpp::ParameterValue(std::string("/global_costmap/costmap")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".enable_rival", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rival_topic", rclcpp::ParameterValue(std::string("/rival/final_pose")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rival_slowdown_radius", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rival_decay", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".controller_function_topic", rclcpp::ParameterValue(std::string("/controller_function")));

    node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(plugin_name_ + ".linear_kp", linear_kp_);
    node->get_parameter(plugin_name_ + ".angular_kp", angular_kp_);
    node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
    node->get_parameter(plugin_name_ + ".costmap_tolerance", costmap_tolerance_);
    node->get_parameter(plugin_name_ + ".speed_decade", speed_decade_);
    node->get_parameter(plugin_name_ + ".keep_planning", keep_planning_);
    node->get_parameter(plugin_name_ + ".spin_delay_threshold", spin_delay_threshold_);
    node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
    node->get_parameter(plugin_name_ + ".base_max_linear_vel", base_max_linear_vel_);

    std::string costmap_topic;
    node->get_parameter(plugin_name_ + ".costmap_topic", costmap_topic);

    node->get_parameter(plugin_name_ + ".enable_rival", enable_rival_);
    std::string rival_topic;
    node->get_parameter(plugin_name_ + ".rival_topic", rival_topic);
    node->get_parameter(plugin_name_ + ".rival_slowdown_radius", rival_slowdown_radius_);
    node->get_parameter(plugin_name_ + ".rival_decay", rival_decay_);

    std::string ctrl_func_topic;
    node->get_parameter(plugin_name_ + ".controller_function_topic", ctrl_func_topic);

    costmap_subscription_ =
        node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic,
            rclcpp::QoS(10),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latest_costmap_ = msg;
            }
        );

    if (enable_rival_) {
        rival_pose_subscription_ =
        node->create_subscription<nav_msgs::msg::Odometry>(
            rival_topic,
            rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                rival_pose_ = *msg;
                rival_distance_ = std::sqrt(
                std::pow(rival_pose_.pose.pose.position.x - cur_pose_.x_, 2.0) +
                std::pow(rival_pose_.pose.pose.position.y - cur_pose_.y_, 2.0));

                if (rival_distance_ > 4.0) {
                    rival_distance_ = 0.0;
                }

                if (rival_distance_pub_) {
                    std_msgs::msg::Float64 d;
                    d.data = rival_distance_;
                    rival_distance_pub_->publish(d);
                }
            }
        );
    }

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 5);
    check_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("check_goal", 5);
    rival_distance_pub_ = node->create_publisher<std_msgs::msg::Float64>("rival_distance", 5);
    goal_reach_pub_ = node->create_publisher<std_msgs::msg::Bool>("goal_reached", 5);

    controller_function_sub_ =
        node->create_subscription<std_msgs::msg::String>(
            ctrl_func_topic,
            rclcpp::QoS(10).reliable().transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
            controller_function_ = msg->data;
        });

    RCLCPP_INFO(logger_, "[%s] configured.", plugin_name_.c_str());
}

void FollowPathController::cleanup()
{
    RCLCPP_INFO(logger_, "[%s] Cleaning up controller", plugin_name_.c_str());
    global_path_pub_.reset();
    check_goal_pub_.reset();
    rival_distance_pub_.reset();
    goal_reach_pub_.reset();
}

void FollowPathController::activate()
{
    RCLCPP_INFO(logger_, "[%s] Activating controller", plugin_name_.c_str());
    global_path_pub_->on_activate();
    check_goal_pub_->on_activate();
    rival_distance_pub_->on_activate();
    goal_reach_pub_->on_activate();
}

void FollowPathController::deactivate()
{
    RCLCPP_INFO(logger_, "[%s] Deactivating controller", plugin_name_.c_str());
    global_path_pub_->on_deactivate();
    check_goal_pub_->on_deactivate();
    rival_distance_pub_->on_deactivate();
    goal_reach_pub_->on_deactivate();
}


void FollowPathController::setPlan(const nav_msgs::msg::Path & path)
{
    if (!path.poses.empty()) {
        const auto & goal_pose = path.poses.back().pose;
        double new_goal_yaw = tf2::getYaw(goal_pose.orientation);

        // 若 goal 沒變且 keep_planning_ = false，就不更新
        if (!global_plan_.poses.empty() &&
            cur_goal_pose_.x_ == goal_pose.position.x &&
            cur_goal_pose_.y_ == goal_pose.position.y &&
            cur_goal_pose_.theta_ == new_goal_yaw &&
            !update_plan_) { return; }

        cur_goal_pose_.x_ = goal_pose.position.x;
        cur_goal_pose_.y_ = goal_pose.position.y;
        cur_goal_pose_.theta_ = new_goal_yaw;
        final_goal_angle_ = new_goal_yaw;
    }

    global_plan_ = path;

    auto msg = std::make_unique<nav_msgs::msg::Path>(global_plan_);
    global_plan_.header = path.header;
    global_path_pub_->publish(std::move(msg));

    update_plan_ = keep_planning_;
}

void FollowPathController::posetoRobotState(
    const geometry_msgs::msg::Pose & pose,
    RobotState & state)
{
    state.x_ = pose.position.x;
    state.y_ = pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    state.theta_ = yaw;
}

void FollowPathController::pathToVector(
    const nav_msgs::msg::Path & path,
    std::vector<RobotState> & vector_path)
{
    vector_path.clear();
    vector_path.reserve(path.poses.size());

    for (const auto & ps : path.poses) {
        RobotState s;
        posetoRobotState(ps.pose, s);
        vector_path.push_back(s);
    }
}

RobotState FollowPathController::globalTOlocal(
    const RobotState & cur_pose,
    const RobotState & goal)
{
    RobotState local_goal;
    double dx = goal.x_ - cur_pose.x_;
    double dy = goal.y_ - cur_pose.y_;

    local_goal.x_ = dx * std::cos(cur_pose.theta_) + dy * std::sin(cur_pose.theta_);
    local_goal.y_ = -dx * std::sin(cur_pose.theta_) + dy * std::cos(cur_pose.theta_);
    local_goal.theta_ = normalize_angle(goal.theta_ - cur_pose.theta_);

    return local_goal;
}

int FollowPathController::getIndex(
    const RobotState & cur_pose,
    const std::vector<RobotState> & path,
    double radius)
{
    if (path.empty()) {
        return 0;
    }

    int next_index = 0;

    for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
        if (cur_pose.distanceTo(path[i]) <= radius) {
            next_index = i;
            break;
        }
    }

    if (next_index == 0) {
        next_index = static_cast<int>(path.size()) - 1;
    }

    else if (next_index < static_cast<int>(path.size()) - 1) {
        next_index += 1;
    }

    return next_index;
}

RobotState FollowPathController::getLookAheadPoint(
    const RobotState & cur_pose,
    const std::vector<RobotState> & path,
    double look_ahead_distance)
{
    if (path.empty()) {
        RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[%s] Path is empty in getLookAheadPoint", plugin_name_.c_str());
        return cur_pose;
    }

    int next_index = getIndex(cur_pose, path, look_ahead_distance);
    RobotState local_goal = path[next_index];


    double dist = cur_pose.distanceTo(path[next_index]);
    if (dist > 1e-6) {
        double ratio = look_ahead_distance / dist;
        local_goal.x_ = cur_pose.x_ + (path[next_index].x_ - cur_pose.x_) * ratio;
        local_goal.y_ = cur_pose.y_ + (path[next_index].y_ - cur_pose.y_) * ratio;
    }

    if (cur_pose.distanceTo(path.back()) < look_ahead_distance + 0.01) {
        local_goal = path.back();
    }

    else if (local_goal.distanceTo(path.back()) < 0.005) {
        local_goal = path.back();
    }

    return local_goal;
}

bool FollowPathController::shouldDelaySpin()
{
    if (controller_function_ != "DelaySpin" || vector_global_path_.empty()) {
        return false;
    }

    const auto & start = vector_global_path_.front();
    const auto & goal  = vector_global_path_.back();

    double dist_start = cur_pose_.distanceTo(start);
    double dist_goal  = cur_pose_.distanceTo(goal);

    if (dist_start < spin_delay_threshold_ && dist_goal > spin_delay_threshold_) {
        return true;
    }

    controller_function_ = "None";
    return false;
}

double FollowPathController::getGoalAngle(double cur_angle, double goal_angle)
{
    if (shouldDelaySpin()) { return 0.0; }

    double ang_diff = normalize_angle(goal_angle - cur_angle);

    if (std::abs(ang_diff) < yaw_goal_tolerance_) {
        return 0.0;
    }

    double w = angular_kp_ * ang_diff;
    return std::clamp(w, -max_angular_vel_, max_angular_vel_);
}

bool FollowPathController::worldToMapIndex(double wx, double wy, int & idx) const
{
    if (!latest_costmap_) { return false; }

    const auto & info = latest_costmap_->info;

    int mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    int my = static_cast<int>((wy - info.origin.position.y) / info.resolution);

    if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(info.width) ||
        my >= static_cast<int>(info.height)) {
        return false;
    }

    idx = my * static_cast<int>(info.width) + mx;

    if (idx < 0 || idx >= static_cast<int>(latest_costmap_->data.size())) {
        return false;
    }

    return true;
}

bool FollowPathController::checkObstacle(int current_index, int check_index)
{
    if (!latest_costmap_ || vector_global_path_.empty()) { return false; }

    int start = std::min(current_index, check_index);
    int end = std::max(current_index, check_index);

    start = std::max(start, 0);
    end = std::min(end, static_cast<int>(vector_global_path_.size()) - 1);

    if (start > end) { return false; }

    if (check_goal_pub_) {
        geometry_msgs::msg::PoseStamped marker;
        marker.header.frame_id = global_plan_.header.frame_id;
        marker.header.stamp = clock_->now();
        marker.pose.position.x = vector_global_path_[end].x_;
        marker.pose.position.y = vector_global_path_[end].y_;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, vector_global_path_[end].theta_);
        marker.pose.orientation = tf2::toMsg(q);
        check_goal_pub_->publish(marker);
    }

    for (int i = start; i <= end; ++i) {
        const auto & p = vector_global_path_[i];

        int idx;
        if (!worldToMapIndex(p.x_, p.y_, idx)) {
            continue;
        }

        if (latest_costmap_->data[idx] > costmap_tolerance_) {
            RCLCPP_DEBUG(
                logger_,
                "[%s] Obstacle at index=%d, cost=%d",
                plugin_name_.c_str(), idx, latest_costmap_->data[idx]);
            return true;
        }
    }

    return false;
}


geometry_msgs::msg::TwistStamped FollowPathController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if (global_plan_.poses.empty()) {
        return cmd_vel;
    }

    // transform pose to RobotState
    posetoRobotState(pose.pose, cur_pose_);
    pathToVector(global_plan_, vector_global_path_);

    if (vector_global_path_.empty()) {
        return cmd_vel;
    }

    // look-ahead global point
    RobotState lookahead_global = getLookAheadPoint(cur_pose_, vector_global_path_, look_ahead_distance_);

    // transform the lookahead to local, decide direction
    RobotState lookahead_local = globalTOlocal(cur_pose_, lookahead_global);
    double local_angle = std::atan2(lookahead_local.y_, lookahead_local.x_);

    // 3. 以「距離終點」決定速度大小（保留原本設計）
    const auto & goal_pose = global_plan_.poses.back().pose;
    double global_distance = std::sqrt(
        std::pow(goal_pose.position.x - cur_pose_.x_, 2.0) +
        std::pow(goal_pose.position.y - cur_pose_.y_, 2.0));

    double v_base = std::min(global_distance * linear_kp_, max_linear_vel_);
    double v = v_base;

    // 4. rival 減速（可選）
    if (enable_rival_ && rival_distance_ > 0.0 &&
        rival_distance_ < rival_slowdown_radius_)
    {
        v *= rival_decay_;
    }

    cmd_vel.twist.angular.z = getGoalAngle(cur_pose_.theta_, final_goal_angle_);
    cmd_vel.twist.linear.x = v * std::cos(local_angle);
    cmd_vel.twist.linear.y = v * std::sin(local_angle);

    double vel_mag = std::hypot(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);
    double check_distance = std::max(vel_mag * 1.0, look_ahead_distance_);
    int current_index = getIndex(cur_pose_, vector_global_path_, look_ahead_distance_);
    int check_index = getIndex(cur_pose_, vector_global_path_, check_distance);

    bool obstacle = checkObstacle(current_index, check_index);

    if (obstacle) {
        cmd_vel.twist.linear.x *= speed_decade_;
        cmd_vel.twist.linear.y *= speed_decade_;
        cmd_vel.twist.angular.z = 0.0;
        update_plan_ = true;

        if (std::abs(cmd_vel.twist.linear.x) < 0.1 &&
            std::abs(cmd_vel.twist.linear.y) < 0.1)
        {
            throw nav2_core::PlannerException("Obstacle detected");
        }
    }

    if (goal_checker &&
        goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity))
    {
        std_msgs::msg::Bool goal_reach;
        goal_reach.data = true;
        goal_reach_pub_->publish(goal_reach);
    }

    return cmd_vel;
}

void FollowPathController::setSpeedLimit(
    const double & speed_limit,
    const bool & percentage)
{
    if (speed_limit <= 0.0) {
        max_linear_vel_ = base_max_linear_vel_;
        return;
    }

    if (percentage) {
        max_linear_vel_ = base_max_linear_vel_ * speed_limit / 100.0;
    } else {
        max_linear_vel_ = speed_limit;
    }
}

}