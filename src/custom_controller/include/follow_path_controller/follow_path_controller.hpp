#ifndef FOLLOW_PATH_CONTROLLER__FOLLOW_PATH_CONTROLLER_HPP_
#define FOLLOW_PATH_CONTROLLER__FOLLOW_PATH_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace follow_path_controller
{

struct RobotState
{
    double x_, y_, theta_;

    RobotState(double x = 0.0, double y = 0.0, double theta = 0.0) : x_(x), y_(y), theta_(theta) {}

    double distanceTo(const RobotState & p) const
    {
        return std::sqrt(std::pow(x_ - p.x_, 2.0) + std::pow(y_ - p.y_, 2.0));
    }
};

class FollowPathController : public nav2_core::Controller
{
public:
    FollowPathController() = default;
    ~FollowPathController() override = default;

    // lifecycle
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    //receive global plan
    void setPlan(const nav_msgs::msg::Path & path) override;

    // control center
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    void posetoRobotState(const geometry_msgs::msg::Pose & pose, RobotState & state);

    void pathToVector(const nav_msgs::msg::Path & path, std::vector<RobotState> & vector_path);

    RobotState globalTOlocal(const RobotState & cur_pose, const RobotState & goal);

    int getIndex(const RobotState & cur_pose, const std::vector<RobotState> & path, double radius);
    RobotState getLookAheadPoint(
        const RobotState & cur_pose,
        const std::vector<RobotState> & path,
        double look_ahead_distance);

    bool shouldDelaySpin();
    double getGoalAngle(double cur_angle, double goal_angle);

    // costmap
    bool worldToMapIndex(double wx, double wy, int & idx) const;
    bool checkObstacle(int current_index, int check_index);


    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string plugin_name_;
    rclcpp::Logger logger_{rclcpp::get_logger("follow_path_controller")};
    rclcpp::Clock::SharedPtr clock_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    nav_msgs::msg::Path global_plan_;
    std::vector<RobotState> vector_global_path_;

    RobotState cur_pose_;
    RobotState cur_goal_pose_;
    double final_goal_angle_;

    // rival
    nav_msgs::msg::Odometry rival_pose_;
    double rival_distance_;
    bool enable_rival_;
    double rival_slowdown_radius_;
    double rival_decay_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rival_pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_function_sub_;

    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr check_goal_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr rival_distance_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr goal_reach_pub_;

    // costmap
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    int costmap_tolerance_;

    // parameters
    double max_linear_vel_;
    double max_angular_vel_;
    double linear_kp_;
    double angular_kp_;
    double look_ahead_distance_;
    double speed_decade_;
    bool keep_planning_;
    double spin_delay_threshold_;
    double yaw_goal_tolerance_;
    double base_max_linear_vel_;

    double last_vel_x_;
    double last_vel_y_;
    bool update_plan_;
    std::string controller_function_{"None"};
};

}

#endif