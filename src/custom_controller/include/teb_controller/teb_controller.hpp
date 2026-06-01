#ifndef TEB_CONTROLLER__TEB_CONTROLLER_HPP_
#define TEB_CONTROLLER__TEB_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <utility>
#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace teb_controller
{

struct TebState
{
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double dt{0.1};  // nominal dt
};

class TebController : public nav2_core::Controller
{
public:
    TebController() = default;
    ~TebController() override = default;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    struct RivalInfo
    {
        bool valid{false};
        double dx_world{0.0};
        double dy_world{0.0};
        double dx_body{0.0};
        double dy_body{0.0};
        double distance{std::numeric_limits<double>::infinity()};
    };

    enum class MotionMode
    {
        FollowPath,
        RivalEscape
    };

    // utils
    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    static double hypot2(double x, double y)
    {
        return std::hypot(x, y);
    }

    static double yawFromQuat(const geometry_msgs::msg::Quaternion & q);
    static double normAngle(double a);

    // band build
    void initTimedElasticBand(const nav_msgs::msg::Path & plan);
    void optimizeBandOnceGlobal();

    // global occupancy grid helpers
    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid & grid, double wx, double wy,
        unsigned int & mx, unsigned int & my) const;
    std::pair<double, double> obstacleRepulsionGlobal(double x, double y) const;
    unsigned char costAtGlobal(double x, double y) const;
    double minObstacleDistanceGlobal(double x, double y, double search_radius) const;
    double minObstacleDistanceOnBandGlobal(size_t start_idx, double arc_len, double search_radius) const;
    unsigned char maxCostOnBandGlobal() const;

    // tracking helpers
    bool findClosestIndex(const geometry_msgs::msg::PoseStamped & pose, size_t & out_idx) const;
    bool sampleLookaheadTargetArc(
        size_t start_idx, double lookahead,
        double & tx, double & ty) const;

    void publishTebPath();
    void publishGoalReached() const;
    bool sendEscapeGoal(const geometry_msgs::msg::PoseStamped & pose, const RivalInfo & rival);
    bool shouldTriggerReplan(bool raw_blocked, const rclcpp::Time & now);
    RivalInfo getRivalInfo(const geometry_msgs::msg::PoseStamped & pose) const;
    bool shouldEnterRivalEscape(const RivalInfo & rival, double cmd_vx, double cmd_vy) const;
    bool shouldExitRivalEscape(
        const geometry_msgs::msg::PoseStamped & pose,
        const RivalInfo & rival,
        bool blocked_and_close,
        bool pose_collision) const;
    bool buildRivalEscapeCommand(
        const geometry_msgs::msg::PoseStamped & pose,
        const RivalInfo & rival,
        double current_speed,
        geometry_msgs::msg::TwistStamped & cmd);
    void setStoppedCommand(geometry_msgs::msg::TwistStamped & cmd) const;
    void resetVelocityMemory(const rclcpp::Time & stamp);
    RivalInfo applyRivalSlowdownStage(
        const geometry_msgs::msg::PoseStamped & pose,
        double & vx,
        double & vy,
        double & w) const;
    bool handleRivalEscapeStage(
        const geometry_msgs::msg::PoseStamped & pose,
        const RivalInfo & rival,
        double cmd_vx,
        double cmd_vy,
        bool blocked_and_close,
        bool pose_collision,
        double current_speed,
        geometry_msgs::msg::TwistStamped & cmd);
    void applyRivalSlowdown(const RivalInfo & rival, double & vx, double & vy, double & w) const;
    bool findRivalEscapeTarget(
        const geometry_msgs::msg::PoseStamped & pose,
        const RivalInfo & rival,
        double & target_x,
        double & target_y) const;
    double distanceFromEscapeStart(const geometry_msgs::msg::PoseStamped & pose) const;
    void beginRivalEscape(const geometry_msgs::msg::PoseStamped & pose);
    void resetRivalEscapeState();
    void updateRivalStopDistance();

private:
    // ros
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("TebController")};
    rclcpp::Clock::SharedPtr clock_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::string name_;
    std::string external_rival_data_path_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rival_pose_sub_;

    std::mutex mtx_;

  nav_msgs::msg::Path global_plan_;
  std::vector<TebState> teb_band_;
  bool has_plan_{false};
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_global_costmap_;
  nav_msgs::msg::Odometry latest_rival_pose_;
  bool has_rival_pose_{false};

    // pubs
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr teb_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    // parameters (band)
    double dt_ref_{0.05};
    double resample_ds_{0.05};
    int iterations_{4};
    // obstacle
    double min_obstacle_dist_{0.25};
    double w_smooth_{0.0};
    double w_obst_{1.0};
    double step_size_{0.05};
    double slowdown_obstacle_dist_{0.7};
    double stop_obstacle_dist_{0.15};
    double obstacle_check_lookahead_{0.5};
    double obstacle_check_time_horizon_{2.0};
    double obstacle_cost_threshold_{150.0};
    double rival_slowdown_dist_{0.4};
    double rival_min_speed_scale_{0.5};
    double rival_close_distance_{0.5};
    double rival_stop_distance_{0.35};
    double rival_stop_distance_prev_{0.35};
    double rival_escape_speed_{0.2};
    double rival_escape_distance_{0.18};
    double rival_escape_arc_angle_deg_{90.0};

    // tracking (holonomic)
    double lookahead_dist_{0.25};
    double k_xy_{2.5};
    double k_w_{4.0};

    // goal behavior
    double goal_xy_stop_dist_{0.03};
    double goal_heading_switch_dist_{5.0};

    // limits
    double max_v_{1.1};
    double min_v_{0.0};
    double max_w_{12.0};
    double max_acc_v_{3.3};
    double max_acc_w_{120.0};

    // speed limit interface
    bool speed_limit_is_percentage_{false};
    double speed_limit_{0.0};  // 0 => disabled

    // state for accel limiting
    rclcpp::Time last_stamp_;
    double last_vx_{0.0};
    double last_vy_{0.0};
    double last_w_{0.0};

    // --- replan trigger params ---
    double max_cost_threshold_{95.0};
    bool treat_no_info_as_obstacle_{true};
    int cost_check_stride_{1};            

    double stop_v_eps_{0.05};             
    double blocked_stop_clearance_{0.3};
    double replan_min_blocked_time_{0.2};
    double replan_cooldown_{0.01};
    rclcpp::Time blocked_since_;
    rclcpp::Time last_replan_time_;

    MotionMode motion_mode_{MotionMode::FollowPath};
    geometry_msgs::msg::PoseStamped rival_escape_start_pose_;
    bool has_rival_escape_start_{false};
    bool rival_escape_pending_stop_{false};
    bool rival_escape_goal_requested_{false};
    bool escape_navigation_active_{false};
    rclcpp::Time last_rival_escape_goal_time_;
    int rival_escape_stall_cycles_{0};
    int rival_escape_attempt_count_{0};
};

}  // namespace teb_controller

#endif  // TEB_CONTROLLER__TEB_CONTROLLER_HPP_