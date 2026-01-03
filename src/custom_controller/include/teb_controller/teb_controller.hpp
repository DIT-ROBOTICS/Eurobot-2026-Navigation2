#ifndef TEB_CONTROLLER__TEB_CONTROLLER_HPP_
#define TEB_CONTROLLER__TEB_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

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

    // band build + optimize
    void initTimedElasticBand(const nav_msgs::msg::Path & plan);
    void optimizeBandOnce(const nav2_costmap_2d::Costmap2D & cm);

    // cost terms
    Eigen::Vector2d obstacleRepulsion(
        const nav2_costmap_2d::Costmap2D & cm, double x, double y) const;

    // tracking helpers
    bool findClosestIndex(const geometry_msgs::msg::PoseStamped & pose, size_t & out_idx) const;
    bool sampleLookaheadTargetArc(
        size_t start_idx, double lookahead,
        double & tx, double & ty) const;

    void publishTebPath();

private:
    // ros
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("TebController")};
    rclcpp::Clock::SharedPtr clock_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::string name_;

    std::mutex mtx_;

  nav_msgs::msg::Path global_plan_;
  std::vector<TebState> teb_band_;
  bool has_plan_{false};

    // pubs
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr teb_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    // parameters (band)
    double dt_ref_{0.1};
    double resample_ds_{0.05};
    int iterations_{2};

    // obstacle
    double min_obstacle_dist_{0.25};
    double w_smooth_{1.0};
    double w_obst_{2.0};
    double step_size_{0.05};

    // tracking (holonomic)
    double lookahead_dist_{0.25};
    double k_xy_{2.5};
    double k_w_{4.0};

    // goal behavior
    double goal_xy_stop_dist_{0.02};
    double goal_heading_switch_dist_{0.20};

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
    double max_cost_threshold_{150.0};     
    bool treat_no_info_as_obstacle_{false}; 
    int cost_check_stride_{1};            

    double stop_v_eps_{0.05};             
    unsigned char maxCostOnBand(const nav2_costmap_2d::Costmap2D & cm) const;
};

}  // namespace teb_controller

#endif  // TEB_CONTROLLER__TEB_CONTROLLER_HPP_