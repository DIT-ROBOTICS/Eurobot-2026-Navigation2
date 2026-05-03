#include "teb_controller/teb_controller.hpp"

#include <cmath>
#include <algorithm>

#include "yaml-cpp/yaml.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_core/exceptions.hpp"

// Logging disabled in this file by request.
#undef RCLCPP_INFO
#undef RCLCPP_INFO_THROTTLE
#undef RCLCPP_DEBUG_THROTTLE
#define RCLCPP_INFO(...)
#define RCLCPP_INFO_THROTTLE(...)
#define RCLCPP_DEBUG_THROTTLE(...)

namespace teb_controller
{

namespace
{

constexpr int kRivalEscapeStallCycleLimit = 5;
constexpr double kRivalStopDistanceMargin = 0.27;

}  // namespace

double TebController::yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
    tf2::Quaternion tq;
    tf2::fromMsg(q, tq);
    double r, p, y;
    tf2::Matrix3x3(tq).getRPY(r, p, y);
    return y;
}

double TebController::normAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void TebController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("TebController: parent node expired");
    }

    name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Declare with defaults
    node->declare_parameter(name_ + ".dt_ref", dt_ref_);
    node->declare_parameter(name_ + ".resample_ds", resample_ds_);
    node->declare_parameter(name_ + ".iterations", iterations_);
    node->declare_parameter(name_ + ".min_obstacle_dist", min_obstacle_dist_);
    node->declare_parameter(name_ + ".w_smooth", w_smooth_);
    node->declare_parameter(name_ + ".w_obst", w_obst_);
    node->declare_parameter(name_ + ".step_size", step_size_);
    node->declare_parameter(name_ + ".slowdown_obstacle_dist", slowdown_obstacle_dist_);
    node->declare_parameter(name_ + ".stop_obstacle_dist", stop_obstacle_dist_);
    node->declare_parameter(name_ + ".obstacle_check_lookahead", obstacle_check_lookahead_);
    node->declare_parameter(name_ + ".obstacle_check_time_horizon", obstacle_check_time_horizon_);
    node->declare_parameter(name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
    node->declare_parameter(name_ + ".rival_slowdown_dist", rival_slowdown_dist_);
    node->declare_parameter(name_ + ".rival_min_speed_scale", rival_min_speed_scale_);
    node->declare_parameter(name_ + ".rival_close_distance", rival_close_distance_);
    node->declare_parameter(name_ + ".rival_stop_distance", rival_stop_distance_);
    node->declare_parameter(name_ + ".external_rival_data_path", external_rival_data_path_);
    node->declare_parameter(name_ + ".rival_escape_speed", rival_escape_speed_);
    node->declare_parameter(name_ + ".rival_escape_distance", rival_escape_distance_);

    node->declare_parameter(name_ + ".lookahead_dist", lookahead_dist_);
    node->declare_parameter(name_ + ".k_xy", k_xy_);
    node->declare_parameter(name_ + ".k_w", k_w_);

    node->declare_parameter(name_ + ".goal_xy_stop_dist", goal_xy_stop_dist_);
    node->declare_parameter(name_ + ".goal_heading_switch_dist", goal_heading_switch_dist_);

    node->declare_parameter(name_ + ".max_v", max_v_);
    node->declare_parameter(name_ + ".min_v", min_v_);
    node->declare_parameter(name_ + ".max_w", max_w_);
    node->declare_parameter(name_ + ".max_acc_v", max_acc_v_);
    node->declare_parameter(name_ + ".max_acc_w", max_acc_w_);

    node->declare_parameter(name_ + ".max_cost_threshold", max_cost_threshold_);
    node->declare_parameter(name_ + ".treat_no_info_as_obstacle", treat_no_info_as_obstacle_);
    node->declare_parameter(name_ + ".cost_check_stride", cost_check_stride_);
    node->declare_parameter(name_ + ".stop_v_eps", stop_v_eps_);
    node->declare_parameter(name_ + ".blocked_stop_clearance", blocked_stop_clearance_);
    node->declare_parameter(name_ + ".replan_min_blocked_time", replan_min_blocked_time_);
    node->declare_parameter(name_ + ".replan_cooldown", replan_cooldown_);

    // Get
    node->get_parameter(name_ + ".dt_ref", dt_ref_);
    node->get_parameter(name_ + ".resample_ds", resample_ds_);
    node->get_parameter(name_ + ".iterations", iterations_);
    node->get_parameter(name_ + ".min_obstacle_dist", min_obstacle_dist_);
    node->get_parameter(name_ + ".w_smooth", w_smooth_);
    node->get_parameter(name_ + ".w_obst", w_obst_);
    node->get_parameter(name_ + ".step_size", step_size_);
    node->get_parameter(name_ + ".slowdown_obstacle_dist", slowdown_obstacle_dist_);
    node->get_parameter(name_ + ".stop_obstacle_dist", stop_obstacle_dist_);
    node->get_parameter(name_ + ".obstacle_check_lookahead", obstacle_check_lookahead_);
    node->get_parameter(name_ + ".obstacle_check_time_horizon", obstacle_check_time_horizon_);
    node->get_parameter(name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
    node->get_parameter(name_ + ".rival_slowdown_dist", rival_slowdown_dist_);
    node->get_parameter(name_ + ".rival_min_speed_scale", rival_min_speed_scale_);
    node->get_parameter(name_ + ".rival_close_distance", rival_close_distance_);
    node->get_parameter(name_ + ".rival_stop_distance", rival_stop_distance_);
    node->get_parameter(name_ + ".external_rival_data_path", external_rival_data_path_);
    node->get_parameter(name_ + ".rival_escape_speed", rival_escape_speed_);
    node->get_parameter(name_ + ".rival_escape_distance", rival_escape_distance_);

    node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(name_ + ".k_xy", k_xy_);
    node->get_parameter(name_ + ".k_w", k_w_);

    node->get_parameter(name_ + ".goal_xy_stop_dist", goal_xy_stop_dist_);
    node->get_parameter(name_ + ".goal_heading_switch_dist", goal_heading_switch_dist_);

    node->get_parameter(name_ + ".max_v", max_v_);
    node->get_parameter(name_ + ".min_v", min_v_);
    node->get_parameter(name_ + ".max_w", max_w_);
    node->get_parameter(name_ + ".max_acc_v", max_acc_v_);
    node->get_parameter(name_ + ".max_acc_w", max_acc_w_);

    node->get_parameter(name_ + ".max_cost_threshold", max_cost_threshold_);
    node->get_parameter(name_ + ".treat_no_info_as_obstacle", treat_no_info_as_obstacle_);
    node->get_parameter(name_ + ".cost_check_stride", cost_check_stride_);
    node->get_parameter(name_ + ".stop_v_eps", stop_v_eps_);
    node->get_parameter(name_ + ".blocked_stop_clearance", blocked_stop_clearance_);
    node->get_parameter(name_ + ".replan_min_blocked_time", replan_min_blocked_time_);
    node->get_parameter(name_ + ".replan_cooldown", replan_cooldown_);

    // Safety clamp
    dt_ref_ = std::max(0.01, dt_ref_);
    resample_ds_ = std::max(0.005, resample_ds_);
    iterations_ = std::max(0, iterations_);
    min_obstacle_dist_ = std::max(0.05, min_obstacle_dist_);
    step_size_ = clamp(step_size_, 0.001, 0.2);
    lookahead_dist_ = std::max(0.02, lookahead_dist_);
    max_v_ = std::max(0.01, max_v_);
    max_w_ = std::max(0.01, max_w_);
    min_v_ = std::max(0.0, min_v_);
    rival_slowdown_dist_ = std::max(0.0, rival_slowdown_dist_);
    rival_min_speed_scale_ = clamp(rival_min_speed_scale_, 0.0, 1.0);
    rival_close_distance_ = std::max(0.0, rival_close_distance_);
    rival_stop_distance_ = std::max(0.0, rival_stop_distance_);
    updateRivalStopDistance();
    rival_stop_distance_ = std::max(0.0, rival_stop_distance_);
    rival_stop_distance_prev_ = rival_stop_distance_;
    rival_escape_speed_ = std::max(0.01, rival_escape_speed_);
    rival_escape_distance_ = std::max(0.02, rival_escape_distance_);

    max_cost_threshold_ = clamp(max_cost_threshold_, 0.0, 255.0);
    cost_check_stride_ = std::max(1, cost_check_stride_);
    stop_v_eps_ = std::max(0.0, stop_v_eps_);
    blocked_stop_clearance_ = std::max(0.0, blocked_stop_clearance_);
    replan_min_blocked_time_ = std::max(0.0, replan_min_blocked_time_);
    replan_cooldown_ = std::max(0.0, replan_cooldown_);

    // Lifecycle publisher (RViz debug)
    teb_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/teb_path", rclcpp::SystemDefaultsQoS());
    // Mirror the custom_controller topic name for RViz
    global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 5);
    goal_reached_pub_ = node->create_publisher<std_msgs::msg::Bool>(
        "goal_reached",
        rclcpp::QoS(10).reliable().transient_local());
    navigate_to_pose_client_ =
        rclcpp_action::create_client<NavigateToPose>(node, "/navigate_to_pose");
    global_costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap",
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latest_global_costmap_ = msg;
        });
    rival_pose_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/rhino_pose",
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::scoped_lock lk(mtx_);
            latest_rival_pose_ = *msg;
            has_rival_pose_ = true;
        });
    RCLCPP_INFO(logger_, "[%s] configured", name_.c_str());
}

void TebController::cleanup()
{
    std::scoped_lock lk(mtx_);
    teb_band_.clear();
    global_plan_ = nav_msgs::msg::Path{};
    has_plan_ = false;
    teb_path_pub_.reset();
    global_plan_pub_.reset();
    goal_reached_pub_.reset();
    navigate_to_pose_client_.reset();
    rival_pose_sub_.reset();
    global_costmap_sub_.reset();
    has_rival_pose_ = false;
    latest_rival_pose_ = nav_msgs::msg::Odometry{};

    last_stamp_ = rclcpp::Time(0, 0, clock_ ? clock_->get_clock_type() : RCL_ROS_TIME);
    blocked_since_ = rclcpp::Time(0, 0, clock_ ? clock_->get_clock_type() : RCL_ROS_TIME);
    last_replan_time_ = rclcpp::Time(0, 0, clock_ ? clock_->get_clock_type() : RCL_ROS_TIME);
    last_vx_ = last_vy_ = last_w_ = 0.0;
    escape_navigation_active_ = false;
    resetRivalEscapeState();
}

void TebController::activate()
{
    if (teb_path_pub_) teb_path_pub_->on_activate();
    if (global_plan_pub_) global_plan_pub_->on_activate();
}

void TebController::deactivate()
{
    if (teb_path_pub_) teb_path_pub_->on_deactivate();
    if (global_plan_pub_) global_plan_pub_->on_deactivate();
}

void TebController::setPlan(const nav_msgs::msg::Path & path)
{
    std::scoped_lock lk(mtx_);
    global_plan_ = path;
    rival_escape_attempt_count_ = 0;
    resetRivalEscapeState();
    RCLCPP_INFO(logger_, "[%s] received global plan with %zu poses", name_.c_str(), path.poses.size());

    if (global_plan_pub_ && global_plan_pub_->is_activated()) {
        global_plan_pub_->publish(global_plan_);
    }

    initTimedElasticBand(global_plan_);
    has_plan_ = (teb_band_.size() >= 2);
    RCLCPP_INFO(logger_, "[%s] initialized teb band with %zu states (has_plan=%d)", name_.c_str(), teb_band_.size(), has_plan_);
}

void TebController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    speed_limit_ = speed_limit;
    speed_limit_is_percentage_ = percentage;
}

void TebController::initTimedElasticBand(const nav_msgs::msg::Path & plan)
{
    teb_band_.clear();
    if (plan.poses.size() < 2) return;

    // cumulative arc-length
    RCLCPP_INFO(logger_, "[TEB Controller]: Cumaltive arc-length started.");
    const auto & poses = plan.poses;
    std::vector<double> s(poses.size(), 0.0);
    for (size_t i = 1; i < poses.size(); ++i) {
        const auto & a = poses[i - 1].pose.position;
        const auto & b = poses[i].pose.position;
        s[i] = s[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
    }
    RCLCPP_INFO(logger_, "[TEB Controller]: Cumaltive arc-length finished.");

    const double total = s.back();
    if (total < 1e-6) return;

    const size_t N = static_cast<size_t>(std::max(2.0, std::ceil(total / resample_ds_) + 1.0));
    const double ds = total / (N - 1);

    size_t j = 0;
    teb_band_.reserve(N);

    RCLCPP_INFO(logger_, "[TEB Controller]: Resampling started.");
    for (size_t k = 0; k < N; ++k) {
        const double sk = ds * k;
        while (j + 1 < s.size() && s[j + 1] < sk) j++;

        double t = 0.0;
        if (j + 1 < s.size()) {
        const double denom = s[j + 1] - s[j];
        t = (denom > 1e-9) ? (sk - s[j]) / denom : 0.0;
        }

        const auto & p0 = poses[j].pose.position;
        const auto & p1 = poses[std::min(j + 1, poses.size() - 1)].pose.position;

        TebState st;
        st.x = (1.0 - t) * p0.x + t * p1.x;
        st.y = (1.0 - t) * p0.y + t * p1.y;
        st.dt = dt_ref_;
        teb_band_.push_back(st);
    }
    RCLCPP_INFO(logger_, "[TEB Controller]: Resampling finished. N = %zu, ds = %f", teb_band_.size(), ds);
    // theta from segment direction
    RCLCPP_INFO(logger_, "[TEB Controller]: Theta assignment started.");
    for (size_t i = 0; i + 1 < teb_band_.size(); ++i) {
        const double dx = teb_band_[i + 1].x - teb_band_[i].x;
        const double dy = teb_band_[i + 1].y - teb_band_[i].y;
        teb_band_[i].theta = std::atan2(dy, dx);
    }
    teb_band_.back().theta = teb_band_[teb_band_.size() - 2].theta;
    RCLCPP_INFO(logger_, "[TEB Controller]: Theta assignment finished.");
}

bool TebController::worldToMap(
    const nav_msgs::msg::OccupancyGrid & grid, double wx, double wy,
    unsigned int & mx, unsigned int & my) const
{
    const double res = grid.info.resolution;
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    if (res <= 0.0) return false;

    const double mx_d = (wx - origin_x) / res;
    const double my_d = (wy - origin_y) / res;
    if (mx_d < 0.0 || my_d < 0.0) return false;

    mx = static_cast<unsigned int>(mx_d);
    my = static_cast<unsigned int>(my_d);
    if (mx >= grid.info.width || my >= grid.info.height) return false;
    return true;
}

unsigned char TebController::costAtGlobal(double x, double y) const
{
    auto grid = latest_global_costmap_;
    if (!grid) return nav2_costmap_2d::NO_INFORMATION;

    unsigned int mx, my;
    if (!worldToMap(*grid, x, y, mx, my)) {
        return nav2_costmap_2d::NO_INFORMATION;
    }

    const int8_t raw = grid->data[my * grid->info.width + mx];
    if (raw < 0) {
        return treat_no_info_as_obstacle_ ? nav2_costmap_2d::LETHAL_OBSTACLE
                                          : nav2_costmap_2d::NO_INFORMATION;
    }
    return static_cast<unsigned char>(raw);
}

double TebController::minObstacleDistanceGlobal(double x, double y, double search_radius) const
{
    auto grid = latest_global_costmap_;
    if (!grid) return std::numeric_limits<double>::infinity();

    unsigned int mx_center, my_center;
    if (!worldToMap(*grid, x, y, mx_center, my_center)) {
        return std::numeric_limits<double>::infinity();
    }

    const double res = grid->info.resolution;
    const int r = std::max(1, (int)std::ceil(search_radius / res));
    double best = std::numeric_limits<double>::infinity();
    const unsigned int w = grid->info.width;
    const unsigned int h = grid->info.height;

    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000,"[TEB Controller]: Min obstacle distance calculation started in minObstacleDistanceGlobal.");
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            const int ix = (int)mx_center + dx;
            const int iy = (int)my_center + dy;
            if (ix < 0 || iy < 0) continue;
            if ((unsigned)ix >= w || (unsigned)iy >= h) continue;

            const int8_t raw = grid->data[iy * w + ix];
            unsigned char c = nav2_costmap_2d::NO_INFORMATION;
            if (raw < 0) {
                c = treat_no_info_as_obstacle_ ? nav2_costmap_2d::LETHAL_OBSTACLE
                                               : nav2_costmap_2d::NO_INFORMATION;
            } else {
                c = static_cast<unsigned char>(raw);
            }
            if (c < obstacle_cost_threshold_) continue;

            const double wx = grid->info.origin.position.x + (ix + 0.5) * res;
            const double wy = grid->info.origin.position.y + (iy + 0.5) * res;
            const double d = std::hypot(x - wx, y - wy);
            best = std::min(best, d);
        }
    }

    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "[TEB Controller]: Min obstacle distance calculation finished in minObstacleDistanceGlobal.");
    return best;
}

double TebController::minObstacleDistanceOnBandGlobal(
    size_t start_idx, double arc_len, double search_radius) const
{
    if (teb_band_.empty()) return std::numeric_limits<double>::infinity();
    if (start_idx >= teb_band_.size()) start_idx = teb_band_.size() - 1;

    double best = std::numeric_limits<double>::infinity();
    double acc = 0.0;

    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "[TEB Controller]: Min obstacle distance calculation started in minObstacleDistanceOnBandGlobal.");
    for (size_t i = start_idx; i + 1 < teb_band_.size(); ++i) {
        const auto & a = teb_band_[i];
        const auto & b = teb_band_[i + 1];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        best = std::min(best, minObstacleDistanceGlobal(a.x, a.y, search_radius));
        best = std::min(best, minObstacleDistanceGlobal(b.x, b.y, search_radius));

        if (seg > 1e-9) {
            acc += seg;
            if (acc >= arc_len) break;
        }
    }
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "[TEB Controller]: Min obstacle distance calculation finished in minObstacleDistanceOnBandGlobal.");

    return best;
}

unsigned char TebController::maxCostOnBandGlobal() const
{
    auto grid = latest_global_costmap_;
    if (!grid || teb_band_.empty()) return 0;

    unsigned char mc = 0;
    unsigned int mx, my;

    RCLCPP_INFO(logger_, "[TEB Controller]: Max cost on band calculation started in maxCostOnBandGlobal.");
    for (size_t i = 0; i < teb_band_.size(); i += (size_t)cost_check_stride_) {
        const double wx = teb_band_[i].x;
        const double wy = teb_band_[i].y;

        if (!worldToMap(*grid, wx, wy, mx, my)) {
            mc = std::max<unsigned char>(mc, nav2_costmap_2d::NO_INFORMATION);
            continue;
        }

        const int8_t raw = grid->data[my * grid->info.width + mx];
        unsigned char c = nav2_costmap_2d::NO_INFORMATION;
        if (raw < 0) {
            c = treat_no_info_as_obstacle_ ? nav2_costmap_2d::LETHAL_OBSTACLE
                                           : nav2_costmap_2d::NO_INFORMATION;
        } else {
            c = static_cast<unsigned char>(raw);
        }
        if (!treat_no_info_as_obstacle_ && c == nav2_costmap_2d::NO_INFORMATION) {
            continue;
        }
        mc = std::max(mc, c);
    }
    RCLCPP_INFO(logger_, "[TEB Controller]: Max cost on band calculation finished in maxCostOnBandGlobal.");

    return mc;
}

bool TebController::findClosestIndex(const geometry_msgs::msg::PoseStamped & pose, size_t & out_idx) const
{
    if (teb_band_.empty()) return false;

    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;

    double best = 1e100;
    size_t bi = 0;

    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "[TEB Controller]: Finding closest index started in findClosestIndex.");
    for (size_t i = 0; i < teb_band_.size(); ++i) {
        const double dx = teb_band_[i].x - px;
        const double dy = teb_band_[i].y - py;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) {
        best = d2;
        bi = i;
        }
    }
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "[TEB Controller]: Finding closest index finished in findClosestIndex.");
    out_idx = bi;
    return true;
}

bool TebController::sampleLookaheadTargetArc(
    size_t start_idx,
    double lookahead,
    double & tx, double & ty) const
{
    if (teb_band_.empty()) return false;
    if (start_idx >= teb_band_.size()) start_idx = teb_band_.size() - 1;

    double acc = 0.0;
    RCLCPP_INFO(logger_, "[TEB Controller]: Sampling lookahead target started in sampleLookaheadTargetArc.");
    for (size_t i = start_idx; i + 1 < teb_band_.size(); ++i) {
        const auto & a = teb_band_[i];
        const auto & b = teb_band_[i + 1];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        if (seg < 1e-9) continue;

        if (acc + seg >= lookahead) {
        const double r = (lookahead - acc) / seg;  // 0..1
        tx = a.x + r * (b.x - a.x);
        ty = a.y + r * (b.y - a.y);
        return true;
        }
        acc += seg;
    }
    RCLCPP_INFO(logger_, "[TEB Controller]: Sampling lookahead target finished in sampleLookaheadTargetArc.");

    tx = teb_band_.back().x;
    ty = teb_band_.back().y;
    return true;
}

void TebController::publishTebPath()
{
    if (!teb_path_pub_ || !teb_path_pub_->is_activated()) {
        return;
    }

    nav_msgs::msg::Path p;
    p.header = global_plan_.header;
    p.poses.reserve(teb_band_.size());

    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "[TEB Controller]: Publishing TEB path started in publishTebPath.");
    for (const auto & st : teb_band_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = p.header;
        ps.pose.position.x = st.x;
        ps.pose.position.y = st.y;
        ps.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, st.theta);
        ps.pose.orientation = tf2::toMsg(q);

        p.poses.push_back(ps);
    }
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "[TEB Controller]: Publishing TEB path finished in publishTebPath.");

    if (teb_path_pub_ && teb_path_pub_->is_activated()) {
        teb_path_pub_->publish(p);
    }
}

void TebController::publishGoalReached() const
{
    if (!goal_reached_pub_) {
        return;
    }

    std_msgs::msg::Bool msg;
    msg.data = true;
    goal_reached_pub_->publish(msg);
}

bool TebController::sendEscapeGoal(
    const geometry_msgs::msg::PoseStamped & pose,
    const RivalInfo & rival)
{
    if (!navigate_to_pose_client_ || !navigate_to_pose_client_->action_server_is_ready()) {
        return false;
    }

    double target_x = 0.0;
    double target_y = 0.0;
    if (!findRivalEscapeTarget(pose, rival, target_x, target_y)) {
        return false;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = global_plan_.header.frame_id.empty() ? "map" : global_plan_.header.frame_id;
    goal_msg.pose.header.stamp = clock_->now();
    goal_msg.pose.pose.position.x = target_x;
    goal_msg.pose.pose.position.y = target_y;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation = pose.pose.orientation;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
            std::scoped_lock lk(mtx_);
            if (!goal_handle) {
                escape_navigation_active_ = false;
                rival_escape_goal_requested_ = false;
            }
        };
    send_goal_options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult & result) {
            std::scoped_lock lk(mtx_);
            escape_navigation_active_ = false;
            rival_escape_goal_requested_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                publishGoalReached();
            }
        };

    escape_navigation_active_ = true;
    rival_escape_goal_requested_ = true;
    navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
}

bool TebController::shouldTriggerReplan(bool raw_blocked, const rclcpp::Time & now)
{
    if (!raw_blocked) {
        blocked_since_ = rclcpp::Time(0, 0, now.get_clock_type());
        return false;
    }

    if (blocked_since_.nanoseconds() == 0) {
        blocked_since_ = now;
    }
    const double blocked_sec = (now - blocked_since_).seconds();
    if (blocked_sec < replan_min_blocked_time_) {
        return false;
    }
    if (last_replan_time_.nanoseconds() != 0) {
        const double cooldown_sec = (now - last_replan_time_).seconds();
        if (cooldown_sec < replan_cooldown_) {
            return false;
        }
    }

    last_replan_time_ = now;
    RCLCPP_INFO(logger_, "[%s] replan allowed after blocked duration %.3fs", name_.c_str(), blocked_sec);
    return true;
}

TebController::RivalInfo TebController::getRivalInfo(const geometry_msgs::msg::PoseStamped & pose) const
{
    RivalInfo rival;
    if (!has_rival_pose_) {
        return rival;
    }

    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    const double yaw = yawFromQuat(pose.pose.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    rival.valid = true;
    rival.dx_world = latest_rival_pose_.pose.pose.position.x - px;
    rival.dy_world = latest_rival_pose_.pose.pose.position.y - py;
    rival.distance = std::hypot(rival.dx_world, rival.dy_world);
    rival.dx_body = c * rival.dx_world + s * rival.dy_world;
    rival.dy_body = -s * rival.dx_world + c * rival.dy_world;
    return rival;
}

bool TebController::shouldEnterRivalEscape(
    const RivalInfo & rival,
    double cmd_vx,
    double cmd_vy) const
{
    if (!rival.valid || rival.distance > rival_stop_distance_) {
        return false;
    }

    const bool rival_in_front_half_plane = rival.dx_body >= -0.05;
    const double approach_projection = cmd_vx * rival.dx_body + cmd_vy * rival.dy_body;
    const bool pushing_toward_rival = approach_projection >= -0.01;
    return rival_in_front_half_plane && pushing_toward_rival;
}

bool TebController::shouldExitRivalEscape(
    const geometry_msgs::msg::PoseStamped & pose,
    const RivalInfo & rival,
    bool blocked_and_close,
    bool pose_collision) const
{
    const double release_distance =
        rival_stop_distance_ + std::max(0.05, rival_escape_distance_ * 0.5);

    if (rival.valid && rival.distance <= release_distance) {
        return false;
    }

    if (blocked_and_close || pose_collision) {
        return false;
    }

    return distanceFromEscapeStart(pose) >= std::max(0.03, rival_escape_distance_ * 0.5);
}

bool TebController::findRivalEscapeTarget(
    const geometry_msgs::msg::PoseStamped & pose,
    const RivalInfo & rival,
    double & target_x,
    double & target_y) const
{
    double away_x = -rival.dx_world;
    double away_y = -rival.dy_world;
    const double norm = std::hypot(away_x, away_y);
    if (norm < 1e-6) {
        const double yaw = yawFromQuat(pose.pose.orientation);
        away_x = -std::cos(yaw);
        away_y = -std::sin(yaw);
    } else {
        away_x /= norm;
        away_y /= norm;
    }

    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    const double step = std::max(0.02, rival_escape_distance_ / 10.0);
    const double release_distance =
        rival_stop_distance_ + std::max(0.05, rival_escape_distance_ * 0.5);
    const double clearance_margin = std::max(0.03, rival_escape_distance_ * 0.25);
    const double min_escape_distance =
        std::max(step, release_distance - rival.distance + clearance_margin);
    const double search_limit =
        std::max(rival_escape_distance_, min_escape_distance + clearance_margin);

    for (double distance = min_escape_distance; distance <= search_limit + 1e-6; distance += step) {
        const double candidate_x = px + away_x * distance;
        const double candidate_y = py + away_y * distance;
        const unsigned char cost = costAtGlobal(candidate_x, candidate_y);
        if (!treat_no_info_as_obstacle_ && cost == nav2_costmap_2d::NO_INFORMATION) {
            target_x = candidate_x;
            target_y = candidate_y;
            return true;
        }
        if (cost < obstacle_cost_threshold_) {
            target_x = candidate_x;
            target_y = candidate_y;
            return true;
        }
    }
    return false;
}

double TebController::distanceFromEscapeStart(const geometry_msgs::msg::PoseStamped & pose) const
{
    if (!has_rival_escape_start_) {
        return 0.0;
    }

    const double dx = pose.pose.position.x - rival_escape_start_pose_.pose.position.x;
    const double dy = pose.pose.position.y - rival_escape_start_pose_.pose.position.y;
    return std::hypot(dx, dy);
}

void TebController::beginRivalEscape(const geometry_msgs::msg::PoseStamped & pose)
{
    motion_mode_ = MotionMode::RivalEscape;
    rival_escape_start_pose_ = pose;
    has_rival_escape_start_ = true;
    rival_escape_pending_stop_ = true;
    rival_escape_goal_requested_ = false;
    rival_escape_stall_cycles_ = 0;
    rival_escape_attempt_count_++;

    RCLCPP_WARN(
        logger_,
        "[%s] rival escape #%d started from robot=(%.3f, %.3f)",
        name_.c_str(),
        rival_escape_attempt_count_,
        pose.pose.position.x,
        pose.pose.position.y);
}

void TebController::resetRivalEscapeState()
{
    motion_mode_ = MotionMode::FollowPath;
    rival_escape_start_pose_ = geometry_msgs::msg::PoseStamped{};
    has_rival_escape_start_ = false;
    rival_escape_pending_stop_ = false;
    rival_escape_goal_requested_ = false;
    rival_escape_stall_cycles_ = 0;
}

void TebController::updateRivalStopDistance()
{
    if (!external_rival_data_path_.empty()) {
        try {
            YAML::Node config = YAML::LoadFile(external_rival_data_path_);
            if (config["nav_rival_parameters"] &&
                config["nav_rival_parameters"]["rival_inscribed_radius"])
            {
                rival_stop_distance_ =
                    config["nav_rival_parameters"]["rival_inscribed_radius"].as<double>() +
                    kRivalStopDistanceMargin;
                rival_stop_distance_ = std::max(0.0, rival_stop_distance_);
                if (rival_stop_distance_prev_ != rival_stop_distance_) {
                    RCLCPP_WARN(
                        logger_,
                        "[%s] rival_stop_distance updated to %f",
                        name_.c_str(),
                        rival_stop_distance_);
                }
            } else {
                RCLCPP_WARN(
                    logger_,
                    "rival_inscribed_radius not found in YAML file, using default value");
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                logger_,
                "Failed to load YAML file: %s, using default value",
                e.what());
        }
    }
    rival_stop_distance_prev_ = rival_stop_distance_;
}

bool TebController::buildRivalEscapeCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const RivalInfo & rival,
    double current_speed,
    geometry_msgs::msg::TwistStamped & cmd)
{
    double target_x = 0.0;
    double target_y = 0.0;
    if (!findRivalEscapeTarget(pose, rival, target_x, target_y)) {
        return false;
    }

    const double progress = distanceFromEscapeStart(pose);
    if (progress < 0.03 && current_speed <= stop_v_eps_) {
        rival_escape_stall_cycles_++;
    } else {
        rival_escape_stall_cycles_ = 0;
    }

    if (rival_escape_stall_cycles_ >= kRivalEscapeStallCycleLimit) {
        return false;
    }

    const double dx = target_x - pose.pose.position.x;
    const double dy = target_y - pose.pose.position.y;
    const double norm = std::hypot(dx, dy);
    if (norm < 1e-6) {
        return false;
    }
    const double dir_x = dx / norm;
    const double dir_y = dy / norm;
    const double yaw = yawFromQuat(pose.pose.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double vel_x_world = dir_x * rival_escape_speed_;
    const double vel_y_world = dir_y * rival_escape_speed_;

    cmd.twist.linear.x = c * vel_x_world + s * vel_y_world;
    cmd.twist.linear.y = -s * vel_x_world + c * vel_y_world;
    cmd.twist.angular.z = 0.0;

    RCLCPP_WARN(
        logger_,
        "[%s] rival escape #%d target=(%.3f, %.3f) cmd=(vx=%.3f, vy=%.3f, wz=%.3f) speed=%.3f",
        name_.c_str(),
        rival_escape_attempt_count_,
        target_x,
        target_y,
        cmd.twist.linear.x,
        cmd.twist.linear.y,
        cmd.twist.angular.z,
        std::hypot(cmd.twist.linear.x, cmd.twist.linear.y));
    return true;
}

void TebController::applyRivalSlowdown(
    const RivalInfo & rival,
    double & vx,
    double & vy,
    double & w) const
{
    if (!rival.valid || rival.distance >= rival_slowdown_dist_) {
        return;
    }

    const double vmag = std::hypot(vx, vy);
    if (vmag <= 1e-6) {
        return;
    }

    double scale = 1.0;
    if (rival.distance <= rival_close_distance_) {
        scale = rival_min_speed_scale_;
    } else {
        const double t = clamp(
            (rival.distance - rival_close_distance_) /
            std::max(1e-6, rival_slowdown_dist_ - rival_close_distance_),
            0.0, 1.0);
        scale = rival_min_speed_scale_ + (1.0 - rival_min_speed_scale_) * t;
    }

    vx *= scale;
    vy *= scale;
    w *= scale;
}

geometry_msgs::msg::TwistStamped TebController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    std::scoped_lock lk(mtx_);
    updateRivalStopDistance();

    geometry_msgs::msg::TwistStamped cmd;
    const rclcpp::Time now = clock_->now();
    cmd.header.stamp = now;
    cmd.header.frame_id = costmap_ros_->getBaseFrameID();

    if (!has_plan_ || teb_band_.size() < 2) {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "[%s] no valid teb plan, publishing zero velocity", name_.c_str());
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
        return cmd;
    }

    // Correct GoalChecker signature
    if (goal_checker && goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity))
    {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "[%s] goal checker reported goal reached", name_.c_str());
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
        return cmd;
    }

    auto global_grid = latest_global_costmap_;
    if (!global_grid) {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "[%s] global costmap is unavailable, publishing zero velocity", name_.c_str());
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
        return cmd;
    }

    const unsigned char mc = maxCostOnBandGlobal();
    const bool cost_bad = (mc >= (unsigned char)std::lround(max_cost_threshold_));

    size_t closest = 0;
    findClosestIndex(pose, closest);
    const double arc_window = std::max(lookahead_dist_, blocked_stop_clearance_ * 2.0);
    const double clearance = minObstacleDistanceOnBandGlobal(closest, arc_window, blocked_stop_clearance_);
    const bool blocked_and_close = cost_bad && (clearance <= blocked_stop_clearance_);

    double tx = teb_band_.back().x;
    double ty = teb_band_.back().y;
    sampleLookaheadTargetArc(closest, lookahead_dist_, tx, ty);

    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    const double yaw = yawFromQuat(pose.pose.orientation);
    const double dx_w = tx - px;
    const double dy_w = ty - py;

    const double gx = global_plan_.poses.back().pose.position.x;
    const double gy = global_plan_.poses.back().pose.position.y;
    const double goal_dist = std::hypot(gx - px, gy - py);
    const double goal_yaw = yawFromQuat(global_plan_.poses.back().pose.orientation);

    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double dx_b = c * dx_w + s * dy_w;
    const double dy_b = -s * dx_w + c * dy_w;

    double vx = k_xy_ * dx_b;
    double vy = k_xy_ * dy_b;

    const unsigned char pose_cost = costAtGlobal(px, py);
    const bool pose_collision = (pose_cost >= obstacle_cost_threshold_);
    const double v_cur = std::hypot(velocity.linear.x, velocity.linear.y);

    if (goal_dist <= goal_xy_stop_dist_) {
        vx = 0.0;
        vy = 0.0;
    }

    double target_yaw = std::atan2(dy_w, dx_w);
    if (goal_dist <= goal_heading_switch_dist_) {
        target_yaw = goal_yaw;
    }
    double heading_err = normAngle(target_yaw - yaw);
    double w = k_w_ * heading_err;

    double vmag = std::hypot(vx, vy);
    if (vmag > max_v_) {
        const double r = max_v_ / std::max(1e-9, vmag);
        vx *= r;
        vy *= r;
        w *= r;
        vmag = max_v_;
    }

    if (min_v_ > 0.0 && vmag > 1e-6 && vmag < min_v_ && goal_dist > goal_xy_stop_dist_) {
        const double r = min_v_ / vmag;
        vx *= r;
        vy *= r;
        w *= r;
    }

    const RivalInfo rival = getRivalInfo(pose);
    applyRivalSlowdown(rival, vx, vy, w);
    vmag = std::hypot(vx, vy);
    if (!escape_navigation_active_ &&
        (motion_mode_ == MotionMode::RivalEscape || shouldEnterRivalEscape(rival, vx, vy))) {
        if (motion_mode_ != MotionMode::RivalEscape) {
            beginRivalEscape(pose);
        }

        if (!shouldExitRivalEscape(pose, rival, blocked_and_close, pose_collision)) {
            if (rival_escape_pending_stop_) {
                cmd.twist.linear.x = 0.0;
                cmd.twist.linear.y = 0.0;
                cmd.twist.angular.z = 0.0;
                if (v_cur <= stop_v_eps_) {
                    rival_escape_pending_stop_ = false;
                }
                publishTebPath();
                return cmd;
            }

            if (!rival_escape_goal_requested_ && !sendEscapeGoal(pose, rival)) {
                resetRivalEscapeState();
                throw nav2_core::PlannerException("TEB: rival escape goal dispatch failed");
            }
            cmd.twist.linear.x = 0.0;
            cmd.twist.linear.y = 0.0;
            cmd.twist.angular.z = 0.0;
            publishTebPath();
            return cmd;
        }

        resetRivalEscapeState();
    } else {
        resetRivalEscapeState();
    }

    if (escape_navigation_active_) {
        w = 0.0;
    }

    if (blocked_and_close) {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
        publishTebPath();
        if (v_cur <= stop_v_eps_ && shouldTriggerReplan(true, now)) {
            throw nav2_core::PlannerException("TEB: max_cost exceeded and speed low -> replan");
        }
        return cmd;
    }

    w = clamp(w, -max_w_, max_w_);

    if (speed_limit_ > 0.0) {
        double vlim = max_v_;
        if (speed_limit_is_percentage_) {
            vlim = max_v_ * clamp(speed_limit_, 0.0, 100.0) / 100.0;
        } else {
            vlim = clamp(speed_limit_, 0.0, max_v_);
        }

        const double vmag2 = std::hypot(vx, vy);
        if (vmag2 > vlim) {
            const double r = vlim / std::max(1e-9, vmag2);
            vx *= r;
            vy *= r;
        }
    }

    const double check_arc = std::max(obstacle_check_lookahead_, vmag * obstacle_check_time_horizon_);

    bool path_blocked = false;
    double acc_arc = 0.0;
    for (size_t i = closest; i < teb_band_.size(); ++i) {
        const auto & st = teb_band_[i];
        const unsigned char c_band = costAtGlobal(st.x, st.y);
        if (c_band >= obstacle_cost_threshold_) {
            path_blocked = true;
            break;
        }
        if (i + 1 < teb_band_.size()) {
            acc_arc += std::hypot(teb_band_[i + 1].x - st.x, teb_band_[i + 1].y - st.y);
            if (acc_arc >= check_arc) {
                break;
            }
        }
    }

    const double min_obst_path =
        minObstacleDistanceOnBandGlobal(closest, check_arc, slowdown_obstacle_dist_);
    const double obst_pose_dist =
        minObstacleDistanceGlobal(px, py, slowdown_obstacle_dist_);
    double obst_dist = std::min(min_obst_path, obst_pose_dist);
    if (pose_collision) {
        obst_dist = 0.0;
    }

    bool request_replan = false;
    if (path_blocked || obst_dist <= stop_obstacle_dist_) {
        vx = 0.0;
        vy = 0.0;
        vmag = 0.0;
        w = 0.0;
        request_replan = true;
    } else if (std::isfinite(obst_dist) && obst_dist <= slowdown_obstacle_dist_) {
        const double alpha = (obst_dist - stop_obstacle_dist_) /
                             std::max(1e-6, slowdown_obstacle_dist_ - stop_obstacle_dist_);
        const double scale = clamp(alpha, 0.0, 1.0);
        vx *= scale;
        vy *= scale;
        vmag *= scale;
    }

    if (last_stamp_.nanoseconds() != 0) {
        const double dt = (now - last_stamp_).seconds();
        if (dt > 1e-3) {
            const double dvx = clamp(vx - last_vx_, -max_acc_v_ * dt, max_acc_v_ * dt);
            const double dvy = clamp(vy - last_vy_, -max_acc_v_ * dt, max_acc_v_ * dt);
            const double dw = clamp(w - last_w_, -max_acc_w_ * dt, max_acc_w_ * dt);
            vx = last_vx_ + dvx;
            vy = last_vy_ + dvy;
            w = last_w_ + dw;
        }
    }
    last_stamp_ = now;
    last_vx_ = vx;
    last_vy_ = vy;
    last_w_ = w;

    const bool replan_ready = shouldTriggerReplan(request_replan, now);
    if (request_replan) {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
        publishTebPath();
        if (replan_ready) {
            throw nav2_core::PlannerException("TEB: path blocked or obstacle too close -> replan");
        }
        return cmd;
    }

    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.angular.z = w;
    publishTebPath();
    return cmd;
}

}  // namespace teb_controller

PLUGINLIB_EXPORT_CLASS(teb_controller::TebController, nav2_core::Controller)
