#include "teb_controller/teb_controller.hpp"

#include <cmath>
#include <algorithm>

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
    global_costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap",
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latest_global_costmap_ = msg;
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

    last_stamp_ = rclcpp::Time(0, 0, clock_ ? clock_->get_clock_type() : RCL_ROS_TIME);
    blocked_since_ = rclcpp::Time(0, 0, clock_ ? clock_->get_clock_type() : RCL_ROS_TIME);
    last_replan_time_ = rclcpp::Time(0, 0, clock_ ? clock_->get_clock_type() : RCL_ROS_TIME);
    last_vx_ = last_vy_ = last_w_ = 0.0;
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

geometry_msgs::msg::TwistStamped TebController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    std::scoped_lock lk(mtx_);

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

    // closest index on band
    size_t closest = 0;
    findClosestIndex(pose, closest);
    const double arc_window = std::max(lookahead_dist_, blocked_stop_clearance_ * 2.0);
    const double clearance = minObstacleDistanceOnBandGlobal(closest, arc_window, blocked_stop_clearance_);
    const bool blocked_and_close = cost_bad && (clearance <= blocked_stop_clearance_);

    const double v_cur = std::hypot(velocity.linear.x, velocity.linear.y);
    const bool slow_enough = (v_cur <= stop_v_eps_);

    if (blocked_and_close) {
        RCLCPP_INFO_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] blocked_and_close: max_cost=%u threshold=%.1f clearance=%.3f blocked_stop_clearance=%.3f speed=%.3f",
            name_.c_str(), mc, max_cost_threshold_, clearance, blocked_stop_clearance_, v_cur);
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;

        publishTebPath();
        if (slow_enough && shouldTriggerReplan(true, now)) {
            RCLCPP_INFO(logger_, "[%s] throwing replan because path cost is too high and robot is nearly stopped", name_.c_str());
            throw nav2_core::PlannerException("TEB: max_cost exceeded and speed low -> replan");
        }
        return cmd;
    }

    // lookahead target along arc
    double tx = teb_band_.back().x;
    double ty = teb_band_.back().y;
    sampleLookaheadTargetArc(closest, lookahead_dist_, tx, ty);

    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    const double yaw = yawFromQuat(pose.pose.orientation);

    const double dx_w = tx - px;
    const double dy_w = ty - py;
    // dist_to_target currently unused but kept for potential future tuning

    // final goal info
    const double gx = global_plan_.poses.back().pose.position.x;
    const double gy = global_plan_.poses.back().pose.position.y;
    const double goal_dist = std::hypot(gx - px, gy - py);
    const double goal_yaw = yawFromQuat(global_plan_.poses.back().pose.orientation);

    // translate world -> body
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double dx_b =  c * dx_w + s * dy_w;
    const double dy_b = -s * dx_w + c * dy_w;

    // Position -> (vx,vy) like your CustomController
    double vx = k_xy_ * dx_b;
    double vy = k_xy_ * dy_b;

    const unsigned char pose_cost = costAtGlobal(px, py);
    const bool pose_collision = (pose_cost >= obstacle_cost_threshold_);

    // Near goal: stop translation to avoid overshoot
    if (goal_dist <= goal_xy_stop_dist_) {
        vx = 0.0;
        vy = 0.0;
    }

    // Heading control:
    // far: face the motion direction
    // near: align with final goal yaw
    double target_yaw = std::atan2(dy_w, dx_w);
    if (goal_dist <= goal_heading_switch_dist_) {
        target_yaw = goal_yaw;
    }
    double heading_err = normAngle(target_yaw - yaw);
    double w = k_w_ * heading_err;

    // limit translational speed magnitude
    double vmag = std::hypot(vx, vy);
    if (vmag > max_v_) {
        const double r = max_v_ / std::max(1e-9, vmag);
        vx *= r;
        vy *= r;
        vmag = max_v_;
    }

    // optional min_v_ (usually keep at 0)
    if (min_v_ > 0.0 && vmag > 1e-6 && vmag < min_v_ && goal_dist > goal_xy_stop_dist_) {
        const double r = min_v_ / vmag;
        vx *= r;
        vy *= r;
    }

    w = clamp(w, -max_w_, max_w_);

    // Apply Nav2 speed limit interface (only scale translation)
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

    // Forward obstacle check arc based on commanded speed
    const double check_arc = std::max(obstacle_check_lookahead_, vmag * obstacle_check_time_horizon_);

    // If any band point within check_arc is high-cost, mark path blocked
    bool path_blocked = false;
    double acc_arc = 0.0;
    RCLCPP_INFO(logger_, "[TEB Controller]: Checking path blockage started in computeVelocityCommands.");
    for (size_t i = closest; i < teb_band_.size(); ++i) {
        const auto & st = teb_band_[i];
        const unsigned char c = costAtGlobal(st.x, st.y);
        if (c >= obstacle_cost_threshold_) {
            path_blocked = true;
            break;
        }
        if (i + 1 < teb_band_.size()) {
            acc_arc += std::hypot(teb_band_[i + 1].x - st.x, teb_band_[i + 1].y - st.y);
            if (acc_arc >= check_arc) break;
        }
    }
    RCLCPP_INFO(logger_, "[TEB Controller]: Checking path blockage finished in computeVelocityCommands.");

    // Slow down / stop based on obstacle distance along band or robot pose
    const double min_obst_path =
        minObstacleDistanceOnBandGlobal(closest, check_arc, slowdown_obstacle_dist_);
    const double obst_pose_dist =
        minObstacleDistanceGlobal(px, py, slowdown_obstacle_dist_);
    double obst_dist = std::min(min_obst_path, obst_pose_dist);
    if (pose_collision) obst_dist = 0.0;

    bool request_replan = false;
    RCLCPP_INFO(logger_, "[TEB CONTROLLER]: Check blocked started in computeVelocityCommands");
    if (path_blocked || obst_dist <= stop_obstacle_dist_) {
        RCLCPP_INFO_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] request_replan path_blocked=%d obst_dist=%.3f stop_obstacle_dist=%.3f pose_collision=%d check_arc=%.3f closest=%zu",
            name_.c_str(), path_blocked, obst_dist, stop_obstacle_dist_, pose_collision, check_arc, closest);
        vx = 0.0;
        vy = 0.0;
        vmag = 0.0;
        w = 0.0;
        request_replan = true;
    } else if (std::isfinite(obst_dist) && obst_dist <= slowdown_obstacle_dist_) {
        RCLCPP_INFO_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] slowing down for obstacle: obst_dist=%.3f slowdown_obstacle_dist=%.3f stop_obstacle_dist=%.3f",
            name_.c_str(), obst_dist, slowdown_obstacle_dist_, stop_obstacle_dist_);
        const double alpha = (obst_dist - stop_obstacle_dist_) /
                             std::max(1e-6, slowdown_obstacle_dist_ - stop_obstacle_dist_);
        const double scale = clamp(alpha, 0.0, 1.0);
        vx *= scale;
        vy *= scale;
        vmag *= scale;
    }
    RCLCPP_INFO(logger_, "[TEB CONTROLLER]: Check blocked finished in computeVelocityCommands");
    // Accel limiting on vx, vy, w
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

    RCLCPP_INFO(logger_, "[TEB CONTROLLER]: Check replan started in computeVelocityCommands");
    const bool replan_ready = shouldTriggerReplan(request_replan, now);
    if (request_replan) {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
        publishTebPath();
        if (replan_ready) {
            RCLCPP_INFO(
                logger_,
                "[%s] throwing replan: request_replan=1 path_blocked=%d obst_dist=%.3f pose_cost=%u max_band_cost=%u",
                name_.c_str(), path_blocked, obst_dist, pose_cost, mc);
            throw nav2_core::PlannerException("TEB: path blocked or obstacle too close -> replan");
        }
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "[%s] request_replan active but waiting for replan cooldown / blocked duration", name_.c_str());
        return cmd;
    }
    RCLCPP_INFO(logger_, "[TEB CONTROLLER]: Check replan finished in computeVelocityCommands");
    RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "[%s] cmd vx=%.3f vy=%.3f wz=%.3f closest=%zu goal_dist=%.3f pose_cost=%u band_max_cost=%u obst_dist=%.3f",
        name_.c_str(), vx, vy, w, closest, goal_dist, pose_cost, mc, obst_dist);
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.angular.z = w;
    
    RCLCPP_INFO(logger_, "[TEB CONTROLLER]: publish teb path in computeVelocityCommands");
    publishTebPath();
    return cmd;
}

}  // namespace teb_controller

PLUGINLIB_EXPORT_CLASS(teb_controller::TebController, nav2_core::Controller)