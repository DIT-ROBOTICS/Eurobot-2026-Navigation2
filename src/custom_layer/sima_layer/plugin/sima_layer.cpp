#include "sima_layer/sima_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace Sima_costmap_plugin {

void SimaLayer::onInitialize()
{
  RCLCPP_INFO(rclcpp::get_logger("SimaLayer"), "Initializing SimaLayer");

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  enabled_ = true;
  current_ = true;
  no_rival_ = true;

  matchSize();
  resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("model_size", rclcpp::ParameterValue(22));
  declareParameter("x_cov_threshold", rclcpp::ParameterValue(0.01));
  declareParameter("y_cov_threshold", rclcpp::ParameterValue(0.01));
  declareParameter("R_sq_threshold", rclcpp::ParameterValue(0.85));
  declareParameter("auto_reset_with_timeout", rclcpp::ParameterValue(true));
  declareParameter("reset_timeout_threshold", rclcpp::ParameterValue(40));
  declareParameter("robot_inscribed_radius", rclcpp::ParameterValue(0.22));
  declareParameter("external_rival_data_path", rclcpp::ParameterValue(""));
  declareParameter("rival_inscribed_radius", rclcpp::ParameterValue(0.22 * kSimaSizeScale));
  declareParameter("halted_inflation_radius", rclcpp::ParameterValue(0.1));
  declareParameter("wandering_inflation_radius", rclcpp::ParameterValue(0.1));
  declareParameter("moving_inflation_radius", rclcpp::ParameterValue(0.1));
  declareParameter("unknown_inflation_radius", rclcpp::ParameterValue(0.1));
  declareParameter("halted_cost_scaling_factor", rclcpp::ParameterValue(10.0));
  declareParameter("wandering_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declareParameter("moving_cost_scaling_factor", rclcpp::ParameterValue(11.0));
  declareParameter("unknown_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declareParameter("max_extend_length", rclcpp::ParameterValue(0.6));
  declareParameter("cov_range_max", rclcpp::ParameterValue(std::sqrt(0.0029)));
  declareParameter("cov_range_min", rclcpp::ParameterValue(std::sqrt(0.0002)));
  declareParameter("vel_range_max", rclcpp::ParameterValue(1.0));
  declareParameter("vel_range_min", rclcpp::ParameterValue(0.05));
  declareParameter("inscribed_radius_rate", rclcpp::ParameterValue(0.99));
  declareParameter("inflation_radius_rate", rclcpp::ParameterValue(1.005));
  declareParameter("debug_mode", rclcpp::ParameterValue(0));
  declareParameter("offset_vel_factor_weight_statistic", rclcpp::ParameterValue(0.42));
  declareParameter("expand_vel_factor_weight_statistic", rclcpp::ParameterValue(0.20));
  declareParameter("offset_vel_factor_weight_localization", rclcpp::ParameterValue(0.42));
  declareParameter("expand_vel_factor_weight_localization", rclcpp::ParameterValue(0.20));
  declareParameter("safe_distance", rclcpp::ParameterValue(0.5));
  declareParameter("use_statistic_method", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".model_size", model_size_);
  node->get_parameter(name_ + ".x_cov_threshold", x_cov_threshold_);
  node->get_parameter(name_ + ".y_cov_threshold", y_cov_threshold_);
  node->get_parameter(name_ + ".R_sq_threshold", r_sq_threshold_);
  node->get_parameter(name_ + ".auto_reset_with_timeout", auto_reset_with_timeout_);
  node->get_parameter(name_ + ".reset_timeout_threshold", reset_timeout_threshold_);
  node->get_parameter(name_ + ".robot_inscribed_radius", robot_inscribed_radius_);
  node->get_parameter(name_ + ".external_rival_data_path", external_rival_data_path_);
  node->get_parameter(name_ + ".rival_inscribed_radius", rival_inscribed_radius_);
  node->get_parameter(name_ + ".halted_inflation_radius", halted_inflation_radius_);
  node->get_parameter(name_ + ".wandering_inflation_radius", wandering_inflation_radius_);
  node->get_parameter(name_ + ".moving_inflation_radius", moving_inflation_radius_);
  node->get_parameter(name_ + ".unknown_inflation_radius", unknown_inflation_radius_);
  node->get_parameter(name_ + ".halted_cost_scaling_factor", halted_cost_scaling_factor_);
  node->get_parameter(name_ + ".wandering_cost_scaling_factor", wandering_cost_scaling_factor_);
  node->get_parameter(name_ + ".moving_cost_scaling_factor", moving_cost_scaling_factor_);
  node->get_parameter(name_ + ".unknown_cost_scaling_factor", unknown_cost_scaling_factor_);
  node->get_parameter(name_ + ".max_extend_length", max_extend_length_);
  node->get_parameter(name_ + ".cov_range_max", cov_range_max_);
  node->get_parameter(name_ + ".cov_range_min", cov_range_min_);
  node->get_parameter(name_ + ".vel_range_max", vel_range_max_);
  node->get_parameter(name_ + ".vel_range_min", vel_range_min_);
  node->get_parameter(name_ + ".inscribed_radius_rate", inscribed_radius_rate_);
  node->get_parameter(name_ + ".inflation_radius_rate", inflation_radius_rate_);
  node->get_parameter(name_ + ".debug_mode", debug_mode_);
  node->get_parameter(name_ + ".offset_vel_factor_weight_statistic", offset_vel_factor_weight_statistic_);
  node->get_parameter(name_ + ".expand_vel_factor_weight_statistic", expand_vel_factor_weight_statistic_);
  node->get_parameter(name_ + ".offset_vel_factor_weight_localization", offset_vel_factor_weight_localization_);
  node->get_parameter(name_ + ".expand_vel_factor_weight_localization", expand_vel_factor_weight_localization_);
  node->get_parameter(name_ + ".safe_distance", safe_distance_);
  node->get_parameter(name_ + ".use_statistic_method", use_statistic_method_);

  updateRadius();

  for (std::size_t i = 0; i < kSimaCount; ++i) {
    const std::string odom_topic = "/sima_" + std::to_string(i + 1) + "/odom";
    const std::string distance_topic = "/sima_" + std::to_string(i + 1) + "/distance";
    odom_subs_[i] = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 100,
      [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odomCallback(i, msg);
      });
    distance_subs_[i] = node->create_subscription<std_msgs::msg::Float64>(
      distance_topic, 100,
      [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
        this->distanceCallback(i, msg);
      });
  }

  set_mode_service_ = node->create_service<std_srvs::srv::SetBool>(
    "/sima_layer/set_mode",
    std::bind(&SimaLayer::handleSetMode, this, std::placeholders::_1, std::placeholders::_2));
}

void SimaLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  *min_x = std::min(min_x_, *min_x);
  *min_y = std::min(min_y_, *min_y);
  *max_x = std::max(max_x_, *max_x);
  *max_y = std::max(max_y_, *max_y);
}

void SimaLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) {
    return;
  }

  updateRadius();
  resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

  bool any_active = false;
  for (std::size_t i = 0; i < kSimaCount; ++i) {
    auto & agent = agents_[i];
    if (agent.pose_received) {
      agent.pose_received = false;
      agent.active = true;
      agent.missed_updates = 0;
      no_rival_ = false;
    } else if (agent.active) {
      agent.missed_updates++;
      if (auto_reset_with_timeout_ && agent.missed_updates >= reset_timeout_threshold_) {
        agent.active = false;
        agent.state = RivalState::UNKNOWN;
      }
    }

    if (!agent.active) {
      continue;
    }

    updateAgentState(agent);
    fieldExpansion(agent);
    any_active = true;
  }

  if (!any_active) {
    no_rival_ = true;
  }

  updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
}

bool SimaLayer::isClearable()
{
  return true;
}

void SimaLayer::reset()
{
  current_ = true;
  no_rival_ = true;
  for (auto & agent : agents_) {
    agent = SimaAgentState{};
  }
  resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
}

void SimaLayer::activate()
{
  RCLCPP_INFO(rclcpp::get_logger("SimaLayer"), "Activating SimaLayer");
}

void SimaLayer::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("SimaLayer"), "Deactivating SimaLayer");
}

void SimaLayer::handleSetMode(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  mode_param_ = request->data;
  response->success = true;
  response->message = mode_param_ ? "SimaLayer is in shrink mode" : "SimaLayer is in default mode";
}

void SimaLayer::odomCallback(std::size_t index, const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (index >= kSimaCount) {
    return;
  }

  auto & agent = agents_[index];
  if (msg->header.frame_id.empty()) {
    agent.active = false;
    agent.state = RivalState::UNKNOWN;
    return;
  }

  agent.x = msg->pose.pose.position.x;
  agent.y = msg->pose.pose.position.y;
  agent.vx = msg->twist.twist.linear.x;
  agent.vy = msg->twist.twist.linear.y;
  agent.pose_received = true;
  agent.distance = std::hypot(agent.x, agent.y);
}

void SimaLayer::distanceCallback(std::size_t index, const std_msgs::msg::Float64::SharedPtr msg)
{
  if (index < kSimaCount) {
    agents_[index].distance = msg->data;
  }
}

void SimaLayer::updateRadius()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  double base_rival_radius = 0.22 * kSimaSizeScale;
  node->get_parameter(name_ + ".rival_inscribed_radius", base_rival_radius);
  if (!external_rival_data_path_.empty()) {
    try {
      YAML::Node config = YAML::LoadFile(external_rival_data_path_);
      if (config["nav_rival_parameters"] && config["nav_rival_parameters"]["rival_inscribed_radius"]) {
        base_rival_radius = config["nav_rival_parameters"]["rival_inscribed_radius"].as<double>() * kSimaSizeScale;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SimaLayer"), "Failed to load YAML file: %s", e.what());
    }
  }

  rival_inscribed_radius_ = base_rival_radius + robot_inscribed_radius_;
  rival_inscribed_radius_prev_ = rival_inscribed_radius_;

  if (!mode_param_) {
    node->get_parameter(name_ + ".halted_inflation_radius", halted_inflation_radius_);
    node->get_parameter(name_ + ".wandering_inflation_radius", wandering_inflation_radius_);
    node->get_parameter(name_ + ".moving_inflation_radius", moving_inflation_radius_);
    node->get_parameter(name_ + ".unknown_inflation_radius", unknown_inflation_radius_);
  } else {
    halted_inflation_radius_ = 0.02;
    wandering_inflation_radius_ = 0.02;
    moving_inflation_radius_ = 0.02;
    unknown_inflation_radius_ = 0.02;
  }

  halted_inflation_radius_ += rival_inscribed_radius_;
  wandering_inflation_radius_ += rival_inscribed_radius_;
  moving_inflation_radius_ += rival_inscribed_radius_;
  unknown_inflation_radius_ += rival_inscribed_radius_;
}

void SimaLayer::updateAgentState(SimaAgentState & agent)
{
  const double speed = std::hypot(agent.vx, agent.vy);
  if (speed < std::max(vel_range_min_, 0.05)) {
    agent.state = RivalState::HALTED;
  } else {
    agent.state = RivalState::MOVING;
  }
}

void SimaLayer::fieldExpansion(const SimaAgentState & agent)
{
  switch (agent.state) {
    case RivalState::HALTED:
      expandPointWithCircle(
        agent.x, agent.y, nav2_costmap_2d::MAX_NON_OBSTACLE,
        halted_inflation_radius_, halted_cost_scaling_factor_, rival_inscribed_radius_);
      break;
    case RivalState::MOVING:
      expandPointWithCircle(
        agent.x, agent.y, nav2_costmap_2d::MAX_NON_OBSTACLE,
        halted_inflation_radius_, halted_cost_scaling_factor_, rival_inscribed_radius_);
      expandLine(
        agent, nav2_costmap_2d::MAX_NON_OBSTACLE, moving_inflation_radius_,
        moving_cost_scaling_factor_, rival_inscribed_radius_, max_extend_length_);
      break;
    case RivalState::UNKNOWN:
    default:
      expandPointWithCircle(
        agent.x, agent.y, nav2_costmap_2d::MAX_NON_OBSTACLE,
        unknown_inflation_radius_, unknown_cost_scaling_factor_, rival_inscribed_radius_);
      break;
  }
}

void SimaLayer::expandPointWithCircle(
  double x, double y, double max_cost, double inflation_radius,
  double cost_scaling_factor, double inscribed_radius)
{
  const double max_x = x + inflation_radius;
  const double min_x = x - inflation_radius;
  unsigned int mx = 0;
  unsigned int my = 0;

  for (double current_x = min_x; current_x <= max_x; current_x += resolution_) {
    const double half_width_sq = inflation_radius * inflation_radius - std::pow(std::fabs(current_x - x), 2);
    if (half_width_sq < 0.0) {
      continue;
    }

    const double max_y = y + std::sqrt(half_width_sq);
    const double min_y = 2.0 * y - max_y;
    for (double current_y = min_y; current_y <= max_y; current_y += resolution_) {
      if (!worldToMap(current_x, current_y, mx, my)) {
        continue;
      }

      const double distance = std::hypot(x - current_x, y - current_y);
      double cost = max_cost;
      if (distance > inscribed_radius) {
        cost = std::ceil(252.0 * std::exp(-cost_scaling_factor * (distance - inscribed_radius)));
        cost = std::clamp(cost, 0.0, max_cost);
      }

      if (getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION) {
        setCost(mx, my, std::max(static_cast<unsigned char>(cost), getCost(mx, my)));
      } else {
        setCost(mx, my, static_cast<unsigned char>(cost));
      }
    }
  }
}

void SimaLayer::expandLine(
  const SimaAgentState & agent, double max_cost, double inflation_radius,
  double cost_scaling_factor, double inscribed_radius, double extend_length)
{
  const double speed = std::hypot(agent.vx, agent.vy);
  if (speed < 1e-6) {
    return;
  }

  const double distance = std::max(agent.distance, 1.0);
  const double vel_factor = std::min(1.0, speed / std::max(vel_range_max_ - vel_range_min_, 1e-6));
  const double position_offset = std::max(agent.distance - safe_distance_, 0.0) * vel_factor * offset_vel_factor_weight_localization_;
  const double direction_x = agent.vx / speed;
  const double direction_y = agent.vy / speed;

  const double start_x = agent.x + position_offset * direction_x;
  const double start_y = agent.y + position_offset * direction_y;
  const double scaled_extend = extend_length * vel_factor * expand_vel_factor_weight_localization_ / distance;
  const int goal_steps = static_cast<int>(scaled_extend / resolution_);

  if (goal_steps <= 0) {
    if (agent.distance < 0.75) {
      expandPointWithCircle(start_x, start_y, max_cost, inflation_radius, cost_scaling_factor, inscribed_radius);
    }
    return;
  }

  double mark_x = 0.0;
  double mark_y = 0.0;
  double current_inscribed_radius = inscribed_radius;
  double current_inflation_radius = inflation_radius;
  for (int i = 0; i < goal_steps; ++i) {
    mark_x += resolution_ * direction_x;
    mark_y += resolution_ * direction_y;
    expandPointWithCircle(
      start_x + mark_x, start_y + mark_y, max_cost,
      current_inflation_radius, cost_scaling_factor, current_inscribed_radius);
    current_inscribed_radius *= inscribed_radius_rate_;
    current_inflation_radius *= inflation_radius_rate_;
  }
}

void SimaLayer::logStateChange(std::size_t /*index*/, RivalState /*new_state*/)
{
}

}  // namespace Sima_costmap_plugin

PLUGINLIB_EXPORT_CLASS(Sima_costmap_plugin::SimaLayer, nav2_costmap_2d::Layer)
