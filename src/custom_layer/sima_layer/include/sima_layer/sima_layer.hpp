#ifndef SIMA_LAYER_HPP_
#define SIMA_LAYER_HPP_

#include <array>
#include <string>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <yaml-cpp/yaml.h>

namespace Sima_costmap_plugin {

class SimaLayer : public nav2_costmap_2d::CostmapLayer {
public:
  SimaLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  bool isClearable() override;
  void reset() override;
  void activate() override;
  void deactivate() override;

  void handleSetMode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

private:
  static constexpr std::size_t kSimaCount = 4;
  static constexpr double kSimaSizeScale = 1.0 / 6.0;

  enum class RivalState {
    HALTED,
    MOVING,
    UNKNOWN
  };

  struct SimaAgentState {
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    double distance{1.0};
    bool active{false};
    bool pose_received{false};
    int missed_updates{0};
    RivalState state{RivalState::UNKNOWN};
  };

  void odomCallback(std::size_t index, const nav_msgs::msg::Odometry::SharedPtr msg);
  void distanceCallback(std::size_t index, const std_msgs::msg::Float64::SharedPtr msg);

  void updateRadius();
  void updateAgentState(SimaAgentState & agent);
  void fieldExpansion(const SimaAgentState & agent);
  void expandPointWithCircle(
    double x, double y, double max_cost, double inflation_radius,
    double cost_scaling_factor, double inscribed_radius);
  void expandLine(
    const SimaAgentState & agent, double max_cost, double inflation_radius,
    double cost_scaling_factor, double inscribed_radius, double extend_length);
  void logStateChange(std::size_t index, RivalState new_state);

  std::array<SimaAgentState, kSimaCount> agents_{};
  std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, kSimaCount> odom_subs_{};
  std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, kSimaCount> distance_subs_{};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;

  bool mode_param_{false};
  bool no_rival_{true};

  int model_size_{22};
  double x_cov_threshold_{0.01};
  double y_cov_threshold_{0.01};
  double r_sq_threshold_{0.85};
  bool auto_reset_with_timeout_{true};
  int reset_timeout_threshold_{40};
  double robot_inscribed_radius_{0.22};
  std::string external_rival_data_path_;
  double rival_inscribed_radius_{0.22 * kSimaSizeScale};
  double rival_inscribed_radius_prev_{0.22 * kSimaSizeScale};
  double halted_inflation_radius_{0.1};
  double wandering_inflation_radius_{0.1};
  double moving_inflation_radius_{0.1};
  double unknown_inflation_radius_{0.1};
  double halted_cost_scaling_factor_{10.0};
  double wandering_cost_scaling_factor_{3.0};
  double moving_cost_scaling_factor_{11.0};
  double unknown_cost_scaling_factor_{3.0};
  double max_extend_length_{0.6};
  double cov_range_max_{0.05385164807134504};
  double cov_range_min_{0.01414213562373095};
  double vel_range_max_{1.0};
  double vel_range_min_{0.05};
  double inscribed_radius_rate_{0.99};
  double inflation_radius_rate_{1.005};
  int debug_mode_{0};
  double offset_vel_factor_weight_statistic_{0.42};
  double expand_vel_factor_weight_statistic_{0.20};
  double offset_vel_factor_weight_localization_{0.42};
  double expand_vel_factor_weight_localization_{0.20};
  double safe_distance_{0.5};
  bool use_statistic_method_{false};

  double min_x_{0.0};
  double min_y_{0.0};
  double max_x_{3.0};
  double max_y_{2.0};
};

}  // namespace Sima_costmap_plugin

#endif  // SIMA_LAYER_HPP_
