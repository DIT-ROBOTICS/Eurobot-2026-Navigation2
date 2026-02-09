#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class SystemCheck : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using DockRobot = opennav_docking_msgs::action::DockRobot;

  const int GROUP_NAVIGATION = 3;
  const int STATE_READY = 1;

  SystemCheck()
  : Node("system_check"), running_(false), obstacle_check_count_(10)
  {
    are_you_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot/startup/are_you_ready", 10,
      std::bind(&SystemCheck::areYouReadyCallback, this, std::placeholders::_1));

    costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(10),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          latest_costmap_ = msg;
      });

    subscription_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/final_pose", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_pose_ = msg;
      });

    ready_srv_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>(
      "/robot/startup/ready_signal");

    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    dock_robot_client_ = rclcpp_action::create_client<DockRobot>(this, "dock_robot");

    this->declare_parameter("costmap_tolerance", 70);
    this->get_parameter("costmap_tolerance", costmap_tolerance_);
    this->declare_parameter("external_rival_data_path", "");
    this->get_parameter("external_rival_data_path", external_rival_data_path_);

    RCLCPP_INFO(this->get_logger(), "\033[1;35m SystemCheck started, waiting for startup plan... \033[0m");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr are_you_ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_pose_;
  nav_msgs::msg::Odometry::SharedPtr latest_pose_;
  rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<DockRobot>::SharedPtr dock_robot_client_;
  rclcpp::TimerBase::SharedPtr obstacle_timer_;
  bool running_;
  int costmap_tolerance_;
  std::string external_rival_data_path_;
  int obstacle_check_count_;

  void areYouReadyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || running_) return;
    
    running_ = true;  // Set running state to true to prevent re-entrance
    RCLCPP_INFO(this->get_logger(), "[NAVIGATION] are_you_ready received, starting system check...");

    // Wait for services and action servers
    if (!dependenciesReady()) {
      RCLCPP_WARN(this->get_logger(), "[NAVIGATION] Dependencies not ready");
      running_ = false; // Reset running state to allow retry
      return;
    }

    // Start obstacle check timer if in obstacle
    if (inObstacle()) {
      RCLCPP_WARN(this->get_logger(), "Robot is in obstacles, waiting for manual action...");
      obstacle_check_count_ = 10;

      if (!obstacle_timer_){
        obstacle_timer_ = this->create_wall_timer(1s, std::bind(&SystemCheck::obstacleCheckTimer, this));
      }
      return;
    }

    sendReadySignal(GROUP_NAVIGATION, STATE_READY);
    running_ = false;
  }

  bool dependenciesReady() {
    // Check if the service and action servers are available
    if (!ready_srv_client_->wait_for_service(0s)) {
      RCLCPP_WARN(this->get_logger(), "Ready service not available after waiting");
      return false;
    }
    if (!navigate_to_pose_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN(this->get_logger(), "NavigateToPose action server not available after waiting");
      return false;
    }
    if (!dock_robot_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN(this->get_logger(), "DockRobot action server not available after waiting");
      return false;
    }
    return true;
  }

  void obstacleCheckTimer()
  {
    if (!inObstacle()) {
      obstacle_timer_->cancel();
      obstacle_timer_.reset();
      RCLCPP_INFO(this->get_logger(), "\033[1;32m [NAVIGATION] Obstacle cleared \033[0m");
      sendReadySignal(GROUP_NAVIGATION, STATE_READY);
      running_ = false;
      return;
    }

    if (obstacle_check_count_-- <= 0) {
      shrinkRivalRadius();
      obstacle_check_count_ = 10; // Reset the counter after shrinking
    }
  }

  // Update sendReadySignal as follows:
  void sendReadySignal(int group, int state)
  {
    auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
    request->group = group;
    request->state = state;
    ready_srv_client_->async_send_request(request,
      [this, group, state](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m ReadySignal SUCCESS: group=%d \033[0m", response->group);
        } else {
          RCLCPP_WARN(this->get_logger(), "[NAVIGATION] READY rejected");
        }
      });
  } 

  bool inObstacle() {
    if (!latest_costmap_ || !latest_pose_) {
        RCLCPP_WARN(this->get_logger(), "[NAVIGATION] No costmap or pose received yet.");
        return true;
    }

    int mapX = static_cast<int>(latest_pose_->pose.pose.position.x * 100.0);
    int mapY = static_cast<int>(latest_pose_->pose.pose.position.y * 100.0);
    int width = latest_costmap_->info.width;
    int index = mapY * width + mapX;

    if (index < 0 || index >= static_cast<int>(latest_costmap_->data.size()))
      return false;

    return latest_costmap_->data[index] > costmap_tolerance_;
  }

  void shrinkRivalRadius() {
    if (external_rival_data_path_.empty()) return;

    try {
      YAML::Node config = YAML::LoadFile(external_rival_data_path_);
      YAML::Node node = config["nav_rival_parameters"]["rival_inscribed_radius"];
      if (!node) return;

      double r = node.as<double>();
      r = std::max(0.0, r - 0.01); // Decrease radius but not below 0.05
      config["nav_rival_parameters"]["rival_inscribed_radius"] = r;
      std::ofstream out(external_rival_data_path_);
      out << config;

      RCLCPP_INFO(this->get_logger(), "[NAVIGATION] shrink rival_inscribed_radius to %.2f", r);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to shrink rival radius %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemCheck>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}