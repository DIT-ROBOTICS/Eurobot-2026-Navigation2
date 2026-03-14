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
  : Node("system_check"), ready_sent_(false)
  {
    are_you_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot/startup/are_you_ready", 10,
      std::bind(&SystemCheck::areYouReadyCallback, this, std::placeholders::_1));

    ready_srv_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>(
      "/robot/startup/ready_signal");

    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    dock_robot_client_ = rclcpp_action::create_client<DockRobot>(this, "dock_robot");

    RCLCPP_INFO(this->get_logger(), "\033[1;35m SystemCheck started, waiting for startup plan... \033[0m");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr are_you_ready_sub_;
  rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<DockRobot>::SharedPtr dock_robot_client_;

  bool ready_sent_;

  void areYouReadyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || ready_sent_) return;

    RCLCPP_INFO(this->get_logger(), "[NAVIGATION] are_you_ready received, starting system check...");

    // Wait for services and action servers
    if (!dependenciesReady()) {
      RCLCPP_WARN(this->get_logger(), "[NAVIGATION] Dependencies not ready");
      return;
    }

    ready_sent_ = true;
    sendReadySignal(GROUP_NAVIGATION, STATE_READY);
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
          ready_sent_ = false;
        }
      });
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