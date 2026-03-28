#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"

using namespace std::chrono_literals;

class SimSystemCheck : public rclcpp::Node
{
public:
  const int GROUP_NAVIGATION = 3;
  const int STATE_READY = 1;

  SimSystemCheck() : Node("sim_system_check")
  {
    are_you_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot/startup/are_you_ready", 10,
      std::bind(&SimSystemCheck::areYouReadyCallback, this, std::placeholders::_1));

    ready_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>(
      "/robot/startup/ready_signal");

    this->declare_parameter("ready_delay_sec_", 0.5);
    ready_delay_sec_ = this->get_parameter("ready_delay_sec_").as_double();
    RCLCPP_INFO(this->get_logger(), "SystemCheck ready, waiting for are_you_ready");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr are_you_ready_sub_;
  rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_client_;
  rclcpp::TimerBase::SharedPtr delay_timer_;
  
  bool ready_sent_;
  double ready_delay_sec_;

  void areYouReadyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || ready_sent_)
      return;

    if (!ready_client_->wait_for_service(0s)) {
      RCLCPP_WARN(this->get_logger(),
        "[SIM] ready_signal service not available");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "[SIM] are_you_ready received, scheduling READY");

    delay_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(ready_delay_sec_),
      std::bind(&SimSystemCheck::sendReadySignal, this));
  }

  void sendReadySignal()
  {
    delay_timer_->cancel();
    ready_sent_ = true;

    auto req = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();

    req->group = GROUP_NAVIGATION;
    req->state = STATE_READY;

    using ResponseFuture = rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture;

    ready_client_->async_send_request(req,[this](ResponseFuture future) {
      auto res = future.get();
      if (res->success) {
        RCLCPP_INFO(this->get_logger(),
          "[SIM] READY acknowledged by Startup");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "[SIM] READY rejected");
        ready_sent_ = false;  // allow retry
      }
    });
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimSystemCheck>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}