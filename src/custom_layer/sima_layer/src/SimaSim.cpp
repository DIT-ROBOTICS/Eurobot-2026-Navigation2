#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class SimaSimNode : public rclcpp::Node
{
public:
  SimaSimNode()
  : Node("sima_sim_node")
  {
    timer_ = create_wall_timer(50ms, std::bind(&SimaSimNode::onTimer, this));

    for (std::size_t i = 0; i < kCount; ++i) {
      const auto idx = std::to_string(i + 1);
      odom_pubs_[i] = create_publisher<nav_msgs::msg::Odometry>("/sima_" + idx + "/odom", 20);
      distance_pubs_[i] = create_publisher<std_msgs::msg::Float64>("/sima_" + idx + "/distance", 20);

      agents_[i].x = 0.8 + 0.4 * static_cast<double>(i);
      agents_[i].y = 0.6 + 0.2 * static_cast<double>(i % 2);
      agents_[i].vx = 0.08 + 0.01 * static_cast<double>(i);
      agents_[i].vy = 0.05 + 0.01 * static_cast<double>(i % 2);
    }
  }

private:
  struct Agent
  {
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
  };

  static constexpr std::size_t kCount = 4;

  void onTimer()
  {
    const double dt = 0.05;
    const double min_x = 0.2;
    const double max_x = 2.8;
    const double min_y = 0.2;
    const double max_y = 1.8;

    for (std::size_t i = 0; i < kCount; ++i) {
      auto & a = agents_[i];

      a.x += a.vx * dt;
      a.y += a.vy * dt;

      if (a.x < min_x || a.x > max_x) {
        a.vx *= -1.0;
        a.x = std::clamp(a.x, min_x, max_x);
      }
      if (a.y < min_y || a.y > max_y) {
        a.vy *= -1.0;
        a.y = std::clamp(a.y, min_y, max_y);
      }

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now();
      odom.header.frame_id = "map";
      odom.child_frame_id = "sima_" + std::to_string(i + 1) + "/base_link";
      odom.pose.pose.position.x = a.x;
      odom.pose.pose.position.y = a.y;
      odom.pose.pose.orientation.w = 1.0;
      odom.twist.twist.linear.x = a.vx;
      odom.twist.twist.linear.y = a.vy;
      odom_pubs_[i]->publish(odom);

      std_msgs::msg::Float64 distance_msg;
      distance_msg.data = std::hypot(a.x, a.y);
      distance_pubs_[i]->publish(distance_msg);
    }
  }

  std::array<Agent, kCount> agents_{};
  std::array<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr, kCount> odom_pubs_{};
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kCount> distance_pubs_{};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimaSimNode>());
  rclcpp::shutdown();
  return 0;
}