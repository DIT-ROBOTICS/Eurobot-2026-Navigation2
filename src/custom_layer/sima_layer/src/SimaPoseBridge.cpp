#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SimaPoseBridge : public rclcpp::Node {
public:
  SimaPoseBridge()
  : Node("sima_pose_bridge")
  {
    for (std::size_t i = 0; i < kSimaCount; ++i) {
      const auto idx = std::to_string(i + 1);
      const std::string pose_topic = "/sima_" + idx + "/pose/global";
      const std::string odom_topic = "/sima_" + idx + "/odom";
      const std::string distance_topic = "/sima_" + idx + "/distance";

      odom_pubs_[i] = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 20);
      distance_pubs_[i] = create_publisher<std_msgs::msg::Float64>(distance_topic, 20);
      pose_subs_[i] = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, 20,
        [this, i](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          this->poseCallback(i, msg);
        });
    }
  }

private:
  struct PreviousSample {
    bool valid{false};
    double x{0.0};
    double y{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  static constexpr std::size_t kSimaCount = 4;

  void poseCallback(std::size_t index, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.child_frame_id = "sima_" + std::to_string(index + 1) + "/base_link";
    odom.pose = msg->pose;

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ? now() : rclcpp::Time(msg->header.stamp);

    double vx = 0.0;
    double vy = 0.0;
    auto & prev = prev_samples_[index];
    if (prev.valid) {
      const double dt = (stamp - prev.stamp).seconds();
      if (dt > 1e-3) {
        vx = (x - prev.x) / dt;
        vy = (y - prev.y) / dt;
      }
    }

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;

    prev.valid = true;
    prev.x = x;
    prev.y = y;
    prev.stamp = stamp;

    odom_pubs_[index]->publish(odom);

    std_msgs::msg::Float64 distance_msg;
    distance_msg.data = std::hypot(x, y);
    distance_pubs_[index]->publish(distance_msg);
  }

  std::array<PreviousSample, kSimaCount> prev_samples_{};
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr, kSimaCount> pose_subs_{};
  std::array<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr, kSimaCount> odom_pubs_{};
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kSimaCount> distance_pubs_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimaPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
