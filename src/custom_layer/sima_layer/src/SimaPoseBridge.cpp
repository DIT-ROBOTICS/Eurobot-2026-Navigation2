#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SimaPoseBridge : public rclcpp::Node {
public:
  SimaPoseBridge() : Node("sima_pose_bridge") {
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    sima_ids_ = this->declare_parameter<std::vector<int64_t>>(
        "sima_ids", std::vector<int64_t>{1, 2, 3, 4});
    if (sima_ids_.empty()) {
      sima_ids_ = {1, 2, 3, 4};
    }

    prev_samples_.assign(sima_ids_.size(), PreviousSample{});
    pose_subs_.resize(sima_ids_.size());
    odom_pubs_.resize(sima_ids_.size());
    distance_pubs_.resize(sima_ids_.size());

    for (std::size_t i = 0; i < sima_ids_.size(); ++i) {
      const auto idx = std::to_string(sima_ids_[i]);
      const std::string pose_topic = "/sima_" + idx + "/pose/global";
      const std::string odom_topic = "/sima_" + idx + "/odom";
      const std::string distance_topic = "/sima_" + idx + "/distance";

      odom_pubs_[i] = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 20);
      distance_pubs_[i] =
          create_publisher<std_msgs::msg::Float64>(distance_topic, 20);
      pose_subs_[i] =
          create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              pose_topic, 20,
              [this,
               i](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                      msg) { this->poseCallback(i, msg); });
    }
  }

private:
  struct PreviousSample {
    bool valid{false};
    double x{0.0};
    double y{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  void poseCallback(
      std::size_t index,
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (index >= prev_samples_.size()) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    if (odom.header.stamp.sec == 0 && odom.header.stamp.nanosec == 0) {
      odom.header.stamp = now();
    }
    if (odom.header.frame_id.empty()) {
      odom.header.frame_id = global_frame_;
    }
    odom.child_frame_id =
        "sima_" + std::to_string(sima_ids_[index]) + "/base_link";
    odom.pose = msg->pose;

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const rclcpp::Time stamp =
        msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
            ? now()
            : rclcpp::Time(msg->header.stamp);

    double vx = 0.0;
    double vy = 0.0;
    auto &prev = prev_samples_[index];
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

  std::vector<int64_t> sima_ids_;
  std::string global_frame_;
  std::vector<PreviousSample> prev_samples_;
  std::vector<rclcpp::Subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      pose_subs_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
      distance_pubs_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimaPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
