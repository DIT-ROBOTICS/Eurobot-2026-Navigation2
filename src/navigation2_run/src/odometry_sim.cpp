#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // toMsg & fromMsg

double car[3] = {0};
double x = 0.0;
double y = 0.0;
double th = 0.0;

void vel_callback(const geometry_msgs::msg::Twist::SharedPtr data) {
    car[0] = data->linear.x;
    car[1] = data->linear.y;
    car[2] = data->angular.z;
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received velocity command: linear.x=%f, linear.y=%f, angular.z=%f", car[0], car[1], car[2]);
}

void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*data*/) {
    // Ignore initial pose so odom stays fixed at the configured map->odom offset.
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("odometry_publisher");

    std::string cmd_cb_name;
    node->declare_parameter("cmd_cb_name", "cmd_vel");
    node->get_parameter("cmd_cb_name", cmd_cb_name);

    // Publishers and subscribers
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("local_pose", 50);
    auto global_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("global_vel", 50);
    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_cb_name, 1000, vel_callback);
    (void)node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initial_pose", 1000, initial_pose_callback);

    tf2_ros::TransformBroadcaster odom_broadcaster(node);
    
    std::string tf_prefix_;
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.frame_id = tf_prefix_ + "map";
    map_to_odom.child_frame_id = tf_prefix_ + "odom";
    map_to_odom.transform.translation.x = 0.5;
    map_to_odom.transform.translation.y = 0.5;
    map_to_odom.transform.translation.z = 0.0;
    {
        tf2::Quaternion map_quat;
        map_quat.setRPY(0, 0, 3.1415926 / 2);
        map_to_odom.transform.rotation = tf2::toMsg(map_quat);
    }

    double vx = 0;
    double vy = 0;
    double vth = 0;

    rclcpp::Time current_time, last_time;
    current_time = node->get_clock()->now();
    last_time = node->get_clock()->now();

    rclcpp::WallRate loop_rate(20.0);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);  // Check for incoming messages
        current_time = node->get_clock()->now();

        map_to_odom.header.stamp = current_time;
        odom_broadcaster.sendTransform(map_to_odom);

        vx = car[0];
        vy = car[1];
        vth = car[2];

        // Compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).seconds();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::msg::Twist global_vel;
        global_vel.linear.x = vx;
        global_vel.linear.y = vy;
        global_vel.angular.z = vth;
        global_vel_pub->publish(global_vel);

        // Since all odometry is 6DOF, we'll need a quaternion created from yaw
        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, th);

        // First, we'll publish the transform over tf
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = tf_prefix_ + "odom";
        odom_trans.child_frame_id = tf_prefix_ + "base_footprint";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf2::toMsg(odom_quat); // tf2::Quaternion to geometry_msgs

        // Send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // Next, we'll publish the odometry message over ROS
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = tf_prefix_ + "odom";

        // Set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(odom_quat); // tf2::Quaternion to geometry_msgs

        // Set the velocity
        odom.child_frame_id = tf_prefix_ + "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // Publish the message
        odom_pub->publish(odom);

        last_time = current_time;
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
