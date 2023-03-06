//This node collects and publishes odometry from the Unitree Go1's IMU data

//This node is an example for working with the Nav2 to stack to command
//the Unitree Go1 to a certain pose in the map.

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"


class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    param.description = "The name of the robot's body frame.";
    declare_parameter("body_id", "base_link", param);
    auto body_id = get_parameter("body_id").get_parameter_value().get<std::string>();

    param.description = "The name of the robot's odometry frame.";
    declare_parameter("odom_id", "odom", param);
    auto odom_id = get_parameter("odom_id").get_parameter_value().get<std::string>();

     //Publishers
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 0);

    //Subscribers
    sub_state_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighState>(
      "high_state",
      10,
      std::bind(&Odometry::state_callback, this, std::placeholders::_1)
    );

    //Broadcasters
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Init odom msg/tf
    odom_msg_.header.frame_id = odom_id;
    odom_msg_.child_frame_id = body_id;
    odom_msg_.pose.covariance = std::array<double, 36> {};
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.covariance = std::array<double, 36> {};

    for (int i = 0; i < 36; i += 6) {
      odom_msg_.pose.covariance[i] = 1.0e-6;
      odom_msg_.twist.covariance[i] = 1.0e-6;
    }

    //Init odom transform
    odom_tf_.header.frame_id = odom_id;
    odom_tf_.child_frame_id = body_id;

    RCLCPP_INFO_STREAM(get_logger(), "odometry node started");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr sub_state_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped odom_tf_;

  void state_callback(const ros2_unitree_legged_msgs::msg::HighState & state)
  {
    // build odometry message
    odom_msg_.pose.pose.position.x = state.position[0];
    odom_msg_.pose.pose.position.y = state.position[1];
    odom_msg_.pose.pose.position.z = state.position[2];

    odom_msg_.pose.pose.orientation.x = state.imu.quaternion[0];
    odom_msg_.pose.pose.orientation.y = state.imu.quaternion[1];
    odom_msg_.pose.pose.orientation.z = state.imu.quaternion[2];
    odom_msg_.pose.pose.orientation.w = state.imu.quaternion[3];

    //Would be better if we could synchronize this stamp with the received message
    odom_msg_.header.stamp = get_clock()->now();

    odom_msg_.twist.twist.angular.z = state.yaw_speed;
    odom_msg_.twist.twist.linear.x = state.velocity[0];
    odom_msg_.twist.twist.linear.y = state.velocity[1];
    odom_msg_.twist.twist.linear.z = state.velocity[2];

    //Build transform
    odom_tf_.transform.translation.x = odom_msg_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_msg_.pose.pose.position.y;
    odom_tf_.transform.translation.z = odom_msg_.pose.pose.position.z;
    odom_tf_.transform.rotation = odom_msg_.pose.pose.orientation;
    odom_tf_.header.stamp = odom_msg_.header.stamp;

    //Publish odometry message
    pub_odom_->publish(odom_msg_);

    //Broadcast transform
    broadcaster_->sendTransform(odom_tf_);
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}