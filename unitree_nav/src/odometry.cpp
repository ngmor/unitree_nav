//This node collects and publishes odometry from the Unitree Go1's IMU data

//This node is an example for working with the Nav2 to stack to command
//the Unitree Go1 to a certain pose in the map.

#include "rclcpp/rclcpp.hpp"



class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {

    RCLCPP_INFO_STREAM(get_logger(), "odometry node started");
  }

private:

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}