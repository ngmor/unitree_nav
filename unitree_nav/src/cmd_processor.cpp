#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CmdProcessor : public rclcpp::Node
{
public:
  CmdProcessor()
  : Node("cmd_processor")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The rate at which the node sends commands (Hz).";
    declare_parameter("rate", 200.0, param);
    rate_ = get_parameter("rate").get_parameter_value().get<double>();
    interval_ = 1.0 / rate_;
    interval_ms_ = static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)),

    param.description = "The amount of time (ms) for which a received cmd_vel will be published.";
    declare_parameter("cmd_vel_timeout", 500, param);
    cmd_vel_timeout_ = static_cast<std::chrono::milliseconds>(
      get_parameter("cmd_vel_timeout").get_parameter_value().get<int>()
    );

    //Timers
    timer_ = create_wall_timer(interval_ms_, std::bind(&CmdProcessor::timer_callback, this));

    //Subscribers
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&CmdProcessor::cmd_vel_callback, this, std::placeholders::_1)
    );

    //Misc variables
    cmd_vel_counter_ = cmd_vel_timeout_; // init to timeout value so we don't publish


    RCLCPP_INFO_STREAM(get_logger(), "cmd_processor node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  
  double rate_, interval_;
  std::chrono::milliseconds interval_ms_, cmd_vel_timeout_, cmd_vel_counter_;

  void timer_callback()
  {

    //Only publish if we haven't timed out
    if (cmd_vel_counter_ < cmd_vel_timeout_) {
      //TODO publish
      
      //TODO remove
      RCLCPP_INFO_STREAM(get_logger(), "Time: " << cmd_vel_counter_.count());

      cmd_vel_counter_ += interval_ms_;
    }

  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {

    //reset timeout
    cmd_vel_counter_ = 0ms;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdProcessor>());
  rclcpp::shutdown();
  return 0;
}