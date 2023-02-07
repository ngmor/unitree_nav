#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"

using namespace std::chrono_literals;

//TODO move into a shared library
enum class Go1Mode : uint8_t
{
   idle = 0
  ,force_stand = 1
  ,target_velocity_walking = 2
  ,target_position_walking = 3
  ,path_walking = 4
  ,position_stand_down = 5
  ,position_stand_up = 6
  ,damping = 7
  ,recovery_stand = 8
  ,backflip = 9
  ,jump_yaw = 10
  ,straight_hand = 11
  ,dance1 = 12
  ,dance2 = 13
};

enum class Go1Gait : uint8_t
{
   idle = 0
  ,trot = 1
  ,trot_running = 2
  ,climb_stairs = 3
  ,trot_obstacle = 4
};

enum class Go1SpeedLevel : uint8_t
{
   low = 0
  ,medium = 1
  ,high = 2
};

//https://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
template <typename Enumeration>
auto to_value(Enumeration const value)
    -> typename std::underlying_type<Enumeration>::type
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

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
    declare_parameter("cmd_vel_timeout", 250, param);
    cmd_vel_timeout_ = static_cast<std::chrono::milliseconds>(
      get_parameter("cmd_vel_timeout").get_parameter_value().get<int>()
    );

    //Timers
    timer_ = create_wall_timer(interval_ms_, std::bind(&CmdProcessor::timer_callback, this));

    //Publishers
    pub_high_cmd_ = create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);

    //Subscribers
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&CmdProcessor::cmd_vel_callback, this, std::placeholders::_1)
    );

    //Services
    srv_stand_up_ = create_service<std_srvs::srv::Empty>(
      "stand_up",
      std::bind(&CmdProcessor::stand_up_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    srv_lay_down_ = create_service<std_srvs::srv::Empty>(
      "lay_down",
      std::bind(&CmdProcessor::lay_down_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    //Misc variables
    cmd_vel_counter_ = cmd_vel_timeout_; // init to timeout value so we don't publish

    //Default command values
    high_cmd_.head[0] = 0xFE; //??
    high_cmd_.head[1] = 0xEF; //??
    high_cmd_.level_flag = 0xEE; //from unitree_legged_sdk, HIGHLEVEL TODO - move to UDP node?
    high_cmd_.mode = to_value(Go1Mode::idle);
    high_cmd_.gait_type = to_value(Go1Gait::idle);
    high_cmd_.speed_level = to_value(Go1SpeedLevel::low);
    high_cmd_.velocity[0] = 0.0;
    high_cmd_.velocity[1] = 0.0;
    high_cmd_.yaw_speed = 0.0;

    RCLCPP_INFO_STREAM(get_logger(), "cmd_processor node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub_high_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stand_up_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_lay_down_;

  double rate_, interval_;
  std::chrono::milliseconds interval_ms_, cmd_vel_timeout_, cmd_vel_counter_;
  geometry_msgs::msg::Twist cmd_vel_ {};
  ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ {};
  

  void timer_callback()
  {

    //Only publish if we haven't timed out
    if (cmd_vel_counter_ < cmd_vel_timeout_) {
      //Increment counter by timer interval
      cmd_vel_counter_ += interval_ms_;

    } else { //cmd_vel has timed out
      //reset to 0 twist
      reset_cmd_vel();
    }

    //Build high command
    high_cmd_.velocity[0] = cmd_vel_.linear.x;
    high_cmd_.velocity[1] = cmd_vel_.linear.y;
    high_cmd_.yaw_speed = cmd_vel_.angular.z;

    //publish high command
    pub_high_cmd_->publish(high_cmd_);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    //Store received cmd_vel
    cmd_vel_ = msg;

    //reset timeout
    cmd_vel_counter_ = 0ms;

    //set mode and gait type
    high_cmd_.mode = to_value(Go1Mode::target_velocity_walking);
    high_cmd_.gait_type = to_value(Go1Gait::trot);
  }

  void reset_cmd_vel() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }

  //put the dog in the stand up state
  void stand_up_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::position_stand_up);
  }

  //put the dog in the stand down state
  void lay_down_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::position_stand_down);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdProcessor>());
  rclcpp::shutdown();
  return 0;
}