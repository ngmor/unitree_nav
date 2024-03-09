#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"
#include "unitree_nav_interfaces/srv/set_gait.hpp"

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
    interval_ms_ = static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0));

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
    srv_reset_state_ = create_service<std_srvs::srv::Empty>(
      "reset_state",
      std::bind(&CmdProcessor::reset_state_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_stand_up_ = create_service<std_srvs::srv::Empty>(
      "stand_up",
      std::bind(&CmdProcessor::stand_up_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_recover_stand_ = create_service<std_srvs::srv::Empty>(
      "recover_stand",
      std::bind(&CmdProcessor::recover_stand_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_lay_down_ = create_service<std_srvs::srv::Empty>(
      "lay_down",
      std::bind(&CmdProcessor::lay_down_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_damping_ = create_service<std_srvs::srv::Empty>(
      "damping",
      std::bind(&CmdProcessor::damping_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_jump_yaw_ = create_service<std_srvs::srv::Empty>(
      "jump_yaw",
      std::bind(&CmdProcessor::jump_yaw_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_beg_ = create_service<std_srvs::srv::Empty>(
      "beg",
      std::bind(&CmdProcessor::beg_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_dance1_ = create_service<std_srvs::srv::Empty>(
      "dance1",
      std::bind(&CmdProcessor::dance1_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_dance2_ = create_service<std_srvs::srv::Empty>(
      "dance2",
      std::bind(&CmdProcessor::dance2_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_set_body_rpy_ = create_service<unitree_nav_interfaces::srv::SetBodyRPY>(
      "set_body_rpy",
      std::bind(&CmdProcessor::set_body_rpy_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_set_gait_ = create_service<unitree_nav_interfaces::srv::SetGait>(
      "set_gait",
      std::bind(&CmdProcessor::set_gait_callback, this,
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

    // default walking gait is trotting
    walking_gait_ = Go1Gait::trot;

    RCLCPP_INFO_STREAM(get_logger(), "cmd_processor node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub_high_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_state_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stand_up_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_recover_stand_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_lay_down_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_damping_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_jump_yaw_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_beg_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_dance1_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_dance2_;
  rclcpp::Service<unitree_nav_interfaces::srv::SetBodyRPY>::SharedPtr srv_set_body_rpy_;
  rclcpp::Service<unitree_nav_interfaces::srv::SetGait>::SharedPtr srv_set_gait_;

  double rate_, interval_;
  std::chrono::milliseconds interval_ms_, cmd_vel_timeout_, cmd_vel_counter_;
  geometry_msgs::msg::Twist cmd_vel_ {};
  ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ {};
  bool reset_state_ = false;
  uint8_t reset_counter_ = 0;
  Go1Gait walking_gait_;

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

    //Reset state to idle after one message is sent for anything but walking
    if (reset_state_) {
      if (reset_counter_ >= 10) {
        high_cmd_.mode = to_value(Go1Mode::idle);
        reset_state_ = false;
      }
      reset_counter_++;
    }
  }

  void reset_state() {
    reset_state_ = true;
    reset_counter_ = 0;
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    //Store received cmd_vel
    cmd_vel_ = msg;

    //reset timeout
    cmd_vel_counter_ = 0ms;

    //set mode and gait type
    high_cmd_.mode = to_value(Go1Mode::target_velocity_walking);
    high_cmd_.gait_type = to_value(walking_gait_);
  }

  void reset_cmd_vel() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }

  //put the dog back into idle state
  void reset_state_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    reset_state(); //Reset state after command is sent
  }

  //put the dog in the stand up state
  void stand_up_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::position_stand_up);
    reset_state(); //Reset state after command is sent
  }

  //put the dog in the recover stand state
  void recover_stand_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::recovery_stand);
    reset_state(); //Reset state after command is sent
  }

  //To lay down completely, call lay down service and then damping service

  //put the dog in the stand down state
  void lay_down_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::position_stand_down);
    reset_state(); //Reset state after command is sent
  }

  //put the dog in the damping state
  // DO NOT CALL WHILE DOG IS IN STANDING STATE
  void damping_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::damping);
    reset_state(); //Reset state after command is sent
  }

  //put the dog into jump_yaw state
  void jump_yaw_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::jump_yaw);
    reset_state(); //Reset state after command is sent
  }

  //put the dog into straight_hand state
  void beg_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::straight_hand);
    reset_state(); //Reset state after command is sent
  }

  //put the dog into dance1 state
  void dance1_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::dance1);
    reset_state(); //Reset state after command is sent
  }

  //put the dog into dance2 state
  void dance2_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    reset_cmd_vel();
    high_cmd_.mode = to_value(Go1Mode::dance2);
    reset_state(); //Reset state after command is sent
  }

  void set_body_rpy_callback(
    const std::shared_ptr<unitree_nav_interfaces::srv::SetBodyRPY::Request> request,
    std::shared_ptr<unitree_nav_interfaces::srv::SetBodyRPY::Response>
  ) {
    reset_cmd_vel();

    high_cmd_.euler[0] = request->roll;
    high_cmd_.euler[1] = request->pitch;
    high_cmd_.euler[2] = request->yaw;

    high_cmd_.mode = to_value(Go1Mode::force_stand);
  }

  void set_gait_callback(
    const std::shared_ptr<unitree_nav_interfaces::srv::SetGait::Request> request,
    std::shared_ptr<unitree_nav_interfaces::srv::SetGait::Response>
  ) {
    // check that we are not setting an invalid gait type
    if (request->gait <= 4){
      walking_gait_ = static_cast<Go1Gait>(request->gait);
    }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdProcessor>());
  rclcpp::shutdown();
  return 0;
}