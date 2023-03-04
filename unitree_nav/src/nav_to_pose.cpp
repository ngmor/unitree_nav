//This node is an example for working with the Nav2 to stack to command
//the Unitree Go1 to a certain pose in the map.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

//https://stackoverflow.com/questions/11714325/how-to-get-enum-item-name-from-its-value
#define STATES \
X(IDLE, "IDLE") \
X(RUNNING, "RUNNING")

#define X(state, name) state,
enum class State : size_t {STATES};
#undef X

#define X(state, name) name,
std::vector<std::string> STATE_NAMES = {STATES};
#undef X

//https://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
template <typename Enumeration>
auto to_value(Enumeration const value)
  -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

auto get_state_name(State state) {
  return STATE_NAMES[to_value(state)];
}

using namespace std::chrono_literals;

class NavToPose : public rclcpp::Node
{
public:
  NavToPose()
  : Node("nav_to_pose")
  {

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)), 
      std::bind(&NavToPose::timer_callback, this)
    );

    //Action Clients
    act_nav_to_pose_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose"
    );

    RCLCPP_INFO_STREAM(get_logger(), "nav_to_pose node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr act_nav_to_pose_;

  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds
  State state_ = State::IDLE;
  State state_last_ = state_;
  State state_next_ = state_;

  void timer_callback()
  {
    
    state_ = state_next_;

    auto new_state = state_ != state_last_;

    if (new_state) {
      RCLCPP_INFO_STREAM(get_logger(), "nav_to_pose state changed to " << get_state_name(state_));

      state_last_ = state_;
    }

    switch(state_) {
      case State::IDLE:
      {
        state_next_ = State::RUNNING;
        break;
      }
      case State::RUNNING:
      {
        state_next_ = State::IDLE;
        break;
      }
      default:
        auto msg = "Unhandled state: " + get_state_name(state_);
        // RCLCPP_ERROR_STREAM(get_logger(), "Unhandled state: " << STATE_NAMES[state_]);
        // throw std::logic_error("Unhandle")
        break;
    }

  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPose>());
  rclcpp::shutdown();
  return 0;
}