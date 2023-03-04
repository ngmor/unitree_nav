//This node is an example for working with the Nav2 to stack to command
//the Unitree Go1 to a certain pose in the map.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_nav_interfaces/srv/nav_to_pose.hpp"

//https://stackoverflow.com/questions/11714325/how-to-get-enum-item-name-from-its-value
#define STATES \
X(IDLE, "IDLE") \
X(SEND_GOAL, "SEND_GOAL")

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

    //Services
    srv_nav_to_pose_ = create_service<unitree_nav_interfaces::srv::NavToPose>(
      "unitree_nav_to_pose",
      std::bind(&NavToPose::srv_nav_to_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2)
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
  rclcpp::Service<unitree_nav_interfaces::srv::NavToPose>::SharedPtr srv_nav_to_pose_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr act_nav_to_pose_;

  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds
  State state_ = State::IDLE;
  State state_last_ = state_;
  State state_next_ = state_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal goal_msg_ {};

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
        break;
      }
      case State::SEND_GOAL:
      {
        break;
      }
      default:
        auto msg = "Unhandled state: " + get_state_name(state_);
        RCLCPP_ERROR_STREAM(get_logger(), msg);
        throw std::logic_error(msg);
        break;
    }

  }

  void srv_nav_to_pose_callback(
    const std::shared_ptr<unitree_nav_interfaces::srv::NavToPose::Request> request,
    std::shared_ptr<unitree_nav_interfaces::srv::NavToPose::Response>
  ) {
    //Store requested pose
    goal_msg_.pose.pose = request->pose;

    //Initiate action call
    state_next_ = State::SEND_GOAL;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPose>());
  rclcpp::shutdown();
  return 0;
}