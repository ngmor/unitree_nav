//This node is an example for working with the Nav2 stack to command
//the Unitree Go1 to a certain pose in the map.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_nav_interfaces/srv/nav_to_pose.hpp"

//https://stackoverflow.com/questions/11714325/how-to-get-enum-item-name-from-its-value
#define STATES \
X(IDLE, "IDLE") \
X(SEND_GOAL, "SEND_GOAL") \
X(WAIT_FOR_GOAL_RESPONSE, "WAIT_FOR_GOAL_RESPONSE") \
X(WAIT_FOR_MOVEMENT_COMPLETE, "WAIT_FOR_MOVEMENT_COMPLETE")

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

std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q);
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);

using namespace std::chrono_literals;

class NavToPose : public rclcpp::Node
{
public:
  NavToPose()
  : Node("nav_to_pose")
  {

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The frame in which poses are sent.";
    declare_parameter("pose_frame", "map", param);
    goal_msg_.pose.header.frame_id = get_parameter("pose_frame").get_parameter_value().get<std::string>();

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

    srv_cancel_nav_ = create_service<std_srvs::srv::Empty>(
      "unitree_cancel_nav",
      std::bind(&NavToPose::cancel_nav_callback, this,
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
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_cancel_nav_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr act_nav_to_pose_;

  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds
  State state_ = State::IDLE;
  State state_last_ = state_;
  State state_next_ = state_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal goal_msg_ {};
  bool goal_response_received_ = false;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ {};
  std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ = nullptr;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>
    result_ = nullptr;
  

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
        if(act_nav_to_pose_->wait_for_action_server(0s)) {
          //Reset status flags and pointers
          goal_response_received_ = false;
          goal_handle_ = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr {};
          result_ = nullptr;

          //Construct and send goal
          auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
          send_goal_options.goal_response_callback = 
            std::bind(&NavToPose::goal_response_callback, this, std::placeholders::_1);
          send_goal_options.feedback_callback =
            std::bind(&NavToPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
          send_goal_options.result_callback =
            std::bind(&NavToPose::result_callback, this, std::placeholders::_1);
          act_nav_to_pose_->async_send_goal(goal_msg_, send_goal_options);

          state_next_ = State::WAIT_FOR_GOAL_RESPONSE;
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Action server not available, aborting.");
          state_next_ = State::IDLE;
        }

        break;
      }
      case State::WAIT_FOR_GOAL_RESPONSE:
      {
        //TODO add timeout
        if (goal_response_received_) {
          if (goal_handle_) {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            state_next_ = State::WAIT_FOR_MOVEMENT_COMPLETE;
          } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was rejected by server");
            state_next_ = State::IDLE;
          }
        }

        break;
      }
      case State::WAIT_FOR_MOVEMENT_COMPLETE:
      {
        if (result_) {
          state_next_ = State::IDLE;
        }
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
    goal_msg_.pose.pose.position.x = request->x;
    goal_msg_.pose.pose.position.y = request->y;
    goal_msg_.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, request->theta);

    //Initiate action call
    state_next_ = State::SEND_GOAL;
  }

  void cancel_nav_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    RCLCPP_INFO_STREAM(get_logger(), "Cancelling navigation.");
    act_nav_to_pose_->async_cancel_all_goals();
    state_next_ = State::IDLE;
  }

  void goal_response_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle
  ) {
    goal_response_received_ = true;
    goal_handle_ = goal_handle;
    RCLCPP_INFO_STREAM(get_logger(), "Goal response");
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
  ) {
    //Store result for later use
    feedback_ = feedback;

    if (feedback_) {
      auto [roll, pitch, yaw] = quaternion_to_rpy(feedback_->current_pose.pose.orientation);

      RCLCPP_INFO_STREAM(get_logger(), "x = " << feedback_->current_pose.pose.position.x
                                  << ", y = " << feedback_->current_pose.pose.position.y
                                  << ", theta = " << yaw
      );
    }
  }

  void result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result
  ) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    //Store result for later use
    result_ = std::make_shared<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>();
    *result_ = result;
  }
};

std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q) {
  //https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
  tf2::Quaternion q_temp;
  tf2::fromMsg(q, q_temp);
  tf2::Matrix3x3 m(q_temp);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPose>());
  rclcpp::shutdown();
  return 0;
}