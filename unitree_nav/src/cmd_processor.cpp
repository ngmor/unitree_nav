#include "rclcpp/rclcpp.hpp"

class CmdProcessor : public rclcpp::Node
{
public:
  CmdProcessor()
  : Node("cmd_processor")
  {

  }

private:

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdProcessor>());
  rclcpp::shutdown();
  return 0;
}