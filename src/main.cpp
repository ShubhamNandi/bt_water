#include "bt_water/bt_executor_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<bt_water::BTExecutorNode>(options);
  
  // Keep node alive after BT completes
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

