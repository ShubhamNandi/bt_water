#ifndef BT_WATER__BT_EXECUTOR_NODE_HPP_
#define BT_WATER__BT_EXECUTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <string>

namespace bt_water
{

class BTExecutorNode : public rclcpp::Node
{
public:
  explicit BTExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadBehaviorTree();
  void executeBehaviorTree();
  std::string resolveTreeFilePath(const std::string & tree_file_param);

  double tick_rate_;
  std::string tree_file_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool cycle_completed_;
};

}  // namespace bt_water

#endif  // BT_WATER__BT_EXECUTOR_NODE_HPP_

