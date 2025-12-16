#include "bt_water/bt_executor_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <thread>

namespace bt_water
{

BTExecutorNode::BTExecutorNode(const rclcpp::NodeOptions & options)
: Node("bt_executor_node", options),
  tick_rate_(10.0),
  cycle_completed_(false)
{
  // Declare parameters
  this->declare_parameter("tick_rate", 10.0);
  this->declare_parameter("tree_file", std::string("trees/water_cycle.xml"));

  // Get parameters
  tick_rate_ = this->get_parameter("tick_rate").as_double();
  tree_file_ = this->get_parameter("tree_file").as_string();

  RCLCPP_INFO(this->get_logger(), "BT Water Executor Node starting...");
  RCLCPP_INFO(this->get_logger(), "Tick rate: %.2f Hz", tick_rate_);
  RCLCPP_INFO(this->get_logger(), "Tree file parameter: %s", tree_file_.c_str());

  // ============================================================================
  // TODO: ADD FESTO NODES HERE
  // ============================================================================
  // Replace the placeholder nodes below with your Festo node implementations.
  //
  // Steps to add Festo nodes:
  // 1. Create Festo node classes (e.g., in include/bt_water/festo_nodes/ and src/festo_nodes/)
  //    - StartDripNode: Activates FESTO IO drip valve
  //    - StopDripNode: Deactivates FESTO IO drip valve
  //    - StartDrainNode: Activates FESTO IO drain valve
  //    - StopDrainNode: Deactivates FESTO IO drain valve
  //
  // 2. Include the Festo node headers here (add #include statements at top of file)
  //    Example: #include "bt_water/festo_nodes/start_drip_node.hpp"
  //
  // 3. Replace the placeholder registrations below with your Festo node classes:
  //    Example: factory_.registerNodeType<StartDripNode>("StartDrip");
  //
  // 4. For sensor nodes (WaitDripComplete, WaitDrainComplete), implement sensor
  //    monitoring nodes that subscribe to water level topics or services.
  // ============================================================================

  // Register placeholder action nodes (REPLACE THESE WITH FESTO NODES)
  factory_.registerNodeType<BT::AlwaysSuccessNode>("StartDrip");        // TODO: Replace with Festo StartDripNode
  factory_.registerNodeType<BT::AlwaysSuccessNode>("WaitDripComplete");  // TODO: Replace with water level sensor node
  factory_.registerNodeType<BT::AlwaysSuccessNode>("StopDrip");         // TODO: Replace with Festo StopDripNode
  factory_.registerNodeType<BT::AlwaysSuccessNode>("StartDrain");       // TODO: Replace with Festo StartDrainNode
  factory_.registerNodeType<BT::AlwaysSuccessNode>("WaitDrainComplete");// TODO: Replace with water level sensor node
  factory_.registerNodeType<BT::AlwaysSuccessNode>("StopDrain");        // TODO: Replace with Festo StopDrainNode

  RCLCPP_INFO(this->get_logger(), "Registered placeholder action nodes");

  // Load and execute behavior tree
  loadBehaviorTree();
  executeBehaviorTree();
}

void BTExecutorNode::loadBehaviorTree()
{
  std::string resolved_path = resolveTreeFilePath(tree_file_);
  RCLCPP_INFO(this->get_logger(), "Loading behavior tree from: %s", resolved_path.c_str());

  if (!std::filesystem::exists(resolved_path)) {
    RCLCPP_ERROR(this->get_logger(), "Tree file not found: %s", resolved_path.c_str());
    throw std::runtime_error("Tree file not found: " + resolved_path);
  }

  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(resolved_path));
  RCLCPP_INFO(this->get_logger(), "Behavior tree loaded successfully");
}

void BTExecutorNode::executeBehaviorTree()
{
  if (!tree_) {
    RCLCPP_ERROR(this->get_logger(), "Behavior tree not loaded");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Starting water cycle execution...");

  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / tick_rate_));
  auto last_tick = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !cycle_completed_) {
    rclcpp::spin_some(this->get_node_base_interface());

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick);

    if (elapsed >= period) {
      BT::NodeStatus status = tree_->tickOnce();

      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Water cycle completed successfully");
        cycle_completed_ = true;
        break;
      } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(this->get_logger(), "Water cycle failed");
        cycle_completed_ = true;
        break;
      }

      last_tick = now;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (cycle_completed_) {
    RCLCPP_INFO(this->get_logger(), "Water cycle execution finished. Node will remain alive.");
  }
}

std::string BTExecutorNode::resolveTreeFilePath(const std::string & tree_file_param)
{
  // If absolute path, use as-is
  if (tree_file_param.find("/") == 0) {
    return tree_file_param;
  }

  // If starts with ~/, expand to home directory
  if (tree_file_param.find("~/") == 0) {
    const char* home = getenv("HOME");
    if (home) {
      return std::string(home) + tree_file_param.substr(1);
    }
  }

  // Relative path: resolve to source package directory
  try {
    std::string package_share = ament_index_cpp::get_package_share_directory("bt_water");
    std::filesystem::path share_path(package_share);
    
    // Navigate from install/share/bt_water to src/bt_water
    // install/share/bt_water -> install -> build -> src -> src/bt_water
    std::filesystem::path source_path = share_path.parent_path().parent_path().parent_path() / "src" / "bt_water";
    
    // If source path doesn't exist, try using share directory (for installed packages)
    if (!std::filesystem::exists(source_path)) {
      source_path = share_path;
    }
    
    std::filesystem::path tree_path = source_path / tree_file_param;
    
    if (std::filesystem::exists(tree_path)) {
      return tree_path.string();
    }
    
    // Fallback: try share directory
    tree_path = share_path / tree_file_param;
    if (std::filesystem::exists(tree_path)) {
      return tree_path.string();
    }
    
    RCLCPP_WARN(this->get_logger(), "Tree file not found in source or share, using: %s", tree_path.string().c_str());
    return tree_path.string();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error resolving tree file path: %s", e.what());
    return tree_file_param;
  }
}

}  // namespace bt_water

