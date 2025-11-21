#include "backend/ros_bridge.hpp"
#include "backend/config.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  UaConfig cfg = ConfigLoader::load_file("/home/plot/ros2-opcua-bridge/src/backend/config/opcua.yaml");

  auto node = std::make_shared<RosBridge>(cfg);
  rclcpp::executors::MultiThreadedExecutor exec;

  exec.add_node(node);
  exec.spin();

  RCLCPP_INFO(node->get_logger(), "Shutting down...");
  rclcpp::shutdown();
  return 0;
}
