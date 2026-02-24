#include "backend/ros_bridge.hpp"
#include "backend/config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  const std::string& opcua_yaml_file = ament_index_cpp::get_package_share_directory("backend") + "/config/opcua.yaml";

  UaConfig cfg = ConfigLoader::load_file(opcua_yaml_file); 

  auto node = std::make_shared<RosBridge>(cfg);
  rclcpp::executors::MultiThreadedExecutor exec;

  exec.add_node(node);
  exec.spin();

  RCLCPP_INFO(node->get_logger(), "Shutting down...");
  rclcpp::shutdown();
  return 0;
}
