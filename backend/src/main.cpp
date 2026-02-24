#include "backend/ros_bridge.hpp"
#include "backend/config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  /* Allow selecting config file via command-line:
   *   ros2 run backend opc_bridge                          -> opcua.yaml  (real PLC)
   *   ros2 run backend opc_bridge --ros-args -p config:=opcua_test.yaml  -> test server
   */
  auto tmp_node = std::make_shared<rclcpp::Node>("_cfg_loader");
  tmp_node->declare_parameter<std::string>("config", "opcua.yaml");
  std::string config_name;
  tmp_node->get_parameter("config", config_name);
  tmp_node.reset();

  const std::string pkg_share = ament_index_cpp::get_package_share_directory("backend");
  const std::string opcua_yaml_file = pkg_share + "/config/" + config_name;

  RCLCPP_INFO(rclcpp::get_logger("main"), "Loading config: %s", opcua_yaml_file.c_str());

  UaConfig cfg = ConfigLoader::load_file(opcua_yaml_file); 

  auto node = std::make_shared<RosBridge>(cfg);
  rclcpp::executors::MultiThreadedExecutor exec;

  exec.add_node(node);
  exec.spin();

  RCLCPP_INFO(node->get_logger(), "Shutting down...");
  rclcpp::shutdown();
  return 0;
}
