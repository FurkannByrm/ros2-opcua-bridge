#include "opcua_to_ros2/magician_demonstrator.hpp"
#include "opcua_to_ros2/config.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto tmp_node = std::make_shared<rclcpp::Node>("_cfg_loader");
  tmp_node->declare_parameter<std::string>("config", "opcua.yaml");
  std::string config_name;
  tmp_node->get_parameter("config", config_name);
  tmp_node.reset();

  const std::string pkg_share = ament_index_cpp::get_package_share_directory("opcua_to_ros2");
  const std::string opcua_yaml_file = pkg_share + "/config/" + config_name;

  RCLCPP_INFO(rclcpp::get_logger("main"), "Loading config: %s", opcua_yaml_file.c_str());

  UaConfig cfg = ConfigLoader::load_file(opcua_yaml_file); 

  auto opc_srv = std::make_shared<OPCuaBridge>(cfg);
  auto common_node = std::make_shared<CommonDemonstratorInterface>(opc_srv);
  auto sensing_node = std::make_shared<Sensing>(opc_srv);
  auto cleaning_node = std::make_shared<Cleaning>(opc_srv);

  rclcpp::executors::MultiThreadedExecutor exec;

  exec.add_node(common_node);
  exec.add_node(sensing_node);
  exec.add_node(cleaning_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
