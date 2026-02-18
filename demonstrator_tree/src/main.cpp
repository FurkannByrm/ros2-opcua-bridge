#include "demonstrator_tree/behavior_node.hpp"

int main(int argc, char **argv){

auto cfg = ConfigLoader::load_file("/home/furkan/magician_ws/src/demonstrator_tree/config/parameters.yaml");
rclcpp::init(argc,argv);    
auto subNode = std::make_shared<DemostratorTree::MagicianSubNode>("home_check_node",cfg);
auto clientHoming = std::make_shared<DemostratorTree::MagicianClientNode>("homing_node",cfg);
auto clientOpcUa  = std::make_shared<DemostratorTree::MagicianOpcUA>("opcua_safe_transfer_node");

rclcpp::executors::MultiThreadedExecutor exe;
exe.add_node(subNode);
exe.add_node(clientHoming);
exe.add_node(clientOpcUa);

bool initialization = false;

auto timer = subNode->create_wall_timer(
    std::chrono::seconds(4),
    [&](){
        initialization = true;
    });

while (!initialization && rclcpp::ok()){
    exe.spin_some();
    std::this_thread::sleep_for(std::chrono::seconds(10));
}

timer->cancel();

BT::BehaviorTreeFactory factory;
factory.registerSimpleAction("IsRobotAtHome", [&](BT::TreeNode&){return subNode->checkPos();});
factory.registerSimpleAction("CallHoming",[&](BT::TreeNode&){return clientHoming->homingCall();});
factory.registerSimpleAction("CallOpcUI",[&](BT::TreeNode&){return clientOpcUa->OpcServiceCall();});
auto tree =factory.createTreeFromFile("/home/cengo/ros_ws/src/demonstrator_tree/config/bt_tree.xml");
tree.tickWhileRunning();

rclcpp::shutdown();


return 0;
}
