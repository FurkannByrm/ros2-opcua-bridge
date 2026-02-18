#include "demonstrator_tree/behavior_node.hpp"

using namespace DemostratorTree;


MagicianSubNode::MagicianSubNode(const std::string& node_name, const CobotConfig& cfg) 
: rclcpp::Node{node_name}, cfg_{cfg}{
    auto robot1 = cfg_.sensing_group.name;
    auto robot2 = cfg_.cleaning_group.name; 
    robot_home_status_.try_emplace(robot1,false,cfg_.sensing_group.sensing_home_vec);
    robot_home_status_.try_emplace(robot2,false,cfg_.cleaning_group.cleaning_home_vec);

    sensing_home_axis_pos_ = create_subscription<sensor_msgs::msg::JointState>
    (cfg_.sensing_group.sensing_joint_states,10,
        [this,robot1](const sensor_msgs::msg::JointState::ConstSharedPtr& msg){
            homePosCallback(msg,robot1);
        });
    cleaning_home_axis_pos_= create_subscription<sensor_msgs::msg::JointState>
    (cfg_.cleaning_group.cleaning_joint_states,10,
            [this, robot2](const sensor_msgs::msg::JointState::ConstSharedPtr& msg){
            homePosCallback(msg,robot2);
        });
    RCLCPP_INFO(get_logger(), " Magician Subscribers initialized");
}


void MagicianSubNode::homePosCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg, const std::string& robot_id){

    auto home_pos_vec   = robot_home_status_[robot_id].second; 

    robot_home_status_[robot_id].first = true;
    
    for(size_t i = 0; i<home_pos_vec.size(); i++)
    {   
            auto target = home_pos_vec[i];       
            auto current = msg->position[i];
            
            if(std::fabs(target - current) > JOINT_TOL){
                
                robot_home_status_[robot_id].first = false;
                break;
            }
        }
        
      if (robot_home_status_[robot_id].first){
        RCLCPP_INFO(get_logger(), "%s - ROBOT HOME POSITION", robot_id.c_str());
    }
    else
    {
        RCLCPP_INFO(get_logger(), "%s - Robot NOT home", robot_id.c_str());
    }
    

}

MagicianClientNode::MagicianClientNode(const std::string& node_name, const CobotConfig& cfg) 
: rclcpp::Node{node_name}, cfg_{cfg}{

    sensing_client_ = this->create_client<std_srvs::srv::SetBool>(cfg_.sensing_group.sensing_service_name);
    cleaning_client_= this->create_client<std_srvs::srv::SetBool>(cfg_.cleaning_group.cleaning_service_name);

    while(!sensing_client_->wait_for_service(std::chrono::microseconds(4))){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(get_logger(),"Client interrupted while waiting for sensing service to appear ");
            assert(0);
        }
           
        RCLCPP_INFO(get_logger(),"Waiting for sensing service to appear");
    }

    RCLCPP_INFO(get_logger(),"sensing and cleaning client created..");

} 


BT::NodeStatus MagicianClientNode::homingCall(){

    auto sensing_at_home = MagicianSubNode::sensingStateGet();
    auto cleaning_at_home = MagicianSubNode::cleaningStateGet();

    auto sensing_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    sensing_request->data = true;
    
    auto cleaning_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    cleaning_request->data = true;

    const auto timeout = std::chrono::seconds(5);

    //Only sensing needs homing
    if(!sensing_at_home && cleaning_at_home){
        return sendHomingRequest(sensing_client_, sensing_request, timeout, "SENSING");
    } 

    //Only cleaning needs homing
    if(sensing_at_home && !cleaning_at_home){
        return sendHomingRequest(cleaning_client_, cleaning_request, timeout, "CLEANING");
    }

    //both need homing
    auto sensing_future = sensing_client_->async_send_request(sensing_request);
    auto cleaning_future = cleaning_client_->async_send_request(cleaning_request);

    //wait for both
    bool sensing_ready = sensing_future.wait_for(timeout) == std::future_status::ready;
    bool cleaning_ready= cleaning_future.wait_for(timeout) == std::future_status::ready;
    
    if(!sensing_ready || !cleaning_ready){
        RCLCPP_ERROR(get_logger(), "Homing TIMEOUT - sensing %s, Cleaning : %s",
                                sensing_ready  ? "OK" : "TIMEOUT",
                                cleaning_ready ? "OK" : "TIMEOUT");
        return BT::NodeStatus::FAILURE;
    }

    auto sensing_response = sensing_future.get();
    auto cleaning_response = cleaning_future.get();

    if(sensing_response->success && cleaning_response->success){
        RCLCPP_INFO(get_logger(), "ALL COBOT HOMING SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }else{
        RCLCPP_ERROR(get_logger(), "HOMING FAILED - Sensing: %s, Cleaning: %s",
                        sensing_response->success  ? "OK" : "FAIL",
                        cleaning_response->success ? "OK" : "FAIL");
        return BT::NodeStatus::FAILURE;
    }
}


BT::NodeStatus MagicianClientNode::sendHomingRequest(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
    std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::chrono::seconds timeout,
    const std::string& robot_name
){
    auto future = client->async_send_request(request);

    if(future.wait_for(timeout) != std::future_status::ready){

        RCLCPP_ERROR(get_logger(),"%s homing TIMEOUT", robot_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if(auto response = future.get(); response->success){
        RCLCPP_INFO(get_logger(), "%s HOMING SUCCESS",robot_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }else{
        RCLCPP_ERROR(get_logger(),"%s HOMING FAILED", robot_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

}

MagicianOpcUA::MagicianOpcUA(const std::string& node_name) : rclcpp::Node{node_name} 
{
    sensing_safe_transfer_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/safetransfer_set");
    cleaning_safe_transfer_client_= create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/safetransfer_set");

    while(!sensing_safe_transfer_client_->wait_for_service(std::chrono::milliseconds(4))){

        if(!rclcpp::ok()){
            RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for opcua services to appear");
            assert(0);
        }
        RCLCPP_INFO(get_logger(),"waiting for services opcua services to appear");    
    }

    RCLCPP_INFO(get_logger(), "OPCUA Clients is ready");
}

BT::NodeStatus MagicianOpcUA::OpcServiceCall(){

    
auto sensing_safe_transfer_request = std::make_shared<std_srvs::srv::SetBool::Request>();
auto cleaning_safe_transfer_request = std::make_shared<std_srvs::srv::SetBool::Request>();

sensing_safe_transfer_request->data = true;
cleaning_safe_transfer_request->data = true;

sensing_safe_transfer_client_->async_send_request(sensing_safe_transfer_request);
cleaning_safe_transfer_client_->async_send_request(cleaning_safe_transfer_request);

RCLCPP_INFO(get_logger(), "Safe Transfer Home enabled.");
return BT::NodeStatus::SUCCESS;

}

