#include "demonstrator_tree/behavior_node.hpp"

using namespace DemostratorTree;


MagicianSubNode::MagicianSubNode(const std::string& node_name, const CobotConfig& cfg) 
: rclcpp::Node{node_name}, cfg_{cfg}{ 
    auto robot1 = cfg_.sensing_group.name;
    auto robot2 = cfg_.cleaning_group.name; 

    robot_home_status_.try_emplace(robot1,false,cfg_.sensing_group.home_vec);
    robot_home_status_.try_emplace(robot2,false,cfg_.cleaning_group.home_vec); 
    system_operation_mode_.try_emplace("COBOT",false);
    system_operation_mode_.try_emplace("AUTOMATIC",false);
    carbody_location_.try_emplace(robot1,false);
    carbody_location_.try_emplace(robot2,false);
    robot_activation_status_.try_emplace(robot1,true); 
    robot_activation_status_.try_emplace(robot2,true);

    group_home_pos_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options_home;
    options_home.callback_group = group_home_pos_;

    group_sequence_opc_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options_seq;
    options_seq.callback_group = group_sequence_opc_; 

    sensing_home_axis_pos_ = create_subscription<xbot_msgs::msg::JointState>
    (cfg_.sensing_group.joint_states,rclcpp::SensorDataQoS(),
        [this,robot1](const xbot_msgs::msg::JointState::ConstSharedPtr& msg){
            homePosCallback(msg,robot1);
        },options_home);
    cleaning_home_axis_pos_= create_subscription<xbot_msgs::msg::JointState>
    (cfg_.cleaning_group.joint_states,rclcpp::SensorDataQoS(),
            [this, robot2](const xbot_msgs::msg::JointState::ConstSharedPtr& msg){
            homePosCallback(msg,robot2);
        },options_home);
   
   
    system_cobot_mode_info_ = create_subscription<std_msgs::msg::Bool>("/ros2_comm/mod/cobot", 10,
            [this](const std_msgs::msg::Bool::ConstSharedPtr& msg){
    systemModeTypeCallback(msg,"COBOT");
            },options_seq);

    system_automatic_mode_info_ =create_subscription<std_msgs::msg::Bool>("/ros2_comm/mod/automatic",10,
            [this](const std_msgs::msg::Bool::ConstSharedPtr& msg){
    systemModeTypeCallback(msg,"AUTOMATIC");
            },options_seq);

    sensing_car_body_located_info_= create_subscription<std_msgs::msg::Bool>("/ros2_comm/sensing/carbody_located_status",10,
      [this,robot1](const std_msgs::msg::Bool::ConstSharedPtr& msg){

      carBodyLocationCallback(msg,robot1); 
      },options_seq);

    cleaning_car_body_located_info_= create_subscription<std_msgs::msg::Bool>("/ros2_comm/cleaning/carbody_located_status",10,
       [this,robot2](const std_msgs::msg::Bool::ConstSharedPtr& msg){
        carBodyLocationCallback(msg,robot2);
       },options_seq);

    sensing_activation_info_ = create_subscription<std_msgs::msg::Bool>("/ros2_comm/sensing/sensing_active",10,
        [this,robot1](const std_msgs::msg::Bool::ConstSharedPtr& msg){
       robotActivationCallback(msg,robot1); 
        },options_seq); 


    cleaning_activation_info_ = create_subscription<std_msgs::msg::Bool>("/ros2_comm/cleaning/cleaning_active",10,
        [this,robot2](const std_msgs::msg::Bool::ConstSharedPtr& msg){
       robotActivationCallback(msg,robot2); 
        },options_seq);  

    RCLCPP_INFO(get_logger(), " Magician Subscribers initialized");
}


void MagicianSubNode::homePosCallback(const xbot_msgs::msg::JointState::ConstSharedPtr& msg, const std::string& robot_id)noexcept{

    auto home_pos_vec   = robot_home_status_[robot_id].second; 

    robot_home_status_[robot_id].first = true;
    
    for(size_t i = 0; i<home_pos_vec.size(); i++)
    {   
            auto target = home_pos_vec[i];       
            auto current = msg->link_position[i];
            
            if(std::fabs(target - current) > JOINT_TOL){
                
                robot_home_status_[robot_id].first = false;
                break;
            }
        }
        
      if (robot_home_status_[robot_id].first){
        RCLCPP_INFO(get_logger(), "%s - ROBOT HOME POSITION", robot_id.c_str());
    }else
    {
        RCLCPP_INFO(get_logger(), "%s - Robot NOT home", robot_id.c_str());
    }

    if (robot_id == cfg_.sensing_group.name) {
        is_sensing_in_op_pos_ = true;
        for(size_t i = 0; i < cfg_.sensing_group.target_pos_vec.size(); i++) {
            if(std::fabs(cfg_.sensing_group.target_pos_vec[i] - msg->link_position[i]) > JOINT_TOL) {
                is_sensing_in_op_pos_ = false;
                break;
            }
        }
    }
    

    else if (robot_id == cfg_.cleaning_group.name) {
        is_cleaning_in_op_pos_ = true;
        for(size_t i = 0; i < cfg_.cleaning_group.target_pos_vec.size(); i++) {
            if(std::fabs(cfg_.cleaning_group.target_pos_vec[i] - msg->link_position[i]) > JOINT_TOL) {
                is_cleaning_in_op_pos_ = false;
                break;
            }
        }
    }
    

}

void MagicianSubNode::systemModeTypeCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg, const std::string& mode){

       system_operation_mode_[mode] = msg->data;
       RCLCPP_INFO(get_logger(),"%s mode is setted as %s",mode.c_str(), msg->data ? "true" : "false");
}


void MagicianSubNode::carBodyLocationCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg, const std::string& robot_id){
        carbody_location_[robot_id] =msg->data; 
       RCLCPP_INFO(get_logger(),"Position of the Car Body in front of the %s : %s",robot_id.c_str(), msg->data ? "true" : "false");
}



void MagicianSubNode::robotActivationCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg,const std::string& robot_id){
        robot_activation_status_[robot_id] = msg->data;
       RCLCPP_INFO(get_logger(),"%s Activation status : %s",robot_id.c_str(), msg->data ? "true" : "false");
}


MagicianClientNode::MagicianClientNode(const std::string& node_name, const CobotConfig& cfg, std::shared_ptr<MagicianSubNode> sub_node) 
: rclcpp::Node{node_name}, cfg_{cfg}, sub_node_{sub_node}{

    sensing_homing_client_ = create_client<std_srvs::srv::SetBool>(cfg_.sensing_group.service_name);
    cleaning_homing_client_= create_client<std_srvs::srv::SetBool>(cfg_.cleaning_group.service_name);
    sensing_mock_operation_client_ = create_client<std_srvs::srv::SetBool>("/sr/xbotcore/homing_ergodic/switch");
    cleaning_mock_operation_client_ = create_client<std_srvs::srv::SetBool>("/cr/xbotcore/homing2/switch");

    while(!sensing_homing_client_->wait_for_service(std::chrono::microseconds(4))){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(get_logger(),"Client interrupted while waiting for sensing service to appear ");
        }
           
        RCLCPP_INFO(get_logger(),"Waiting for sensing service to appear");
    }

    RCLCPP_INFO(get_logger(),"sensing and cleaning client created..");

}

BT::NodeStatus MagicianClientNode::HomingCall(){

    auto sensing_at_home = sub_node_->sensingHomeInfo();
    auto cleaning_at_home = sub_node_->cleaningHomeInfo();

    auto sensing_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    sensing_request->data = true;
    
    auto cleaning_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    cleaning_request->data = true;

    const auto timeout = std::chrono::seconds(8);

    //Only sensing needs homing
    if(!sensing_at_home && cleaning_at_home){
        return SendHomingRequest(sensing_homing_client_, sensing_request, timeout, "SENSING");
    } 

    //Only cleaning needs homing
    if(sensing_at_home && !cleaning_at_home){
        return SendHomingRequest(cleaning_homing_client_, cleaning_request, timeout, "CLEANING");
    }

    //both need homing
    auto sensing_future = sensing_homing_client_->async_send_request(sensing_request);
    auto cleaning_future = cleaning_homing_client_->async_send_request(cleaning_request);

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


BT::NodeStatus MagicianClientNode::SendHomingRequest(
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
    automatic_mode_set_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/mod/full_automatic_mode_set");
    cobot_mode_set_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/mod/cobot_mode_set");


    sensing_safe_transfer_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/safetransfer_set");
    cleaning_safe_transfer_client_= create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/safetransfer_set");

    sensing_activation_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/active_set");

    touch_sensing_activation_client_= create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_active_set");

    cleaning_activation_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_active_set");

    sensing_finished_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/finished_set");
    touch_sensing_finished_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_finished_set");

    cleaning_finished_client_ = create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_finished_set"); 

        while(!sensing_safe_transfer_client_->wait_for_service(std::chrono::milliseconds(4))){

        if(!rclcpp::ok()){
            RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for opcua services to appear");
        }
        RCLCPP_INFO(get_logger(),"waiting for services opcua services to appear");    
    }

    RCLCPP_INFO(get_logger(), "OPCUA Clients is ready");
}

