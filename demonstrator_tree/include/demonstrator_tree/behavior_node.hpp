#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <xbot_msgs/msg/joint_state.hpp> 

#include "behaviortree_cpp/bt_factory.h"
#include "demonstrator_tree/parameters_parser.hpp"


namespace DemostratorTree{


    class MagicianSubNode : public rclcpp::Node{


        public:
        MagicianSubNode(const std::string& node_name, const CobotConfig& cfg);

        BT::NodeStatus CheckPos()const{
         
            auto flage1 = sensingHomeInfo();
            auto flage2 = cleaningHomeInfo();
            
            if(flage1 && flage2){
                RCLCPP_INFO(get_logger(),"SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            else{
                RCLCPP_WARN(get_logger(),"FALLBACK IS CONTINUES");
                return BT::NodeStatus::FAILURE;
            }

        }

        BT::NodeStatus CheckMode()const{
            
            if(system_operation_mode_.at("COBOT") || system_operation_mode_.at("AUTOMATIC")){
                return BT::NodeStatus::SUCCESS;
            }
            else{
                RCLCPP_WARN(get_logger(),"WAITING FOR MODE SELECTION");
                return BT::NodeStatus::FAILURE; 
            }
        }

        BT::NodeStatus IsCobotMode()const{
            
            if(system_operation_mode_.at("COBOT")){
                RCLCPP_INFO(get_logger(),"COBOT MODE ENABLE");
                return BT::NodeStatus::SUCCESS;
            }else{
                return BT::NodeStatus::FAILURE;
            }
        }

        BT::NodeStatus IsAutomaticMode()const{
            
            if(system_operation_mode_.at("AUTOMATIC")){
                RCLCPP_INFO(get_logger(),"AUTOMATIC MODE ENABLE");
                return BT::NodeStatus::SUCCESS;
            }else{
                return BT::NodeStatus::FAILURE;
            }
        }

        BT::NodeStatus CarBodyLocation(const std::string& robot_name)const{
            if(carBodyLocatedInfo(robot_name)){
                RCLCPP_INFO(get_logger(),"Car Body is located.");
                return BT::NodeStatus::SUCCESS;
            }else{

                RCLCPP_WARN(get_logger(),"WAITING FOR CARBODY HOME LOCATION...");
                return BT::NodeStatus::FAILURE;
            }
        }
        

        BT::NodeStatus DeactivationMode(const std::string& robot_name)const{
            if(deactivationState(robot_name)){

                RCLCPP_INFO(get_logger(),"%s cobot is deactivated",robot_name.c_str());
                return BT::NodeStatus::SUCCESS;
            }else{ 
                RCLCPP_WARN(get_logger(),"WAITING FOR %s OPERATION...",robot_name.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }


    BT::NodeStatus CheckOperationPos(const std::string& robot_name) const {
        if (robot_name == cfg_.sensing_group.name) {
            if (is_sensing_in_op_pos_) {
                RCLCPP_INFO(get_logger(), "SENSING ROBOT operation position reached.");
                return BT::NodeStatus::SUCCESS;
            }
        } 
        else if (robot_name == cfg_.cleaning_group.name) {
            if (is_cleaning_in_op_pos_) {
                RCLCPP_INFO(get_logger(), "CLEANING ROBOT operation position reached.");
                return BT::NodeStatus::SUCCESS;
            }
        }
        
        RCLCPP_WARN(get_logger(), "WAITING FOR %s TO REACH OPERATION POSITION...", robot_name.c_str());
        return BT::NodeStatus::FAILURE;
    }


        bool sensingHomeInfo()const{
            return robot_home_status_.at(cfg_.sensing_group.name).first;

        }
        bool cleaningHomeInfo()const{
            return robot_home_status_.at(cfg_.cleaning_group.name).first;
        }
        
        bool carBodyLocatedInfo(const std::string& robot_name)const{
           if(carbody_location_.at(robot_name)){
               return  true;
           }else {
            return false;
           } 
        }
        bool deactivationState(const std::string& robot_name)const{
            if(!robot_activation_status_.at(robot_name)){
                return true;
            }else{
                return false;
            }
        }

        

        private:        
        
        void homePosCallback(const xbot_msgs::msg::JointState::ConstSharedPtr& msg, const std::string& robot_id)noexcept;
        void systemModeTypeCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg, const std::string& mode); 
        
        void carBodyLocationCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg, const std::string& robot_id);
       
        void robotActivationCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg, const std::string& robot_id);

        const double JOINT_TOL = 0.01;
        CobotConfig cfg_; 
        std::unordered_map<std::string, std::pair<bool, std::vector<double>>> robot_home_status_;
        std::unordered_map<std::string, bool> system_operation_mode_;
        std::unordered_map<std::string,bool> carbody_location_;  
        std::unordered_map<std::string,bool> robot_activation_status_; 
        
        rclcpp::Subscription<xbot_msgs::msg::JointState>::SharedPtr sensing_home_axis_pos_;
        rclcpp::Subscription<xbot_msgs::msg::JointState>::SharedPtr cleaning_home_axis_pos_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr system_cobot_mode_info_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr system_automatic_mode_info_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensing_car_body_located_info_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cleaning_car_body_located_info_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensing_activation_info_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cleaning_activation_info_;

        rclcpp::CallbackGroup::SharedPtr group_home_pos_;
        rclcpp::CallbackGroup::SharedPtr group_sequence_opc_;


        //temporary scenerio
    bool is_sensing_in_op_pos_ = false;
    bool is_cleaning_in_op_pos_ = false;

    };

    enum class TargetOpcUaClient{
    sensing_activation,
    cleaning_activation,
    sensing_finished,
    cleaning_finished,
    automatic_mod,
    cobot_mod,
    cleaning_home,
    sensing_home,
    both_home,
    sensing_operation,
    cleaning_operation
    };

    class MagicianOpcUA : public rclcpp::Node{

        public:
        
        MagicianOpcUA(const std::string& node_name);

        BT::NodeStatus SetSensingActivation(bool act_req);
        template<TargetOpcUaClient Target>
        BT::NodeStatus ExecuteServiceByName(bool act_req){
            if constexpr (Target == TargetOpcUaClient::sensing_activation) {
                return callSetBoolServices(act_req, &MagicianOpcUA::sensing_activation_client_,&MagicianOpcUA::touch_sensing_activation_client_); 
            }
            else if constexpr (Target == TargetOpcUaClient::cleaning_activation){
                return callSetBoolServices(act_req, &MagicianOpcUA::cleaning_activation_client_);
            }
            else if constexpr (Target == TargetOpcUaClient::sensing_finished){
                return callSetBoolServices(act_req, &MagicianOpcUA::sensing_finished_client_, &MagicianOpcUA::touch_sensing_finished_client_);
            }
            else if constexpr (Target == TargetOpcUaClient::cleaning_finished){
                return callSetBoolServices(act_req, &MagicianOpcUA::cleaning_finished_client_);
            }
            else if constexpr (Target == TargetOpcUaClient::automatic_mod){
                return callSetBoolServices(act_req, &MagicianOpcUA::automatic_mode_set_client_);
            }
            else if constexpr (Target == TargetOpcUaClient::cobot_mod){
                return callSetBoolServices(act_req, &MagicianOpcUA::cobot_mode_set_client_);
            }
            else if constexpr (Target == TargetOpcUaClient::both_home){
                return callSetBoolServices(act_req, &MagicianOpcUA::cleaning_safe_transfer_client_,&MagicianOpcUA::sensing_safe_transfer_client_);
            }
            else if constexpr (Target == TargetOpcUaClient::sensing_home){
                return callSetBoolServices(act_req, &MagicianOpcUA::sensing_safe_transfer_client_);
            }
            else if constexpr (Target == TargetOpcUaClient::cleaning_home){
                return callSetBoolServices(act_req, &MagicianOpcUA::cleaning_safe_transfer_client_);
            }
        }

        
        private:
    
        template<typename... ClientPtr>
            BT::NodeStatus callSetBoolServices(bool act_req,ClientPtr... clients){
                const auto timeout = std::chrono::seconds(10);
                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = act_req;

                std::vector<std::shared_future<std::shared_ptr<std_srvs::srv::SetBool::Response>>> futures{
                    (this->*clients)->async_send_request(req).share()...
                };

                bool all_ready = true;
                for(auto& futr : futures){
                    if(futr.wait_for(timeout) != std::future_status::ready){
                        all_ready = false;
                    }
                }

                if(!all_ready){
                    return BT::NodeStatus::FAILURE;
                }

                for (auto& futr : futures)
                {
                    if (!futr.get()->success)
                    {
                        return BT::NodeStatus::FAILURE;
                    }
                    
                }

                return BT::NodeStatus::SUCCESS;

            }

        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr automatic_mode_set_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cobot_mode_set_client_;


        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sensing_safe_transfer_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cleaning_safe_transfer_client_;
        
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sensing_activation_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr touch_sensing_activation_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cleaning_activation_client_;

        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sensing_finished_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr touch_sensing_finished_client_;

        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cleaning_finished_client_;

    };


    class MagicianClientNode : public rclcpp::Node{

        public:
            MagicianClientNode(const std::string& node_name, const CobotConfig& cfg,  std::shared_ptr<MagicianSubNode> sub_node);


            BT::NodeStatus HomingCall();
            BT::NodeStatus SendHomingRequest(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
                    std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::chrono::seconds timeout,
                    const std::string& robot_name);
        
        template<TargetOpcUaClient Target>
        BT::NodeStatus ExecuteServiceByName(bool act_req){
            if constexpr (Target == TargetOpcUaClient::sensing_operation) {
                return callSetBoolServices(act_req, &MagicianClientNode::sensing_mock_operation_client_); 
            }
            else if constexpr (Target == TargetOpcUaClient::cleaning_operation){
                return callSetBoolServices(act_req, &MagicianClientNode::cleaning_mock_operation_client_);
            }
        }

        private:    
            
        
        template<typename... ClientPtr>
            BT::NodeStatus callSetBoolServices(bool act_req,ClientPtr... clients){
                const auto timeout = std::chrono::seconds(5);
                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = act_req;

                std::vector<std::shared_future<std::shared_ptr<std_srvs::srv::SetBool::Response>>> futures{
                    (this->*clients)->async_send_request(req).share()...
                };

                bool all_ready = true;
                for(auto& futr : futures){
                    if(futr.wait_for(timeout) != std::future_status::ready){
                        all_ready = false;
                    }
                }

                if(!all_ready){
                    return BT::NodeStatus::FAILURE;
                }

                for (auto& futr : futures)
                {
                    if (!futr.get()->success)
                    {
                        return BT::NodeStatus::FAILURE;
                    }
                    
                }
                
                return BT::NodeStatus::SUCCESS;

            }



            CobotConfig cfg_;
            std::shared_ptr<MagicianSubNode> sub_node_;
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sensing_homing_client_;
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cleaning_homing_client_;

            // Mock operation 
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sensing_mock_operation_client_;
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cleaning_mock_operation_client_;
    };

}
