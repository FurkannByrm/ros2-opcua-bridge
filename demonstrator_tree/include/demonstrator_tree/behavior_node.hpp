#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // ->you have to change this message type with xbot_msgs format
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "demonstrator_tree/parameters_parser.hpp"



namespace DemostratorTree{


    class MagicianSubNode : public rclcpp::Node{


        public:
        MagicianSubNode(const std::string& node_name, const CobotConfig& cfg);

        BT::NodeStatus checkPos()const{
         
            auto flage1 = sensingStateGet();
            auto flage2 = cleaningStateGet();
            
            if(flage1 && flage2){
                RCLCPP_INFO(get_logger(),"SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            else{
                RCLCPP_ERROR(get_logger(),"FAILURE" );
                return BT::NodeStatus::FAILURE;
            }

        }

        static inline bool sensingStateGet(){
            return robot_home_status_.rbegin()->second.first;
        }
        static inline bool cleaningStateGet(){
            return robot_home_status_.begin()->second.first;
        }

        
        private:        
        
        void homePosCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg, const std::string& robot_id);

        const double JOINT_TOL = 0.01;
        CobotConfig cfg_;
        static  inline std::map<std::string, std::pair<bool, std::vector<double>>> robot_home_status_;
        
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensing_home_axis_pos_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cleaning_home_axis_pos_;


    };

    class MagicianClientNode : public rclcpp::Node{

        public:
        MagicianClientNode(const std::string& node_name, const CobotConfig& cfg);

        
        BT::NodeStatus homingCall();

        BT::NodeStatus sendHomingRequest(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
                                         std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                         std::chrono::seconds timeout,
                                         const std::string& robot_name);


        private:
        CobotConfig cfg_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sensing_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cleaning_client_;

    };


    class MagicianOpcUA : public rclcpp::Node{

        public:
        
        MagicianOpcUA(const std::string& node_name);

        BT::NodeStatus OpcServiceCall();
        private:
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sensing_safe_transfer_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cleaning_safe_transfer_client_;

    };

}