#pragma once
#include "opcua_to_ros2/naming.hpp"
#include "opcua_to_ros2/opcua_client.hpp"
#include "opcua_to_ros2/config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "opcua_to_ros2/srv/set_int16.hpp"
#include "opcua_to_ros2/srv/set_float32.hpp"
#include "opcua_to_ros2/srv/get_spot_weights.hpp"
#include "opcua_to_ros2/msg/welding.hpp"


using BoolPub = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using BoolSrv = rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr;

class OPCuaBridge{
    
    public:
    explicit OPCuaBridge(const UaConfig& cfg);
    
    template<typename T>
        void get_workcell(const std::string& opc_topic, std::function<void(T)> callb){
               access<OpcOperation::Subscribe,T>(opc_topic, std::move(callb),cfg_.structs.workcell_status);
        }

    template<typename T>
        void get_sensing(const std::string& opc_topic, std::function<void(T)> callb){
            
           access<OpcOperation::Subscribe,T>(opc_topic, std::move(callb),cfg_.structs.sensing_root);
        }

    template<typename T>
        void get_cleaning(const std::string& opc_topic, std::function<void(T)> callb){
           access<OpcOperation::Subscribe,T>(opc_topic, std::move(callb),cfg_.structs.cleaning_root);
        }

    template<typename T>
        void get_mod(const std::string& opc_topic, std::function<void(T)> callb){
           access<OpcOperation::Subscribe,T>(opc_topic, std::move(callb),cfg_.structs.mod_root);
        }
    template<typename T>
        void welding_load_1(const std::string& opc_topic, std::function<void(T)> callb){
            access<OpcOperation::Read,T>(opc_topic, std::move(callb), cfg_.structs.spot_M1_root);
        }

    template<typename T>
        void set_workcell(const std::string& opc_topic, T value){
               access<OpcOperation::Write,T>(opc_topic, std::move(value),cfg_.structs.workcell_status);
        }

    template<typename T>
        void set_sensing(const std::string& opc_topic, T value){
            
           access<OpcOperation::Write,T>(opc_topic, std::move(value),cfg_.structs.sensing_root);
        }

    template<typename T>
        void set_cleaning(const std::string& opc_topic, T value){
           access<OpcOperation::Write,T>(opc_topic, std::move(value),cfg_.structs.cleaning_root);
        }

    template<typename T>
        void set_mod(const std::string& opc_topic, T value){
           access<OpcOperation::Write,T>(opc_topic, std::move(value),cfg_.structs.mod_root);
        }

    std::unordered_map<std::string,bool> welding_lookup_;
    std::vector<std::string> welding_insertion_order_;
    std::mutex welding_mutex_;    
    private:
    UaClient ua_;
    UaConfig cfg_;
    void weldingLoad();

    enum class OpcOperation
    {
        Subscribe,
        Write,
        Read
    };
    template<OpcOperation op,typename T, typename Args>
        void access(const std::string& opc_topic, Args&& callb, const std::string& root = "");
    


};

template<OPCuaBridge::OpcOperation op,typename T, typename Args>
void OPCuaBridge::access( const std::string& opc_topic, Args&& callb,const std::string& root){
    const std::string node = root.empty() ? opc_topic : make_child_node(root, opc_topic);
    if constexpr(op == OpcOperation::Subscribe){

        if constexpr (std::is_same_v<T, double>) {
        ua_.subscribe_double(node, std::forward<Args>(callb));
        }
        else if constexpr (std::is_same_v<T, int>) {
        ua_.subscribe_int16(node, std::forward<Args>(callb));
        }
        else if constexpr (std::is_same_v<T, bool>) {
        ua_.subscribe_bool(node, std::forward<Args>(callb));
        }else{
        static_assert(always_false_v<T>,"OPCUA does not support this type");
        }
    }else if constexpr (op == OpcOperation::Write) { 

        if constexpr (std::is_same_v<T, double>) {
        ua_.enqueue_write_double(node, std::forward<Args>(callb));
        }
        else if constexpr (std::is_same_v<T, int>) {
        ua_.enqueue_write_int16(node, std::forward<Args>(callb));
        }
        else if constexpr (std::is_same_v<T, bool>) {
        ua_.enqueue_write_bool(node, std::forward<Args>(callb));
        }else{
        static_assert(always_false_v<T>,"OPCUA does not support this type");
        }
    }else if constexpr (op == OpcOperation::Read) {
    if constexpr (std::is_same_v<T, bool>) {    
        ua_.subscribe_bool(opc_topic, std::forward<Args>(callb)); 
    } else {
        static_assert(always_false_v<T>, "OPCUA does not support this type");
    }
}

}

class CommonDemonstratorInterface : public rclcpp::Node{
    
    public:
    CommonDemonstratorInterface(std::shared_ptr<OPCuaBridge> opc_srv);
    
    private:
    void modSelection(rclcpp::QoS qos) ; 
    std::shared_ptr<OPCuaBridge> opc_srv_; 
    BoolPub  pub_cobot_mode_;
    BoolPub pub_automatic_mode_;
    
    BoolSrv srv_cobot_mode_set_;
    BoolSrv srv_full_automatic_mode_set_;
    rclcpp::Service<opcua_to_ros2::srv::GetSpotWeights>::SharedPtr get_welding_data_;


};    

class Sensing : public rclcpp::Node{
    public:
    Sensing(std::shared_ptr<OPCuaBridge> opc_srv); 
    private:
    void sensingPublishers(rclcpp::QoS qos);
    void sensingServices();
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_sensing_slider_actual_pos_; 
    BoolPub pub_sensing_robot_home_st_;
    BoolPub pub_sensing_finished_;
    BoolPub pub_touch_sensing_finished_;
    BoolPub pub_sensing_active_;
    BoolPub pub_touch_sensing_active_;
    BoolPub pub_sensing_slide_command_;
    BoolPub pub_sensing_running_;
    BoolPub pub_sensing_carbody_located_st_;
    BoolPub pub_sensing_position_2_;
    BoolPub pub_sensing_position_3_;
    BoolPub pub_sensing_position_4_; 
    BoolPub pub_sensing_position_5_;
    BoolPub pub_sensing_position_2_reached_;
    BoolPub pub_sensing_position_3_reached_;
    BoolPub pub_sensing_position_4_reached_; 
    BoolPub pub_sensing_position_5_reached_;

    BoolSrv srv_sensing_robot_home_st_set_;
    BoolSrv srv_sensing_finished_set_;
    BoolSrv srv_touch_sensing_finished_set_;
    BoolSrv srv_sensing_active_set_;
    BoolSrv srv_touch_sensing_active_set_;
    BoolSrv srv_slide_sensing_command_set_;
    BoolSrv srv_running_sensing_set_;
    BoolSrv srv_sensing_pos2_set_;
    BoolSrv srv_sensing_pos3_set_;
    BoolSrv srv_sensing_pos4_set_;
    BoolSrv srv_sensing_pos5_set_;

    std::shared_ptr<OPCuaBridge> opc_srv_; 
};


class Cleaning : public rclcpp::Node{
    public:
    Cleaning(std::shared_ptr<OPCuaBridge> opc_srv );
    private:
    void cleaningPublishers(rclcpp::QoS qos);
    void cleaningServices();
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cleaning_slider_actual_pos_; 
    BoolPub pub_cleaning_robot_home_st_;
    BoolPub pub_cleaning_finished_;
    BoolPub pub_cleaning_active_;
    BoolPub pub_cleaning_slide_command_;
    BoolPub pub_cleaning_running_;
    BoolPub pub_cleaning_carbody_located_st_; 
    BoolPub pub_cleaning_position_2_;
    BoolPub pub_cleaning_position_3_;
    BoolPub pub_cleaning_position_4_;
    BoolPub pub_cleaning_position_5_;
    BoolPub pub_cleaning_position_2_reached_;
    BoolPub pub_cleaning_position_3_reached_;
    BoolPub pub_cleaning_position_4_reached_;
    BoolPub pub_cleaning_position_5_reached_;


    BoolSrv srv_cleaning_robot_home_st_set_;
    BoolSrv srv_cleaning_finished_set_;
    BoolSrv srv_cleaning_active_set_;
    BoolSrv srv_slide_cleaning_command_set_;
    BoolSrv srv_running_cleaning_set_; 
    BoolSrv srv_cleaning_pos2_set_;
    BoolSrv srv_cleaning_pos3_set_;
    BoolSrv srv_cleaning_pos4_set_; 
    BoolSrv srv_cleaning_pos5_set_;

    std::shared_ptr<OPCuaBridge> opc_srv_; 
};


