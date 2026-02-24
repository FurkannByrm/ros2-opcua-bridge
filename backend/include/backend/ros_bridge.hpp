#pragma once
#include "backend/opcua_client.hpp"
#include "backend/config.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include "backend/srv/set_int16.hpp"
#include "backend/srv/set_float32.hpp"

using BoolPub = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using BoolSrv = rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr;

class RosBridge : public rclcpp::Node {
public:

explicit RosBridge(const UaConfig& cfg);

private:
  std::shared_ptr<UaClient> ua_;
  UaConfig cfg_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_speed_;

  BoolPub  pub_cobot_;
  //sensing publishers
  BoolPub pub_sensing_robot_home_st_;
  BoolPub pub_sensing_finished_;
  BoolPub pub_touch_sensing_finished_;
  BoolPub pub_sensing_active_;
  BoolPub pub_touch_sensing_active_;
  BoolPub pub_sensing_slide_command_;
  BoolPub pub_sensing_running_;
  
  //cleaning publishers
  BoolPub pub_cleaning_robot_home_st_;
  BoolPub pub_cleaning_finished_;
  BoolPub pub_cleaning_active_;
  BoolPub pub_cleaning_slide_command_;
  BoolPub pub_cleaning_running_;
  

  rclcpp::Service<backend::srv::SetInt16>::SharedPtr srv_speed_set_;  
  BoolSrv srv_cobot_set_;
  
  //Sensing robot
  BoolSrv srv_sensing_robot_home_st_set_;
  BoolSrv srv_sensing_finished_set_;
  BoolSrv srv_touch_sensing_finished_set_;
  BoolSrv srv_sensing_active_set_;
  BoolSrv srv_touch_sensing_active_set_;
  BoolSrv srv_slide_sensing_command_set_;
  BoolSrv srv_running_sensing_set_;
  rclcpp::Service<backend::srv::SetFloat32>::SharedPtr srv_slider1_set_pos_;
  BoolSrv srv_slider1_go_pos_;

  //cleaning robot
  BoolSrv srv_cleaning_robot_home_st_set_;
  BoolSrv srv_cleaning_finished_set_;
  BoolSrv srv_cleaning_active_set_;
  BoolSrv srv_slide_cleaning_command_set_;
  BoolSrv srv_running_cleaning_set_;
  rclcpp::Service<backend::srv::SetFloat32>::SharedPtr srv_slider2_set_pos_;
  BoolSrv srv_slider2_go_pos_;


  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_speed_;

  // void setup_subscriptions();
  void setup_services();
  void setup_publishers();
};
