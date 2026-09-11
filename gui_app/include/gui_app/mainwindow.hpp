#pragma once
#include <QMainWindow>
#include <QPushButton>
#include <QLineEdit>
#include <QTimer>
#include <qpushbutton.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>

using BoolSubscription    = rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr;
using BoolClient = rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr;

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget* parent=nullptr);
  ~MainWindow();

private:
  rclcpp::Node::SharedPtr                                 node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr    executor_;
  QTimer*                                                 ros_timer_;
  
  BoolClient       cli_mod_cobot_;
  BoolClient       cli_mod_automatic_;

  // Sensing robot clients
  BoolClient       cli_sensing_safetransfer_;
  BoolClient       cli_sensing_finished_;
  BoolClient       cli_sensing_touch_finished_;
  BoolClient       cli_sensing_active_;
  BoolClient       cli_sensing_touch_active_;
  BoolClient       cli_sensing_running_;
  BoolClient       cli_sensing_carbody_located_st_;  
  BoolClient       cli_sensing_pos2_st_;  
  BoolClient       cli_sensing_pos3_st_;  
  BoolClient       cli_sensing_pos4_st_;  
  BoolClient       cli_sensing_pos5_st_;  


  // Cleaning robot clients
  BoolClient       cli_cleaning_safetransfer_;
  BoolClient       cli_cleaning_finished_;
  BoolClient       cli_cleaning_active_;
  BoolClient       cli_cleaning_running_;
  BoolClient       cli_cleaning_carbody_located_st_;
  BoolClient       cli_cleaning_pos2_st_;  
  BoolClient       cli_cleaning_pos3_st_;  
  BoolClient       cli_cleaning_pos4_st_;  
  BoolClient       cli_cleaning_pos5_st_;  


  // Subscriptions
  BoolSubscription   sub_cobot_mode_;
  BoolSubscription   sub_automatic_mode_;
  
  // Sensing subscriptions
  BoolSubscription   sub_sensing_safetransfer_;
  BoolSubscription   sub_sensing_finished_;
  BoolSubscription   sub_sensing_touch_finished_;
  BoolSubscription   sub_sensing_active_;
  BoolSubscription   sub_sensing_touch_active_;
  BoolSubscription   sub_sensing_running_;
  BoolSubscription   sub_sensing_carbody_located_st_;
  BoolSubscription   sub_sensing_pos2_st_;
  BoolSubscription   sub_sensing_pos3_st_;
  BoolSubscription   sub_sensing_pos4_st_;
  BoolSubscription   sub_sensing_pos5_st_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_sensing_slider_;

  // Cleaning subscriptions
  BoolSubscription   sub_cleaning_safetransfer_;
  BoolSubscription   sub_cleaning_finished_;
  BoolSubscription   sub_cleaning_active_;
  BoolSubscription   sub_cleaning_running_;
  BoolSubscription   sub_cleaning_carbody_located_st_;
  BoolSubscription   sub_cleaning_pos2_st_;
  BoolSubscription   sub_cleaning_pos3_st_;
  BoolSubscription   sub_cleaning_pos4_st_;
  BoolSubscription   sub_cleaning_pos5_st_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cleaning_slider_;

  // UI Elements - Toggle Buttons
  QPushButton* btnCobotModeToggle_; 
  QPushButton* btnAutomaticModeToggle_;
  QPushButton* btnSensingSafeTransferToggle_;
  QPushButton* btnSensingFinishedToggle_;
  QPushButton* btnSensingTouchFinishedToggle_;
  QPushButton* btnSensingActiveToggle_;
  QPushButton* btnSensingTouchActiveToggle_;
  QPushButton* btnSensingRunningToggle_;
  QPushButton* btnSensingCarbodyLocatedSt_;
  QPushButton* btnSensingPos2Toggle_;
  QPushButton* btnSensingPos3Toggle_;
  QPushButton* btnSensingPos4Toggle_;
  QPushButton* btnSensingPos5Toggle_;

  QPushButton* btnCleaningSafeTransferToggle_;
  QPushButton* btnCleaningFinishedToggle_;
  QPushButton* btnCleaningActiveToggle_;
  QPushButton* btnCleaningRunningToggle_;
  QPushButton* btnCleaningCarbodyLocatedSt_;
  QPushButton* btnCleaningPos2Toggle_;
  QPushButton* btnCleaningPos3Toggle_;
  QPushButton* btnCleaningPos4Toggle_;
  QPushButton* btnCleaningPos5Toggle_;
  QLineEdit* slider1Pos_;
  QLineEdit* slider2Pos_;
    
  rclcpp::QoS slider_qos_;
  rclcpp::QoS common_qos_;
  rclcpp::QoS sensing_and_cleaning_qos_;
  void setup_ros();
  void call_service(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value);
  
  QPushButton* createToggleButton(const QString& label);
  void updateToggleButtonStyle(QPushButton* btn, bool state);
};
