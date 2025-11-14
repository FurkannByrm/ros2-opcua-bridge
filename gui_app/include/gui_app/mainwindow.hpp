#pragma once
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "backend/srv/set_int16.hpp"

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget* parent=nullptr);
  ~MainWindow();

private:
  rclcpp::Node::SharedPtr                                 node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr    executor_;
  QTimer*                                                 ros_timer_;
  
  rclcpp::Client<backend::srv::SetInt16>::SharedPtr       cli_speed_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_cobot_;
  
  // Sensing robot clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_sensing_safetransfer_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_sensing_finished_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_sensing_touch_finished_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_sensing_active_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_sensing_touch_active_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_sensing_slide_command_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_sensing_running_;
  
  // Cleaning robot clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_cleaning_safetransfer_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_cleaning_finished_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_cleaning_active_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_cleaning_slide_command_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr       cli_cleaning_running_;
  
  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr   sub_speed_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_cobot_;
  
  // Sensing subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_sensing_safetransfer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_sensing_finished_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_sensing_touch_finished_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_sensing_active_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_sensing_touch_active_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_sensing_slide_command_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_sensing_running_;
  
  // Cleaning subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_cleaning_safetransfer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_cleaning_finished_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_cleaning_active_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_cleaning_slide_command_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_cleaning_running_;

  // UI Elements - Toggle Buttons
  QPushButton* btnCobotToggle_;
  QPushButton* btnSensingSafeTransferToggle_;
  QPushButton* btnSensingFinishedToggle_;
  QPushButton* btnSensingTouchFinishedToggle_;
  QPushButton* btnSensingActiveToggle_;
  QPushButton* btnSensingTouchActiveToggle_;
  QPushButton* btnSensingSlideCommandToggle_;
  QPushButton* btnSensingRunningToggle_;
  
  QPushButton* btnCleaningSafeTransferToggle_;
  QPushButton* btnCleaningFinishedToggle_;
  QPushButton* btnCleaningActiveToggle_;
  QPushButton* btnCleaningSlideCommandToggle_;
  QPushButton* btnCleaningRunningToggle_;

  void setup_ros();
  void call_speed_set(int value);
  void call_service(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value);
  
  QPushButton* createToggleButton(const QString& label);
  void updateToggleButtonStyle(QPushButton* btn, bool state);
};