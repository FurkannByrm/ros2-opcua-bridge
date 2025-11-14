#include "backend/ros_bridge.hpp"
#include "backend/naming.hpp"

RosBridge::RosBridge(const UaConfig& cfg)
: Node("ros2_opcua_bridge"), cfg_{cfg} {
  ua_ = std::make_shared<UaClient>();
  ua_->connect(cfg_);
  ua_->start();

  setup_publishers();
  setup_services();
  }

    
  void RosBridge::setup_publishers(){
      
  rclcpp::QoS qos(10);
  pub_speed_ = this->create_publisher<std_msgs::msg::Int16>("/ros2_comm/speed",qos);
  pub_cobot_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/mod/cobot",qos);

  pub_sensing_robot_home_st_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/home_st",qos);
  pub_sensing_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/finished",qos);
  pub_touch_sensing_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/touch_finished",qos);
  pub_sensing_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/sensing_active",qos);
  pub_touch_sensing_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/touch_active",qos);
  pub_sensing_slide_command_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/slide_command",qos);
  pub_sensing_running_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/running",qos);


  pub_cleaning_robot_home_st_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/home_st",qos);
  pub_cleaning_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/finished",qos);
  pub_cleaning_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/cleaning_active",qos);
  pub_cleaning_slide_command_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/slide_command",qos);
  pub_cleaning_running_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/running",qos);


  ua_->subscribe_int16(cfg_.nodes.speed, [this](int16_t v){
    std_msgs::msg::Int16 msg; 
    msg.data = v;
    pub_speed_->publish(msg);
  });
  
  
  ua_->subscribe_bool(make_child_node(cfg_.structs.mod_root, "COBOT"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cobot_->publish(msg);
  });

  //sensing
ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "robothome_safetransfer"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_robot_home_st_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "sensing-finished"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_finished_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-finished"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_touch_sensing_finished_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "sensing-active"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_active_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-active"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_touch_sensing_active_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "slide_command"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_slide_command_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "running"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_running_->publish(msg);
  });



  //cleaning
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "robothome_safetransfer"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_robot_home_st_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-finished"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_finished_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-active"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_active_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "slide_command"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_slide_command_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "running"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_running_->publish(msg);
  });
  
  

  
}

void RosBridge::setup_services(){
  
  srv_cobot_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/mod/cobot_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.mod_root, "COBOT"), req->data);
        res->success = true;
        res->message = std::string("COBOT set to ") + (req->data ? "true" : "false");
      });

  //sensing service
  srv_sensing_robot_home_st_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/safetransfer_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "robothome_safetransfer"), req->data);
        res->success = true;
        res->message = std::string("robothome_safetransfer set to ") + (req->data ? "true" : "false");
      });

  srv_sensing_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "sensing-finished"), req->data);
        res->success = true;
        res->message = std::string("sensing-finished set to ") + (req->data ? "true" : "false");
      });
  

  srv_touch_sensing_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/touch_finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-finished"), req->data);
        res->success = true;
        res->message = std::string("touchsensing-finished set to ") + (req->data ? "true" : "false");
      });


  srv_sensing_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "sensing-active"), req->data);
        res->success = true;
        res->message = std::string("sensing-active set to ") + (req->data ? "true" : "false");
      });

  srv_touch_sensing_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/touch_active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-active"), req->data);
        res->success = true;
        res->message = std::string("touchsensing-active set to ") + (req->data ? "true" : "false");
      });    


  srv_slide_sensing_command_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/slide_command_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "slide_command"), req->data);
        res->success = true;
        res->message = std::string("slide_command set to ") + (req->data ? "true" : "false");
      });



  srv_running_sensing_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/running",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "running"), req->data);
        res->success = true;
        res->message = std::string("running set to ") + (req->data ? "true" : "false");
      });

  //cleaning service

  srv_cleaning_robot_home_st_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/cobot_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "robothome_safetransfer"), req->data);
        res->success = true;
        res->message = std::string("robothome_safetransfer set to ") + (req->data ? "true" : "false");
      });

  srv_cleaning_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/cleaning_finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-finished"), req->data);
        res->success = true;
        res->message = std::string("cleaning-finished set to ") + (req->data ? "true" : "false");
      });
  

  srv_cleaning_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/cleaning_active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-active"), req->data);
        res->success = true;
        res->message = std::string("cleaning-active set to ") + (req->data ? "true" : "false");
      });

  srv_slide_cleaning_command_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/slide_command_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "slide_command"), req->data);
        res->success = true;
        res->message = std::string("slide_command set to ") + (req->data ? "true" : "false");
      });

  srv_running_cleaning_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/running_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "running"), req->data);
        res->success = true;
        res->message = std::string("running set to ") + (req->data ? "true" : "false");
      });
  
      
  srv_speed_set_ = create_service<backend::srv::SetInt16>(
    "/ros2_comm/speed_set",
    [this](const backend::srv::SetInt16::Request::SharedPtr req,
      backend::srv::SetInt16::Response::SharedPtr res) {
        int16_t new_speed = req->data;
        ua_->enqueue_write_int16(cfg_.nodes.speed, new_speed);
        res->success = true;
        res->message = "Speed updated";
      });
        
        }

        // void RosBridge::setup_subscriptions(){
        
        //   sub_speed_ = this->create_subscription<std_msgs::msg::Int16>("/ros2_comm/speed_cmd",10, [this](const std_msgs::msg::Int16& msg){
        
        //     ua_->enqueue_write_int16(cfg_.nodes.speed, msg.data);
          
        //   });
        
        // }