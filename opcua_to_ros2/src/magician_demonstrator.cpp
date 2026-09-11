#include "opcua_to_ros2/magician_demonstrator.hpp"


OPCuaBridge::OPCuaBridge(const UaConfig& cfg)
: cfg_{cfg} {
  ua_.connect(cfg_);
  ua_.start();
    
  weldingLoad();
}

void OPCuaBridge::weldingLoad(){
    
for(const auto& weld_topic : cfg_.structs.spatter_size){
         
    welding_insertion_order_.push_back(weld_topic);
    welding_lookup_.insert({weld_topic, 0}); 
    welding_load_1<bool>(
            weld_topic,
            [this, weld_topic](bool v){
            std::lock_guard<std::mutex> lock(welding_mutex_);    
            welding_lookup_[weld_topic] = v;
            });
  }


}



CommonDemonstratorInterface::CommonDemonstratorInterface(std::shared_ptr<OPCuaBridge> opc_srv) : 
Node{"demonstrator_common_node"},
opc_srv_{opc_srv} 
{
    modSelection(rclcpp::QoS(20));
}
void CommonDemonstratorInterface::modSelection(rclcpp::QoS qos){

  pub_cobot_mode_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/mod/cobot",qos.reliable());
  pub_automatic_mode_= this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/mod/automatic",qos.reliable());
      


  opc_srv_->get_mod<bool>("COBOT",[this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cobot_mode_->publish(msg);
  });

  opc_srv_->get_mod<bool>("FULLY AUTOMATIC",[this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cobot_mode_->publish(msg);
  });

  srv_cobot_mode_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/mod/cobot_mode_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_mod<bool>("COBOT",req->data);     
        res->success = true;
        res->message = std::string("COBOT set to ") + (req->data ? "true" : "false");
      });

 srv_full_automatic_mode_set_ =create_service<std_srvs::srv::SetBool>(
        "/ros2_comm/mod/full_automatic_mode_set",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
            std_srvs::srv::SetBool::Response::SharedPtr res){
        opc_srv_->set_mod<bool>("FULLY AUTOMATIC",req->data);
        res->success =true;
        res->message = std::string("Full Automatic Mod set to ") + (req->data ? "true" : "false");
        });


 get_welding_data_ = create_service<opcua_to_ros2::srv::GetSpotWeights>(
         "/ros2_comm/get_welding_spot_size",
         [this](const opcua_to_ros2::srv::GetSpotWeights::Request::SharedPtr req,
                opcua_to_ros2::srv::GetSpotWeights::Response::SharedPtr res){
            res->weldings.clear();
    std::lock_guard<std::mutex> lock(opc_srv_->welding_mutex_);
         for(const auto& spot_name : opc_srv_->welding_insertion_order_){
            auto it = opc_srv_->welding_lookup_.find(spot_name);
        if (it != opc_srv_->welding_lookup_.end() && it->second == true) {
            opcua_to_ros2::msg::Welding weld;
            weld.welding_name = spot_name;
            res->weldings.push_back(weld);
        }        
           }
        });


}


Sensing::Sensing(std::shared_ptr<OPCuaBridge> opc_srv) : Node{"sensing_demonsrator_node"}, opc_srv_{opc_srv}{
    sensingPublishers(rclcpp::QoS(20));
    sensingServices();
}

void Sensing::sensingPublishers(rclcpp::QoS qos){


  pub_sensing_robot_home_st_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/home_st",qos.best_effort());
  pub_sensing_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/finished",qos.best_effort());
  pub_touch_sensing_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/touch_finished",qos.best_effort());
  pub_sensing_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/sensing_active",qos.best_effort());
  pub_touch_sensing_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/touch_active",qos.best_effort());
  pub_sensing_running_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/running",qos.best_effort());
  pub_sensing_carbody_located_st_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/carbody_located_status", qos.best_effort());
  pub_sensing_slider_actual_pos_ =this->create_publisher<std_msgs::msg::Float32>("/ros2_comm/sensing/slider_actual_pos",qos.best_effort()); 

  pub_sensing_position_2_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos2_status",qos.best_effort());
  pub_sensing_position_3_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos3_status",qos.best_effort());
  pub_sensing_position_4_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos4_status",qos.best_effort());
  
  pub_sensing_position_5_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos5_status",qos.best_effort());


  pub_sensing_position_2_reached_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos2_status_reached",qos.best_effort());
  pub_sensing_position_3_reached_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos3_status_reached",qos.best_effort());
  pub_sensing_position_4_reached_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos4_status_reached",qos.best_effort());
  
  pub_sensing_position_5_reached_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/pos5_status_reached",qos.best_effort());





  opc_srv_->get_workcell<double>("Slider_1_actual position-linear", [this](double v){
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(v);   
    pub_sensing_slider_actual_pos_->publish(msg);
  });

opc_srv_->get_sensing<bool>("robothome_safetransfer", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_robot_home_st_->publish(msg);
  });

opc_srv_->get_sensing<bool>("sensing-finised", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_finished_->publish(msg);
  });

opc_srv_->get_sensing<bool>("touchsensing-finished", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_touch_sensing_finished_->publish(msg);
  });

opc_srv_->get_sensing<bool>("sensing-active", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_active_->publish(msg);
  });

  opc_srv_->get_sensing<bool>("touchsensing-active", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_touch_sensing_active_->publish(msg);
  });
  opc_srv_->get_sensing<bool>("running", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_running_->publish(msg);
  });
    
    opc_srv_->get_sensing<bool>("Car_Poss_Ok", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_carbody_located_st_->publish(msg);
    });

    
    opc_srv_->get_sensing<bool>("Sensing_Pos_2", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_2_->publish(msg);
    });


    opc_srv_->get_sensing<bool>("Sensing_Pos_3", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_3_->publish(msg);
    });


    opc_srv_->get_sensing<bool>("Sensing_Pos_4", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_4_->publish(msg);
    });

    
    opc_srv_->get_sensing<bool>("Sensing_Pos_5", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_5_->publish(msg);
    });

    
    opc_srv_->get_sensing<bool>("Sensing_Pos_2_Enable", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_2_reached_->publish(msg);
    });


    opc_srv_->get_sensing<bool>("Sensing_Pos_3_Enable", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_3_reached_->publish(msg);
    });


    opc_srv_->get_sensing<bool>("Sensing_Pos_4_Enable", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_4_reached_->publish(msg);
    });

    
    opc_srv_->get_sensing<bool>("Sensing_Pos_5_Enable", [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_position_5_reached_->publish(msg);
    });





}


void Sensing::sensingServices(){


  srv_sensing_robot_home_st_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/safetransfer_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("robothome_safetransfer", req->data);
        res->success = true;
        res->message = std::string("robothome_safetransfer set to ") + (req->data ? "true" : "false");
      });

  srv_sensing_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("sensing-finised", req->data);
        res->success = true;
        res->message = std::string("sensing-finished set to ") + (req->data ? "true" : "false");
      });
  

  srv_touch_sensing_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/touch_finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("touchsensing-finished", req->data);
        res->success = true;
        res->message = std::string("touchsensing-finished set to ") + (req->data ? "true" : "false");
      });


  srv_sensing_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("sensing-active", req->data);
        res->success = true;
        res->message = std::string("sensing-active set to ") + (req->data ? "true" : "false");
      });

  srv_touch_sensing_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/touch_active_set",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("touchsensing-active", req->data);
        res->success = true;
        res->message = std::string("touchsensing-active set to ") + (req->data ? "true" : "false");
      });    


  srv_running_sensing_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/running",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("running", req->data);
        res->success = true;
        res->message = std::string("running set to ") + (req->data ? "true" : "false");
      });


  srv_sensing_pos2_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/pos2_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("Sensing_Pos_2", req->data);
        res->success = true;
        res->message = std::string("sensing-position set to ") + (req->data ? "true" : "false");
      });

  srv_sensing_pos3_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/pos3_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("Sensing_Pos_3", req->data);
        res->success = true;
        res->message = std::string("sensing-position set to ") + (req->data ? "true" : "false");
      });

  srv_sensing_pos4_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/pos4_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("Sensing_Pos_4", req->data);
        res->success = true;
        res->message = std::string("sensing-position set to ") + (req->data ? "true" : "false");
      });

    
  srv_sensing_pos5_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/pos5_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
       opc_srv_->set_sensing<bool>("Sensing_Pos_5", req->data);
        res->success = true;
        res->message = std::string("sensing-position set to ") + (req->data ? "true" : "false");
      });
}



    
Cleaning::Cleaning(std::shared_ptr<OPCuaBridge> opc_srv) : Node{"cleaning_demonsrator_node"}, opc_srv_{opc_srv}{
    cleaningPublishers(rclcpp::QoS(20));
    cleaningServices();
}

void Cleaning::cleaningPublishers(rclcpp::QoS qos){
    
  pub_cleaning_robot_home_st_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/home_st",qos.best_effort());
  pub_cleaning_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/finished",qos.best_effort());
  pub_cleaning_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/cleaning_active",qos.best_effort());
  pub_cleaning_running_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/running",qos.best_effort());
  pub_cleaning_carbody_located_st_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/carbody_located_status",qos.best_effort());
  pub_cleaning_slider_actual_pos_ =this->create_publisher<std_msgs::msg::Float32>("/ros2_comm/cleaning/slider_actual_pos",qos.best_effort());


  pub_cleaning_position_2_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos2_status", qos.best_effort());
  pub_cleaning_position_3_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos3_status", qos.best_effort());
  pub_cleaning_position_4_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos4_status", qos.best_effort());
  pub_cleaning_position_5_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos5_status", qos.best_effort());



  pub_cleaning_position_2_reached_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos2_status_reached", qos.best_effort());
  pub_cleaning_position_3_reached_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos3_status_reached", qos.best_effort());
  pub_cleaning_position_4_reached_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos4_status_reached", qos.best_effort());
  pub_cleaning_position_5_reached_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/pos5_status_reached", qos.best_effort());



  opc_srv_->get_workcell<double>("Slider_2_actual position-linear",[this](double v){
          std_msgs::msg::Float32 msg;
          msg.data = static_cast<float>(v);
          pub_cleaning_slider_actual_pos_->publish(msg);    
          });
  opc_srv_->get_cleaning<bool>("robothome_safetransfer", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_robot_home_st_->publish(msg);
  });
  opc_srv_->get_cleaning<bool>("cleaning-finished", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_finished_->publish(msg);
  });
  opc_srv_->get_cleaning<bool>("cleaning-active", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_active_->publish(msg);
  });
  opc_srv_->get_cleaning<bool>("running", [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_running_->publish(msg);
  });
  opc_srv_->get_cleaning<bool>("Car_Pos_Ok",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_carbody_located_st_->publish(msg);    
  });

  opc_srv_->get_cleaning<bool>("Cleaning_Pos_2",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_2_->publish(msg);
  });


  opc_srv_->get_cleaning<bool>("Cleaning_Pos_3",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_3_->publish(msg); 
  });

    
  opc_srv_->get_cleaning<bool>("Cleaning_Pos_4",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_4_->publish(msg); 
  });
    

  opc_srv_->get_cleaning<bool>("Cleaning_Pos_5",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_5_->publish(msg); 
  });



  opc_srv_->get_cleaning<bool>("Cleaning_Pos_2_Enable",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_2_reached_->publish(msg);
  });


  opc_srv_->get_cleaning<bool>("Cleaning_Pos_3_Enable",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_3_reached_->publish(msg); 
  });

    
  opc_srv_->get_cleaning<bool>("Cleaning_Pos_4_Enable",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_4_reached_->publish(msg); 
  });
    

  opc_srv_->get_cleaning<bool>("Cleaning_Pos_5_Enable",[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_position_5_reached_->publish(msg); 
  });




}

    void Cleaning::cleaningServices(){


  srv_cleaning_robot_home_st_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/safetransfer_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("robothome_safetransfer", req->data);
        res->success = true;
        res->message = std::string("robothome_safetransfer set to ") + (req->data ? "true" : "false");
      });

  srv_cleaning_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/cleaning_finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("cleaning-finished", req->data);
        res->success = true;
        res->message = std::string("cleaning-finished set to ") + (req->data ? "true" : "false");
      });
  

  srv_cleaning_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/cleaning_active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("cleaning-active", req->data);
        res->success = true;
        res->message = std::string("cleaning-active set to ") + (req->data ? "true" : "false");
      });

  srv_running_cleaning_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/running_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("running", req->data);
        res->success = true;
        res->message = std::string("running set to ") + (req->data ? "true" : "false");
      });


    srv_cleaning_pos2_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/pos2_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("Cleaning_Pos_2", req->data);
        res->success = true;
        res->message = std::string("cleaning-position set to ") + (req->data ? "true" : "false");
      });


    srv_cleaning_pos3_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/pos3_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("Cleaning_Pos_3", req->data);
        res->success = true;
        res->message = std::string("cleaning-position set to ") + (req->data ? "true" : "false");
      });


    srv_cleaning_pos4_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/pos4_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("Cleaning_Pos_4", req->data);
        res->success = true;
        res->message = std::string("cleaning-position set to ") + (req->data ? "true" : "false");
      });


    srv_cleaning_pos5_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/pos5_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        opc_srv_->set_cleaning<bool>("Cleaning_Pos_5", req->data);
        res->success = true;
        res->message = std::string("cleaning-position set to ") + (req->data ? "true" : "false");
      });
    }

