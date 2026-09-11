#include <chrono>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char * argv[]){

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("cleaning_send_goal");
    auto node_cleaning_off = std::make_shared<rclcpp::Node>("cleaning_finished_command");
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node);
    exe.add_node(node_cleaning_off);
    auto cleaning_activation_request = std::make_shared<std_srvs::srv::SetBool::Request>();  
    // const auto timeout = std::chrono::seconds(8);
    cleaning_activation_request->data = false;
    auto action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node,"/cleaning_controller/follow_joint_trajectory");
    auto cleaning_finished_client = node_cleaning_off->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_active_set"); 


if(!action_client->wait_for_action_server(std::chrono::seconds(20))){
    
    RCLCPP_ERROR(node->get_logger(),"Action server not available after waiting");
    return 1;
}




auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal(); 
goal_msg.trajectory.joint_names = {"cleaning_joint_1","cleaning_joint_2","cleaning_joint_3","cleaning_joint_4","cleaning_joint_5","cleaning_joint_6"};

trajectory_msgs::msg::JointTrajectoryPoint point;
point.positions = {0.0, -1.05, 1.83, 0.0, -0.78, 0.0};
point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
point.time_from_start = rclcpp::Duration::from_seconds(5.0);

goal_msg.trajectory.points.push_back(point);
auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.result_callback = 
        [&](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node->get_logger(), "Goal succeeded!");
                
    auto cleaning_future = cleaning_finished_client->async_send_request(cleaning_activation_request);
     
            rclcpp::shutdown();

            } else {
                RCLCPP_ERROR(node->get_logger(), "Goal failed with status: %d", static_cast<int>(result.code));
                rclcpp::shutdown();
            }
        };

    RCLCPP_INFO(node->get_logger(), "Sending goal...");



    action_client->async_send_goal(goal_msg,send_goal_options);
exe.spin();
rclcpp::shutdown();
       

return 0;
}
