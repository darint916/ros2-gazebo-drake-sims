#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/get_target.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"


// #include <drake/tra

#include <memory>
#include <vector>
using std::placeholders::_1;
// class TrajectoryServer : public rclcpp::Node
// {
// 	private:
// 		rclcpp::Service<robot_msgs::srv::GetTarget>::SharedPtr _service; 
// 	public:
// 		TrajectoryServer() : Node("trajectory_server")
// 		{
// 			//no bind for additional arguments
// 			_service = this->create_service<robot_msgs::srv::GetTarget>("get_target", &TrajectoryServer::handle_get_target);

// 			//Build trajectory 
// 			const int samples = 200;
// 			// const double delta_s = traj;
// 		}

// 		void handle_get_target(
// 		const std::shared_ptr<robot_msgs::srv::GetTarget::Request> request,
// 		const std::shared_ptr<robot_msgs::srv::GetTarget::Response> response)
// 		{
// 			//might need to fix casting? ros2->c++
// 			response -> waypoint_next = request -> waypoint + 1;
// 			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
// 			//temp hardcode pose
// 			geometry_msgs::msg::Pose2D temp_pose;
// 			if(request -> waypoint % 2 == 0){
// 				temp_pose.x = 5.0;
// 				temp_pose.y = 0.0;
// 				temp_pose.theta = 0.0;
// 			}
// 			else{
// 				temp_pose.x = 0.0;
// 				temp_pose.y = 0.0;
// 				temp_pose.theta = 0.0;
// 			}
// 			response -> position = temp_pose;
// 		}
// };


void handle_get_target(
		const std::shared_ptr<robot_msgs::srv::GetTarget::Request> request,
		const std::shared_ptr<robot_msgs::srv::GetTarget::Response> response)
		{
			//might need to fix casting? ros2->c++
			response -> waypoint_next = request -> waypoint + 1;
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
			//temp hardcode pose
			geometry_msgs::msg::Pose2D temp_pose;
			if(request -> waypoint % 2 == 0){
				temp_pose.x = 5.0;
				temp_pose.y = 0.0;
				temp_pose.theta = 0.0;
			}
			else{
				temp_pose.x = 0.0;
				temp_pose.y = 0.0;
				temp_pose.theta = 0.0;
			}
			response -> position = temp_pose;
		}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("trajectory_server");
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to get trajectory target.");
	auto service = node->create_service<robot_msgs::srv::GetTarget>("get_target", &handle_get_target);
    // rclcpp::spin(std::make_shared<TrajectoryServer>());
    rclcpp::spin(node);
    rclcpp::shutdown();

}