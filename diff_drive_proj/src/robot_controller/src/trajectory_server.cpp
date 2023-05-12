#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/get_target.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/parameter.hpp"

#include <cmath>
#include <memory>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;
class TrajectoryServer : public rclcpp::Node
{
	private:
		rclcpp::Service<robot_msgs::srv::GetTarget>::SharedPtr _service; 
	public:
		TrajectoryServer() : Node("trajectory_server")
		{
			_service = this->create_service<robot_msgs::srv::GetTarget>("get_target", std::bind(&TrajectoryServer::handle_get_target, this, _1, _2));
			this -> declare_parameter<int>("trajectory", 1);
			this -> declare_parameter<int>("points", 15);
		}

		void handle_get_target(
		const std::shared_ptr<robot_msgs::srv::GetTarget::Request> request,
		const std::shared_ptr<robot_msgs::srv::GetTarget::Response> response)
		{
			int traj_choice = this -> get_parameter("trajectory").as_int();
			response -> waypoint_next = request -> waypoint + 1;
			RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Incoming request, waypoint: " << request -> waypoint); 
			geometry_msgs::msg::Pose2D temp_pose;
			
			if(traj_choice == 0){  //Square!
				temp_pose.x = 5.0;
				temp_pose.y = 5.0;
				temp_pose.theta = 3.14/2.0;
				if(response -> waypoint_next == 2){
					temp_pose.x = -5.0;
					temp_pose.y = 5.0;
					temp_pose.theta = 3.14/2.0;
				}
				if(response -> waypoint_next == 3){
					temp_pose.x = -5.0;
					temp_pose.y = -5.0;
					temp_pose.theta = 3.14/2.0;
				}
				if(response -> waypoint_next == 4){
					temp_pose.x = 5.0;
					temp_pose.y = -5.0;
					temp_pose.theta = 3.14/2.0;
				}
				if(response -> waypoint_next == 5){
					response -> waypoint_next = 1;
				}
			}

			if(traj_choice == 1){				//Lissajous curve
				int points = this -> get_parameter("points").as_int();
				if(response -> waypoint_next >= points) {
					response -> waypoint_next = 0;
				}
				double t = (double)(response -> waypoint_next) * 2 * M_PI / (double)points;

				double A = 5.0;
				double B = 5.0;
				double a = 1.0;
				double b = 2.0;
				double delta = M_PI/2.0;
				temp_pose.x = A*sin(a*t + delta);
				temp_pose.y = B*sin(b*t);
				temp_pose.theta = 0; //not req
				RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "t: " << t << " x: " << temp_pose.x << " y: " << temp_pose.y);
			}
			
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending response");
			response -> position = temp_pose;
		}
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryServer>());
    rclcpp::shutdown();

}