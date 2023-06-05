#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
//Go into ros2 install -> pkg -> include folder to find exact file names
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"

/*
TODO: 
Be able to extract parameters from gazebo plugin to input as headers,
parameter joints should be ordered properly?
*/


class ControlNode : public rclcpp::Node
{
	private:
		rclcpp::TimerBase::SharedPtr _timer; 

		//in
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr _poseArraySubscriber;
		std::map<std::string, double> _jointPositionMap;
		geometry_msgs::msg::Pose _currentPose;
		double _currentPoseTime;
		double _previousPoseTime;

		//out
		std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> _jointControlPublishersMap;
		std::map<std::string, double> _jointTorqueControlMap;

		//data extraction
		std::mutex _mutex;
		std::string _dataHeaders;
		std::ofstream _csvFile;
        std::ofstream _csvWriter;
        // int _csv_writer_buffer; //Buffer if data comes in too fast, not fully implemented yet
	public:
		ControlNode() : Node("control_node")
		{
			//Timer for publication
			_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));

			//params
			auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
			param_desc.description = "Joint names to be controlled";
			//Doesn't seem to be map parameter avail, so use 2 vectors
			this->declare_parameter<std::vector<std::string>>("joint_names", {"placeholder"}, param_desc);
			this->declare_parameter<std::vector<std::string>>("joint_control_topics", {"placeholder"});
			this->declare_parameter<std::string>("position_topic", "/world/world1/dynamic_pose/info");
			this->declare_parameter<int>("control_publish_frequency", -1);
			this->declare_parameter<std::string>("data_file_path", "../data/data.csv");

			//creating joint publishers and mapping joints
			std::vector<std::string> jointNames = this->get_parameter("joint_names").as_string_array();
			std::vector<std::string> jointControlTopics = this->get_parameter("joint_control_topics").as_string_array();
			int jointNameSize = jointNames.size();
			int jointControlTopicSize = jointControlTopics.size();
			if (jointNameSize != jointControlTopicSize) {
				RCLCPP_ERROR(this->get_logger(), "Joint name and control topic size mismatch");
				return;
			}
			for (int i = 0; i < jointNameSize; i++) {
				_jointControlPublishersMap[jointNames[i]] = this->create_publisher<std_msgs::msg::Float64>(jointControlTopics[i], 10);
				_jointTorqueControlMap[jointNames[i]] = 0.0;
				_jointPositionMap[jointNames[i]] = 0.0;
			}

			//Subscriber for Position
			_poseArraySubscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
				this->get_parameter("position_topic").as_string(),
				10, std::bind(&ControlNode::position_topic_callback,
				this, std::placeholders::_1));

			//Data extraction setup
			std::string dataFilePath = this->get_parameter("data_file_path").as_string();
			_csvFile.open(dataFilePath);
            _csvWriter = std::ofstream(dataFilePath, std::ios::out | std::ios::app);
			_dataHeaders = "time,position_x,position_y,position_z,quaternion_x,quaternion_y,quaternion_z,quaternion_w";
			for (const auto & jointName : jointNames) {
				_dataHeaders += "," + jointName;
			}
			_csvWriter << _dataHeaders << "\n";
            // _csv_writer_buffer = 0;
		}

	private:
		void position_topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
		{
			std::lock_guard<std::mutex> lock(_mutex);
			RCLCPP_INFO_STREAM(this->get_logger(), "Received position topic time: " << msg->header.stamp.sec << " " << msg->header.stamp.nanosec);
			_currentPose = msg->poses[0]; 
			int pose_size = msg->poses.size();

			//Time updates
			_previousPoseTime = _currentPoseTime;
			_currentPoseTime = msg->poses[pose_size - 1].position.x; //Time is stored as last pose obj in plugin, due to missing header
			_csvWriter << _currentPoseTime << "," << _currentPose.position.x << "," << _currentPose.position.y << "," << _currentPose.position.z << ","
						<< _currentPose.orientation.x << "," << _currentPose.orientation.y << "," << _currentPose.orientation.z << "," << _currentPose.orientation.w;
			
			if(pose_size != static_cast<int>(_jointPositionMap.size()) + 2) { //+2, base pose + time pose
				RCLCPP_ERROR(this->get_logger(), "Incoming Pose size mismatch with joint map size");
				return;
			}
			//Update joint angle values
			int i = 0;
			for (auto joint = _jointPositionMap.begin(); joint != _jointPositionMap.end(); joint++, i++) {
				joint->second = msg->poses[i + 1].position.x;  //skip base pose 
				_csvWriter << "," << joint->second;
			}
			_csvWriter << "\n";
		}

		void timer_callback(){ 
			//Control loop
			sin_torque_control();
			
			//Publishes joint torques
			auto message = std_msgs::msg::Float64();
			for (auto & joint : _jointTorqueControlMap) {
				message.data = joint.second;
				_jointControlPublishersMap[joint.first]->publish(message);
			}
		}
		
		void sin_torque_control(){
			//hardcoded params
			double amplitude = 20;
			double frequency = 1;
			double phase = 0;
			
			for (auto & joint : _jointTorqueControlMap) {
				joint.second = amplitude * std::sin(2.0 * M_PI * frequency * _currentPoseTime + phase);
			} 
		}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControlNode>());
	rclcpp::shutdown();
	return 0;
}
