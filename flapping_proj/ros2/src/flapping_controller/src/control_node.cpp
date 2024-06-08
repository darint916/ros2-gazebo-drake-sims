#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
//Go into ros2 install -> pkg -> include folder to find exact file names
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "flapping_controller/pid_controller.hpp"
/*
TODO: 
Be able to extract parameters from gazebo plugin to input as headers,
parameter joints should be ordered properly?
Add launchfile params here somewhere
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

		//Control diff drive vel 
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _diffDriveVelPublisher;

		//data extraction
		std::mutex _mutex;
		std::string _dataHeaders;
		std::ofstream _csvFile;
        std::ofstream _csvWriter;
        // int _csv_writer_buffer; //Buffer if data comes in too fast, not fully implemented yet
	
		//Altitude PID controller
		PIDController _altitude_pid;
		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _altitude_subscriber;
		double _altitude_target;
		int _error_buffer;
		std::string _dataHeadersPID;
		std::ofstream _csvFilePID;
        std::ofstream _csvWriterPID;

		bool _motor_torque_calc_enabled;
		double _max_voltage;
		double _motor_resistance;
		double _motor_torque_constant;
		double _sim_length;
		std::string _kill_flag_file_path;

	public:
		ControlNode() : Node("control_node")
		{

			//params
			auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
			param_desc.description = "Joint names to be controlled";
			//Doesn't seem to be map parameter avail, so use 2 vectors
			this->declare_parameter<std::vector<std::string>>("joint_names", {"placeholder"}, param_desc);
			this->declare_parameter<std::vector<std::string>>("joint_control_topics", {"placeholder"});
			this->declare_parameter<std::string>("position_topic", "/world/world1/dynamic_pose/info");
			this->declare_parameter<int>("control_publish_frequency", 30);
			this->declare_parameter<std::string>("data_file_path", "../data/data.csv");

			//Velocity publisher (uses diff drive plugin receiver)
			_diffDriveVelPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/world/diff_drive/cmd_vel", 10);
			this->declare_parameter<double>("amplitude", 10);
			this->declare_parameter<double>("frequency", 10);

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
			//Timer for publication
			auto publishPeriod = std::chrono::duration<double>(1.0 / this->get_parameter("control_publish_frequency").as_int());
			_timer = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(publishPeriod), std::bind(&ControlNode::timer_callback, this));
			
			//Subscriber for Position
			RCLCPP_INFO_STREAM(this->get_logger(), "Subscribing to position topic: " << this->get_parameter("position_topic").as_string());
			_poseArraySubscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
				this->get_parameter("position_topic").as_string(),
				1000, std::bind(&ControlNode::position_topic_callback,
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

			//Altitude PID controller
			this->declare_parameter<bool>("altitude_pid_enabled", false);
			this->declare_parameter<bool>("pid_data_enabled", false);
			this->declare_parameter<std::string>("pid_data_file_path", "../data/pid_data.csv");
			this->declare_parameter<double>("altitude_kp", 0.0);
			this->declare_parameter<double>("altitude_ki", 0.0);
			this->declare_parameter<double>("altitude_kd", 0.0);
			rcl_interfaces::msg::ParameterDescriptor altitude_max_pid_output_desc;
			altitude_max_pid_output_desc.description = "Max output of PID controller, will be added to base signal";
			this->declare_parameter<double>("altitude_max_pid_output", 0.0, altitude_max_pid_output_desc);
			this->declare_parameter<double>("altitude_max_integral", 0.0);
			_altitude_pid = PIDController(
				this->get_parameter("altitude_kp").as_double(),
				this->get_parameter("altitude_ki").as_double(),
				this->get_parameter("altitude_kd").as_double(),
				this->get_parameter("altitude_max_pid_output").as_double(),
				this->get_parameter("altitude_max_integral").as_double()
			);
			//subscriber to dynamic altitude topic
			_altitude_subscriber = this->create_subscription<std_msgs::msg::Float64>(
				"/control/altitude",
				1000, std::bind(&ControlNode::altitude_topic_callback,
				this, std::placeholders::_1));
			//static altitude

			this->declare_parameter<double>("static_altitude", 4.0);
			if(this->has_parameter("static_altitude")){
				_altitude_target = this->get_parameter("static_altitude").as_double();
			} else {
				RCLCPP_INFO_STREAM(this->get_logger(), "No static altitude parameter found, using default: 4");
				_altitude_target = 4.0;
			}	
			_error_buffer = 0;

			std::string pidDataFilePath = this->get_parameter("pid_data_file_path").as_string();
			_csvFilePID.open(pidDataFilePath);
			_csvWriterPID = std::ofstream(pidDataFilePath, std::ios::out | std::ios::app);
			_dataHeadersPID = "time,altitude_target,altitude_error,altitude_output";
			_csvWriterPID << _dataHeadersPID << "\n";


			//optimization sector
			this->declare_parameter<bool>("motor_torque_calc_enabled", false);
			this->declare_parameter<double>("max_voltage", 6.0);
			this->declare_parameter<double>("motor_resistance", 1.0);
			this->declare_parameter<double>("motor_torque_constant", 1.0);
			this->declare_parameter<double>("sim_length", 5.0);
			this->declare_parameter<std::string>("kill_flag_path", "../optimization/kill_flag.txt");
			_motor_torque_calc_enabled = this->get_parameter("motor_torque_calc_enabled").as_bool();
			_max_voltage = this->get_parameter("max_voltage").as_double();
			_motor_resistance = this->get_parameter("motor_resistance").as_double();
			_motor_torque_constant = this->get_parameter("motor_torque_constant").as_double();
			_sim_length = this->get_parameter("sim_length").as_double();
			_kill_flag_file_path = this->get_parameter("kill_flag_path").as_string();
			
		}

	private:
		void position_topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
		{
			std::lock_guard<std::mutex> lock(_mutex);
			// RCLCPP_INFO_STREAM(this->get_logger(), "Received position topic time: " << msg->header.stamp.sec << " " << msg->header.stamp.nanosec);
			_currentPose = msg->poses[0]; 
			int pose_size = msg->poses.size();

			//Time updates
			_previousPoseTime = _currentPoseTime;
			if (_currentPoseTime == _sim_length) {
				std::ofstream kill_flag_file(_kill_flag_file_path);
				kill_flag_file << 1;
				kill_flag_file.close();
				RCLCPP_INFO_STREAM(this->get_logger(), "Data written to file");
				rclcpp::shutdown();
			}
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
			//Control loop for choosing what type of control
			sin_torque_control();
			if(this->get_parameter("altitude_pid_enabled").as_bool()){
				altitude_pid_control();
			} 
			if(this->get_parameter("motor_torque_calc_enabled").as_bool()){
				motor_torque_output();
			}
			//Publishes joint torques
			auto message = std_msgs::msg::Float64();
			// int flag = 0;
			// auto msg = geometry_msgs::msg::Twist();

			//Switch joint axis? instead of flipping torque
			for (auto & joint : _jointTorqueControlMap) {
				if (joint.first == "joint_RW_J_Flap"){ //edit to change dir later?
					joint.second *= -1;
				}
				message.data = joint.second;
				_jointControlPublishersMap[joint.first]->publish(message);
				
				
				// msg.linear.x = joint.second; //same direction spin
				// msg.angular.z = joint.second;
				//Execute once 
				// if(flag == 0){
				// 	_diffDriveVelPublisher->publish(msg);
				// 	flag = 1;
				// }
			}

			
		}

		void sin_torque_control(){
			//hardcoded params
			// double amplitude = 10;
			// double frequency = 10;
			double amplitude = this->get_parameter("amplitude").as_double();
			double frequency = this->get_parameter("frequency").as_double();
			double phase = 0;
			// RCLCPP_INFO_STREAM(this->get_logger(), "Current time: " << _currentPoseTime);
			for (auto & joint : _jointTorqueControlMap) {
				joint.second = amplitude * std::sin(2.0 * M_PI * frequency * _currentPoseTime + phase);
				
				//square wave (min max)
				// if (joint.second < 0) {
				// 	joint.second = -amplitude;
				// } else {
				// 	joint.second = amplitude;
				// }

			} 
		}
		
		void altitude_pid_control(){
			double error = _altitude_target - _currentPose.position.z;
			double dt = _currentPoseTime - _previousPoseTime;
			double output = _altitude_pid.calculate(error, dt);
			// _error_buffer += 1;
			// if (_error_buffer > 10000){
			// 	RCLCPP_INFO_STREAM(this->get_logger(), "Altitude PID error: " << error);
			// 	RCLCPP_ERROR_STREAM(this->get_logger(), "Altitude PID output: " << output);
			// 	_error_buffer = 0;
			// }

			//Adds onto base amplitude from sin signal
			double amplitude = this->get_parameter("amplitude").as_double();
			for (auto & joint : _jointTorqueControlMap) {
				if(amplitude + output <= 0){
					joint.second = amplitude * .3; //allows for negative pid to map to minimal (but visible) force (lets spring dampening kill altitude)
				} else if(amplitude != 0){ //Applies amplitude to output properly (keeps sin)
					joint.second = (joint.second / amplitude) * (amplitude + output);
				}
			}
			if(this->get_parameter("pid_data_enabled").as_bool()){
				_csvWriterPID << _currentPoseTime << "," << _altitude_target << "," << error << "," << output << "\n";
			}
		}

		void altitude_topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
		{
			_altitude_target = msg->data;
		}

		void motor_torque_output (){
			//TODO 
			// GET OMEGA VALUE
			double amplitude = this->get_parameter("max_voltage").as_double();
			double frequency = this->get_parameter("frequency").as_double();
			double voltage = amplitude * std::sin(2.0 * M_PI * frequency * _currentPoseTime);
			double motor_resistance = this->get_parameter("motor_resistance").as_double();
			double motor_torque_constant = this->get_parameter("motor_torque_constant").as_double();
			for (auto & joint : _jointTorqueControlMap) {
				//torque = (u - k_a * omega) * k_a / r_a
				joint.second = (voltage - motor_torque_constant * 2) * motor_torque_constant / motor_resistance;
				// joint.second = 1; //test

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
