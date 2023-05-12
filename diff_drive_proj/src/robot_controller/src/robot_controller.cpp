#include <chrono>
#include <memory>
#include <string>
#include <functional>
// #include <drake/math>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/differential_in.hpp"
#include "rclcpp/parameter.hpp"
#include "robot_msgs/msg/motor_speeds.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/srv/get_target.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "robot_controller/pid_controller.hpp"

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotController : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
        rclcpp::Client<robot_msgs::srv::GetTarget>::SharedPtr _client;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr _subscription2;
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _clock_sub;
        int _current_waypoint;
        geometry_msgs::msg::Pose2D _target_pose;

        PIDController _linear_pid;
        PIDController _angular_pid;
        PIDController _linear_weight_pid;
        
        double _prev_time;
        double _curr_time;
        bool _next_point_received;

        double _ang_vel_l;
        double _ang_vel_r;

        std::ofstream _csv_file;
        std::ofstream _csv_writer;
        int _csv_writer_buffer;

    public:
        RobotController() : Node("robot_controller")
        {
            //Declares parameters
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "The distance between the wheels of the robot, pos, angle";
            this -> declare_parameter<double>("wheel_radius", 0.0);
            this -> declare_parameter<double>("wheel_distance", 0.0);
            this -> declare_parameter<double>("pos", 0.0);
            this -> declare_parameter<double>("angle", 0.0);
            this -> declare_parameter<int>("csv_buffer_skip_length", 30);

            //Publishes to "motor_speeds" topic with a queue size of 10
            _publisher = this->create_publisher<geometry_msgs::msg::Twist>("motor_speeds", 10);

            //Subscribes to "diff_drive" topic with a queue size of 10
            // _subscription = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotController::topic_robot_input_callback, this, _1));
            _subscription2 = this->create_subscription<tf2_msgs::msg::TFMessage>("world/car_world/dynamic_pose/info", 10, std::bind(&RobotController::topic_pose_input_callback, this, _1));
            _clock_sub = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&RobotController::topic_clock_callback, this, _1));
            _prev_time = 0.0;
            _curr_time = 0.0;
            
            _client = this->create_client<robot_msgs::srv::GetTarget>("get_target");

            _current_waypoint = 0;
            _target_pose.x = 0;
            _target_pose.y = 0;
            _target_pose.theta = 0;

            //integrators
            // _linear_pid = PIDController(3, 1, 1, 2, 2.0);
            // _linear_weight_pid = PIDController(.5, 0.1, 0.1, 1.0, 1.0);
            // _angular_pid = PIDController(3, 1, 1, 4, 4.0);

            //dir
            _linear_pid = PIDController(3.3, 0.5, 0.6, 1.0, 1.0);
            _angular_pid = PIDController(2, 0.4, 0.6, 2, 1.0);

            _next_point_received = false;
            
            _ang_vel_l = 0.0;
            _ang_vel_r = 0.0;
            
            _csv_file.open("../data/data.csv"); //should add as parameter path
            _csv_writer = std::ofstream("../data/data.csv", std::ios::out | std::ios::app);
            _csv_writer_buffer = 0;
        }

    private:
        void topic_clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg){
            _prev_time = _curr_time;
            _curr_time = msg->clock.sec + msg->clock.nanosec * 1e-9;
        }

        void topic_pose_input_callback(const tf2_msgs::msg::TFMessage & msg){
            
            double time_diff = _curr_time - _prev_time;
            time_diff = 1;
            double curr_cart_x = msg.transforms[0].transform.translation.x;
            double curr_cart_y = msg.transforms[0].transform.translation.y;

            double curr_quat_w = msg.transforms[0].transform.rotation.w;
            double curr_quat_z = msg.transforms[0].transform.rotation.z;
            
            double wheel_radius = this -> get_parameter("wheel_radius").as_double();
            double wheel_distance = this -> get_parameter("wheel_distance").as_double();

            double curr_euler_angle = 2 * atan2(curr_quat_z, curr_quat_w); //Finds Yaw, multiply by 2 because of the  quaternion works

            double dx = _target_pose.x - curr_cart_x;
            double dy = _target_pose.y - curr_cart_y;

            double error_dist = sqrt(dx*dx + dy*dy);
            
            double angle_to_target = atan2(dy, dx);
            double error_angle = angle_to_target - curr_euler_angle;
            //Keeps error angle between -pi and pi
            // RCLCPP_INFO_STREAM(this->get_logger(), "ANGLE ERRORS\n" << "error_angle: " << error_angle );
            while (error_angle > M_PI) error_angle -= 2 * M_PI;
            while (error_angle < -M_PI) error_angle += 2 * M_PI;
            
            RCLCPP_INFO_STREAM(this->get_logger(), "curr_cart_x: " << curr_cart_x << " curr_cart_y: " << curr_cart_y);
            RCLCPP_INFO_STREAM(this->get_logger(), "target_x: " << _target_pose.x << " target_y: " << _target_pose.y);


            double angVelLeft = 0;
            double angVelRight = 0;
            if(abs(error_angle) > .2 && error_dist > .15) { //error dist cuz angle error goes up in close proximity, larger tolerance to reach dest faster
                RCLCPP_INFO_STREAM(this->get_logger(), "ANGLE ERRORS\n" << "error_angle: " << error_angle << " Target: " << angle_to_target << " Current: " << curr_euler_angle); 
                RCLCPP_INFO_STREAM(this->get_logger(), "DIST ERRORS\n" << "error_dist: " << error_dist << "Target: " << _target_pose.x << "Current: " << curr_cart_x); 
                double angular_velocity = _angular_pid.calculate(error_angle, time_diff);
                angVelLeft = (angular_velocity * wheel_distance / 2) / wheel_radius;
                angVelRight = -angVelLeft;
            }else {
                RCLCPP_INFO_STREAM(this->get_logger(), "ANGLE THRESHOLD PASSED\n" << "error_angle: " << error_angle << " Target: " << angle_to_target << " Current: " << curr_euler_angle);
                if(abs(error_angle) > .05 && error_dist > .15){ //Fine tune angle while traversing
                    double angular_velocity = _angular_pid.calculate(error_angle, time_diff);
                    angVelLeft = (angular_velocity * wheel_distance / 2) / wheel_radius;
                    angVelRight = -angVelLeft;
                }
                if(error_dist > .09) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "DIST ERRORS\n" << "error_dist: " << error_dist << "Target: " << _target_pose.x << "Current: " << curr_cart_x); 
                    double linear_velocity = _linear_pid.calculate(error_dist, time_diff);
                    angVelLeft += linear_velocity / wheel_radius;
                    angVelRight += linear_velocity / wheel_radius;
                }else{
                    RCLCPP_INFO_STREAM(this->get_logger(), "DIST THRESHOLD PASSED\n");
                    RCLCPP_INFO_STREAM(this->get_logger(), "curr_cart_x: " << curr_cart_x << " curr_cart_y: " << curr_cart_y);
                    RCLCPP_INFO_STREAM(this->get_logger(), "target_x: " << _target_pose.x << " target_y: " << _target_pose.y);
                    if(_next_point_received == false){ //rename to next point requested
                        _next_point_received = true;
                        set_next_target();
                    }
                }
            }
            
            //integrator (not used as of now)
            _ang_vel_l += angVelLeft * time_diff * 10;
            _ang_vel_r += angVelRight * time_diff * 10;

            pub_timer_callback(angVelLeft, angVelRight);
            // pub_timer_callback(_ang_vel_l, _ang_vel_r);

            if (_csv_writer_buffer >=  this -> get_parameter("csv_buffer_skip_length").as_int()){
                _csv_writer_buffer = 0;
                _csv_writer << _curr_time << "," << curr_cart_x << "," << curr_cart_y << "," << curr_euler_angle << "," << _target_pose.x << "," << _target_pose.y << "," << angVelLeft << "," << angVelRight << "\n";
            } else _csv_writer_buffer++;
        }

        void pub_timer_callback(double angVelLeft = 0 , double angVelRight = 0)
        { //TODO change var names, give lin and ang vel
            auto message = geometry_msgs::msg::Twist();
            tuple <double, double> motor_speeds = calculate_motor_speeds(angVelLeft, angVelRight);
            message.linear.x = get<0>(motor_speeds);
            message.angular.z = get<1>(motor_speeds);

            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing - left_motor_speed:'" << angVelLeft
                << "'" << " right_motor_speed:'" << angVelRight << "'");
            _publisher->publish(message);
        }

        std::tuple<double, double> calculate_motor_speeds(double angVelRight, double angVelLeft) //Angular velocity of both wheels
        {
            double wheel_radius = this->get_parameter("wheel_radius").as_double();
            double wheel_distance = this->get_parameter("wheel_distance").as_double();
            double leftLinVel = wheel_radius * angVelLeft;
            double rightLinVel = wheel_radius * angVelRight;
            double linVel = (leftLinVel + rightLinVel) / 2;
            double angVel = (rightLinVel - leftLinVel) / wheel_distance;
            return std::make_tuple(linVel, angVel);
        }    

        void set_next_target()
        {
            auto request = std::make_shared<robot_msgs::srv::GetTarget::Request>();
            request -> waypoint = _current_waypoint;
            while(!_client -> wait_for_service(1s)){
                    if(!rclcpp::ok()){
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto response_future = _client -> async_send_request(request, std::bind(&RobotController::get_target_callback, this, _1));
        }

        void get_target_callback(rclcpp::Client<robot_msgs::srv::GetTarget>::SharedFuture future){
            auto status = future.wait_for(1s);
            if(status == std::future_status::ready){
                auto response = future.get();
                RCLCPP_INFO_STREAM(this->get_logger(), "Response next waypoint: " << response -> waypoint_next);
                _target_pose = response -> position;
                _current_waypoint = response -> waypoint_next;
            } else if (status == std::future_status::timeout){
                RCLCPP_ERROR(this->get_logger(), "Service call timed out :(");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed >:(");
            }
            _next_point_received = false;
        }
        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}