#include <chrono>
#include <memory>
#include <string>
#include <functional>
// #include <drake/math>
#include <cmath>
#include <coroutine>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/differential_in.hpp"
#include "rclcpp/parameter.hpp"
#include "robot_msgs/msg/motor_speeds.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/srv/get_target.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"


#include "robot_controller/pid_controller.hpp"
//error red squiggles from c++ extension
using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotController : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscription;
        rclcpp::Client<robot_msgs::srv::GetTarget>::SharedPtr _client;
        int _current_waypoint;
        geometry_msgs::msg::Pose2D _target_pose;

        PIDController _linear_pid;
        PIDController _angular_pid;

        bool _next_point_received;
        // PIDController _linear_pid(0.1, 0.0, 0.0, 2.0, 1.0);
        // PIDController _angular_pid(0.1, 0.0, 0.0, 2.0, 1.0); //no idea what max should be for both...
        // double _left_motor_speed;
        // double _right_motor_speed;

    public:
        RobotController() : Node("robot_controller")
        {
            //Declares parameters
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "The distance between the wheels of the robot, and override for speed and angle";
            this -> declare_parameter<double>("wheel_radius", 0.0);
            this -> declare_parameter<double>("wheel_distance", 0.0);
            this -> declare_parameter<double>("pos", 0.0);
            this -> declare_parameter<double>("angle", 0.0);
            // this -> 
            //Publishes to "motor_speeds" topic with a queue size of 10
            _publisher = this->create_publisher<geometry_msgs::msg::Twist>("motor_speeds", 10);

            //Creates a timer that publishes to the "motor_speeds" topic every 500ms
            // _timer = this->create_wall_timer(500ms, std::bind(&RobotController::pub_timer_callback, this));

            //Subscribes to "diff_drive" topic with a queue size of 10
            _subscription = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotController::topic_robot_input_callback, this, _1));
            
            _client = this->create_client<robot_msgs::srv::GetTarget>("get_target");

            _current_waypoint = 0;
            _target_pose.x = -1;
            _target_pose.y = -1;
            _target_pose.theta = 0;
 
            _linear_pid = PIDController(2, 0.1, 0.1, 1.0, 1.0);
            _angular_pid = PIDController(2, 0.2, 0.4, 2, 1.0);

            _next_point_received = false;
        }

    private:

        void topic_robot_input_callback(const nav_msgs::msg::Odometry & msg)
        {
            double wheel_radius = this -> get_parameter("wheel_radius").as_double();
            double wheel_distance = this -> get_parameter("wheel_distance").as_double();
            // RCLCPP_INFO_STREAM(this->get_logger(), "Inputted - speed:'" << msg.pose. << "'" << " angle:'" << msg.twist << "'");
            double curr_cart_x = msg.pose.pose.position.x;
            double curr_cart_y = msg.pose.pose.position.y;
            //ignore z for as car can be seen as 2d
            double curr_quat_w = msg.pose.pose.orientation.w;
            double curr_quat_z = msg.pose.pose.orientation.z;
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
            if(abs(error_angle) > .05 && error_dist > .15) {
                RCLCPP_INFO_STREAM(this->get_logger(), "ANGLE ERRORS\n" << "error_angle: " << error_angle << " Target: " << angle_to_target << " Current: " << curr_euler_angle); 
                RCLCPP_INFO_STREAM(this->get_logger(), "DIST ERRORS\n" << "error_dist: " << error_dist << "Target: " << _target_pose.x << "Current: " << curr_cart_x); 
                double angular_velocity = _angular_pid.calculate(error_angle);
                angVelLeft = (angular_velocity * wheel_distance / 2) / wheel_radius;
                // angVelLeft = -angVelRight;
                angVelRight = -angVelLeft;
            }else {
                RCLCPP_INFO_STREAM(this->get_logger(), "ANGLE THRESHOLD PASSED\n" << "error_angle: " << error_angle << " Target: " << angle_to_target << " Current: " << curr_euler_angle);
                if(error_dist > .05) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "DIST ERRORS\n" << "error_dist: " << error_dist << "Target: " << _target_pose.x << "Current: " << curr_cart_x); 
                    double linear_velocity = _linear_pid.calculate(error_dist);
                    angVelLeft = linear_velocity / wheel_radius;
                    angVelRight = angVelLeft;
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
            // double angular_velocity = 0;
            // double linear_velocity = 0;
            // double angVelRight;
            // double angVelLeft;
            // RCLCPP_INFO_STREAM(this->get_logger(), "curr_cart_x: " << curr_cart_x << " curr_cart_y: " << curr_cart_y << " curr_euler_angle: " << curr_euler_angle);
            // RCLCPP_INFO_STREAM(this->get_logger(), "error_dist: " << error_dist << " error_angle: " << error_angle);

            // if(error_angle > .1){
            //     angular_velocity = _angular_pid.calculate(error_angle);
            //     angVelRight = (angular_velocity * wheel_distance / 2) / wheel_radius;
            //     angVelLeft = -(angular_velocity * wheel_distance / 2) / wheel_radius;
            // } else if (error_dist > .1){
            //     linear_velocity = _linear_pid.calculate(error_dist);
            //     angVelLeft = linear_velocity / wheel_radius;
            //     angVelRight = angVelLeft;
            // } else {
            //     angVelLeft = 0;
            //     angVelRight = 0;
            //     // set_next_target();
            // }
            // if(error_angle < 0.1 && error_dist < 0.1) set_next_target();
            // double linear_velocity = _linear_pid.calculate(error_dist);
            

            // wheel_radius = 1;
            // wheel_distance = 1;

            //temp solution to turn pid to reach a point
            

            //Yeah I know that we kind of reverse this when publishing, but think, if this was a real robot, we would be calculating 
            //The wheel angular velocity, which converts torque then motor speeds, so this part is flexible towards hardware
            //Also proof of understanding diff drive kinematics 
            // double angVelLeft = (linear_velocity - angular_velocity * wheel_distance / 2) / wheel_radius;
            // double angVelRight = (linear_velocity + angular_velocity * wheel_distance / 2) / wheel_radius;
            // RCLCPP_INFO_STREAM(this->get_logger(), "lin vel" << linear_velocity  << "'" << " ang vel'" << angular_velocity << "'" << " waypoint: " << _current_waypoint << " wheel_radius: " << wheel_radius << " wheel_distance: " << wheel_distance << " target x: " << _target_pose.x << " target y: " << _target_pose.y);
            
            // << " error_dist: " << error_dist << " error_angle: " << error_angle << " angVelLeft: " << angVelLeft << " angVelRight: " << angVelRight
            
            pub_timer_callback(angVelLeft, angVelRight);
        }

        void pub_timer_callback(double angVelLeft = 0 , double angVelRight = 0)
        { //TODO change var names, give lin and ang vel
            auto message = geometry_msgs::msg::Twist();
            // tuple <double, double> motor_speeds = calculate_motor_speeds(this -> get_parameter("speed").as_double(), this -> get_parameter("angle").as_double());
            tuple <double, double> motor_speeds = calculate_motor_speeds(angVelLeft, angVelRight);
            // auto [left_motor_speed, right_motor_speed] = calculate_motor_speeds(this -> get_parameter("speed").as_double(), this -> get_parameter("angle").as_double());
            // message.linear = get<0>(motor_speeds);
            // message.angular = get<1>(motor_speeds);
            // cout << "left_motor_speed: " << get<0>(motor_speeds) << endl;
            message.linear.x = get<0>(motor_speeds);
            message.angular.z = get<1>(motor_speeds);

            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing - left_motor_speed:'" << angVelLeft
                << "'" << " right_motor_speed:'" << angVelRight << "'");
            _publisher->publish(message);
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
            // auto response_future = _client -> async_send_request(request);
            // auto response = co_await response_future;
            // RCLCPP_INFO_STREAM(this->get_logger(), "Response next waypoint: " << response -> waypoint_next);

            //wait until result future is done
            // while(rclcpp::)
            // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
            // {
            //     RCLCPP_ERROR(this->get_logger(), "Service call failed >:(");
            //     return;
            // }
            // auto response = result.get();
            // RCLCPP_INFO_STREAM(this->get_logger(), "Response next waypoint: " << response -> waypoint_next);
            // _target_pose = response -> position;
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
        std::tuple<double, double> calculate_motor_speeds(double angVelRight, double angVelLeft) //Angular velocity of both wheels
        {
            double wheel_radius = this->get_parameter("wheel_radius").as_double();
            double wheel_distance = this->get_parameter("wheel_distance").as_double();
            double leftLinVel = wheel_radius * angVelLeft;
            double rightLinVel = wheel_radius * angVelRight;
            double linVel = (leftLinVel + rightLinVel) / 2;
            double angVel = (rightLinVel - leftLinVel) / wheel_distance;
            return std::make_tuple(linVel, angVel);
            // return {0, 0};
        }    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}