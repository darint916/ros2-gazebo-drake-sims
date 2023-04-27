#include <chrono>
#include <memory>
#include <string>
#include <functional>
// #include <drake/math>
#include <cmath>

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
            _target_pose = geometry_msgs::msg::Pose2D();
 
            _linear_pid = PIDController(0.1, 0.0, 0.0, 2.0, 1.0);
            _angular_pid = PIDController(0.1, 0.0, 0.0, 2.0, 1.0);
        }

    private:

        void topic_robot_input_callback(const nav_msgs::msg::Odometry & msg)
        {
            // RCLCPP_INFO_STREAM(this->get_logger(), "Inputted - speed:'" << msg.pose. << "'" << " angle:'" << msg.twist << "'");
            double curr_cart_x = msg.pose.pose.position.x;
            double curr_cart_y = msg.pose.pose.position.y;
            //ignore z for as car can be seen as 2d
            double curr_quat_w = msg.pose.pose.orientation.w;
            double curr_quat_z = msg.pose.pose.orientation.z;
            double curr_euler_angle = 2 * atan2(curr_quat_z, curr_quat_w);

            double dx = _target_pose.x - curr_cart_x;
            double dy = _target_pose.y - curr_cart_y;

            double error_dist = sqrt(dx*dx + dy*dy);
            double error_angle = atan2(dy, dx) - curr_euler_angle;
            while (error_angle > M_PI) error_angle -= 2 * M_PI;
            while (error_angle < -M_PI) error_angle += 2 * M_PI;

            if(error_angle < 0.1 && error_dist < 0.1) set_next_target();

            double linear_velocity = _linear_pid.calculate(error_dist);
            double angular_velocity = _angular_pid.calculate(error_angle);

            double wheel_radius = this -> get_parameter("wheel_radius").as_double();
            double wheel_distance = this -> get_parameter("wheel_distance").as_double();
            // wheel_radius = 1;
            // wheel_distance = 1;

            //Yeah I know that we kind of reverse this when publishing, but think, if this was a real robot, we would be calculating 
            //The wheel angular velocity, which converts torque then motor speeds, so this part is flexible towards hardware
            //Also proof of understanding diff drive kinematics 
            double angVelLeft = (linear_velocity - angular_velocity * wheel_distance / 2) / wheel_radius;
            double angVelRight = (linear_velocity + angular_velocity * wheel_distance / 2) / wheel_radius;

            pub_timer_callback(angVelLeft, angVelRight);

        }

        void pub_timer_callback(double angVelLeft = 0 , double angVelRight = 0)
        {
            auto message = geometry_msgs::msg::Twist();
            // tuple <double, double> motor_speeds = calculate_motor_speeds(this -> get_parameter("speed").as_double(), this -> get_parameter("angle").as_double());
            tuple <double, double> motor_speeds = calculate_motor_speeds(angVelLeft, angVelRight);
            // auto [left_motor_speed, right_motor_speed] = calculate_motor_speeds(this -> get_parameter("speed").as_double(), this -> get_parameter("angle").as_double());
            // message.linear = get<0>(motor_speeds);
            // message.angular = get<1>(motor_speeds);
            cout << "left_motor_speed: " << get<0>(motor_speeds) << endl;
            message.linear.x = get<0>(motor_speeds);
            message.angular.z = get<1>(motor_speeds);

            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing - left_motor_speed:'" << message.linear.x 
                << "'" << " right_motor_speed:'" << message.angular.z << "'");
            _publisher->publish(message);
        }

        void set_next_target()
        {
            while(!_client -> wait_for_service(1s)){
                    if(!rclcpp::ok()){
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }
                auto request = std::make_shared<robot_msgs::srv::GetTarget::Request>();
                request -> waypoint = _current_waypoint;
                auto result = _client -> async_send_request(request);
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed >:(");
                    return;
                }
                auto response = result.get();
                RCLCPP_INFO_STREAM(this->get_logger(), "Response next waypoint: " << response -> waypoint_next);
                _target_pose = response -> position;
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