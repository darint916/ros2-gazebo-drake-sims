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
//error red squiggles from c++ extension
using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotController : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<robot_msgs::msg::MotorSpeeds>::SharedPtr _publisher;
        rclcpp::Subscription<robot_msgs::msg::DifferentialIn>::SharedPtr _subscription;
        // double _left_motor_speed;
        // double _right_motor_speed;

        void topic_robot_input_callback(const robot_msgs::msg::DifferentialIn & msg)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Inputted - speed:'" << msg.speed << "'" << " angle:'" << msg.angle << "'");
            this -> set_parameter(rclcpp::Parameter("speed", msg.speed));
            this -> set_parameter(rclcpp::Parameter("angle", msg.angle));
        }
        
        void pub_timer_callback()
        {
            auto message = robot_msgs::msg::MotorSpeeds();
            auto [left_motor_speed, right_motor_speed] = calculate_motor_speeds(this -> get_parameter("speed").as_double(), this -> get_parameter("angle").as_double());
            message.left_wheel_speed = left_motor_speed;
            message.right_wheel_speed = right_motor_speed;
            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing - left_motor_speed:'" << message.left_wheel_speed 
                << "'" << " right_motor_speed:'" << message.right_wheel_speed << "'");
            _publisher->publish(message);
        }

        std::tuple<double, double> calculate_motor_speeds(double speed, double angle)
        {
            double wheel_radius = this->get_parameter("wheel_radius").as_double();
            double wheel_distance = this->get_parameter("wheel_distance").as_double();
            double wl = (2 * speed - wheel_distance * angle) / (2 * wheel_radius);
            double wr = (2 * speed + wheel_distance * angle) / (2 * wheel_radius);
            double left_motor_speed = wheel_radius / 2 * ( wl + wr) * cos(angle);
            double right_motor_speed = wheel_radius / 2 * ( wl + wr) * sin(angle);
            return std::make_tuple(left_motor_speed, right_motor_speed);
        }

    //TODO: add positional parameters for intake from gazebo and then do trajectory + correct
    public:
        RobotController() : Node("robot_controller")
        {
            //Declares parameters
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "The distance between the wheels of the robot, and override for speed and angle";
            this -> declare_parameter<double>("wheel_radius", 0.0);
            this -> declare_parameter<double>("wheel_distance", 0.0);
            this -> declare_parameter<double>("speed", 0.0);
            this -> declare_parameter<double>("angle", 0.0);

            //Publishes to "motor_speeds" topic with a queue size of 10
            _publisher = this->create_publisher<robot_msgs::msg::MotorSpeeds>("motor_speeds", 10);

            //Creates a timer that publishes to the "motor_speeds" topic every 500ms
            _timer = this->create_wall_timer(500ms, std::bind(&RobotController::pub_timer_callback, this));

            //Subscribes to "diff_drive" topic with a queue size of 10
            _subscription = this->create_subscription<robot_msgs::msg::DifferentialIn>("angle_speed_in", 10, std::bind(&RobotController::topic_robot_input_callback, this, _1));
           
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}