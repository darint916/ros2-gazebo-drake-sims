#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/pose.pb.h>

namespace gazebo
{
  class ModelPosePublisher : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Initialize Ignition Gazebo transport node
      transport_node_ = std::make_unique<ignition::transport::Node>();

      // Subscribe to the model's pose
      pose_subscriber_ = transport_node_->Subscribe(
        "/model/" + _parent->GetName() + "/pose/info", 
        &ModelPosePublisher::onPoseMsg, this);

      // Initialize ROS node
      int argc = 0;
      char **argv = NULL;
      rclcpp::init(argc, argv);

      // Create ROS publisher for the model's pose
      publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);

      // Create timer to update ROS publisher at a fixed rate
      auto timer_callback = std::bind(&ModelPosePublisher::onTimer, this);
      timer_ = node_->create_wall_timer(ros::Duration(1.0 / rate_), timer_callback);
    }

  private:
    void onPoseMsg(const ignition::msgs::Pose& _msg)
    {
      // Convert Ignition message to ROS message
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = rclcpp::Clock().now();
      pose_msg.pose.position.x = _msg.position().x();
      pose_msg.pose.position.y = _msg.position().y();
      pose_msg.pose.position.z = _msg.position().z();
      pose_msg.pose.orientation.x = _msg.orientation().x();
      pose_msg.pose.orientation.y = _msg.orientation().y();
      pose_msg.pose.orientation.z = _msg.orientation().z();
      pose_msg.pose.orientation.w = _msg.orientation().w();

      // Update ROS publisher
      pose_publisher_->publish(pose_msg);
    }

    void onTimer()
    {
      // Get current model pose and publish to Ignition transport
      ignition::math::Pose3d pose = this->model_->WorldPose();
      ignition::msgs::Pose
