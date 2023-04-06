#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/transport.hh>
#include <ros/ros.h>
#include <geometry_msgs/msg/point.hpp>

class GazeboRosPositionPlugin : public gazebo::ModelPlugin {
public:
    GazeboRosPositionPlugin();
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
    void OnPoseMsg(ConstPosePtr &_msg);
    ros::Publisher publisher_;
    
};
