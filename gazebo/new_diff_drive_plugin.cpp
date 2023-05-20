#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
// #include <ignition/plugin/Register.hh>
#include<string>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/common/Console.hh>

#include "new_diff_drive_plugin.hpp"

#include <ignition/common/Profiler.hh>
#include <ignition/math/DiffDriveOdometry.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
IGNITION_ADD_PLUGIN(
    new_diff_drive_plugin::NewDiffDrivePlugin, //classname
    ignition::gazebo::System, //gz system
    new_diff_drive_plugin::NewDiffDrivePlugin::ISystemConfigure, //inheritance of interface
    new_diff_drive_plugin::NewDiffDrivePlugin::ISystemUpdate //inheritance of interface
)
using namespace ignition::common;
using namespace new_diff_drive_plugin;

void NewDiffDrivePlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
    auto leftJointName = _sdf->Get<std::string>("left_joint");
    auto rightJointName = _sdf->Get<std::string>("right_joint");
    auto linearAcceleration = _sdf->Get<double>("linear_acceleration");
    auto angularAcceleration = _sdf->Get<double>("angular_acceleration");

    auto model = ignition::gazebo::v6::Model(_entity);
    auto leftJoint = model.JointByName(_ecm, leftJointName);
    auto rightJoint = model.JointByName(_ecm, rightJointName);
    
    if (!leftJoint)
    {
        ignerr << "Unable to find left joint [" << leftJointName
            << "] for entity [" << _entity << "]" << std::endl;
        return;
    }

    if (!rightJoint)
    {
        ignerr << "Unable to find right joint [" << rightJointName
            << "] for entity [" << _entity << "]" << std::endl;
        return;
    }
}

void NewDiffDrivePlugin::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    std::string msg = "Test please work D: Simulation is currently ";
    if (!_info.paused) msg += "not ";
    msg += "paused.";

}
        
