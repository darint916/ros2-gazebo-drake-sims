#include <mutex>

#include <ignition/plugin/RegisterMore.hh> //Register the plugin
#include <ignition/transport/Node.hh> //For nodes, subs, stuff, found in workspace, gz-transport Node.cc
//systems imported in header

#include "DiffDrive.hpp"

//ignition::gazebo used a lot

 
#include <ignition/msgs/odometry.pb.h>

#include <mutex>

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

#include "DiffDrive.hh"
#include "SpeedLimiter.hh"



DiffDrive::DiffDrive(){
    dataPtr = std::make_unique<DiffDriveData>();
}

void DiffDrive::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
{
    this->dataPtr->model = ignition::gazebo::Model(_entity);

    // Get the canonical link
    // std::vector<Entity> links = this->dataPtr->model.Links(_ecm);
    std::vector<Entity> links = _ecm.ChildrenByComponents(this->dataPtr->model.Entity(), components::CanonicalLink());

    if(!links.empty()) this->dataPtr->canonicalLink = links[0];
    if(!this->dataPtr->canonicalLink.Valid(_ecm)) {
        ignerr << "DiffDrive plugin should be attached to a model entity, fail init" << std::endl;
        return;
    }

    //test without const_cast
    auto sdf_ptr = const_cast<sdf::Element*>(_sdf.get()); //cast to non-const sdf element to use method
    
    sdf::ElementPtr leftJointElements = sdf_ptr->GetElement("left_joint");
    while(leftJointElements){
        this->dataPtr->leftJointNames.push_back(leftJointElements->Get<std::string>());
        leftJointElements = leftJointElements->GetNextElement("left_joint");
    }
    sdf::ElementPtr rightJointElements = sdf_ptr->GetElement("right_joint");
    while(rightJointElements){
        this->dataPtr->rightJointNames.push_back(rightJointElements->Get<std::string>());
        rightJointElements = rightJointElements->GetNextElement("right_joint");
    }

    //second arg sets if first arg is null/not found
    this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius", this->dataPtr->wheelRadius).first;
    this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation", this->dataPtr->wheelSeparation).first;
    
    double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50.0).first;
    if(odomFreq > 0.0){
        std::chrono::duration<double> odomPeriod = 1 / odomFreq;
        this->data->odomPubPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPeriod);
    }

    //odom setup
    this->dataPtr->
}

void DiffDrive::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    auto modelName = this->dataPtr->model.Name(_ecm);
    if(this->dataPtr->leftJoints.empty() || this->dataPtr-rightJoints.empty()){
        bool warned = false;
        for(const std::string &jointName : this->dataPtr->leftJointNames){
            ignition::gazebo::Entity joint = this
    }
    if(_info.paused) return;
    
    //start setting values
    ignition::gazebo::Entity leftJoint = this->dataPtr->leftJoints

}