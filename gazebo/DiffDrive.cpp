#include <mutex>
#include <ignition/plugin/Register.hh> //Register the plugin
#include <ignition/transport/Node.hh> //For nodes, subs, stuff, found in workspace, gz-transport Node.cc
//systems imported in header 
#include <ignition/msgs/odometry.pb.h>
#include <ignition/common/Profiler.hh>
#include <ignition/math/DiffDriveOdometry.hh>
#include <ignition/math/Quaternion.hh>
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/Pose.hh"   
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "DiffDrive.hpp"

/*
ARGUMENT TAGS FOR PLUGIN
left_joint
right_joint
wheel_radius
wheel_separation
odom_publish_frequency
control_input_topic
odometry_topic
left_link_inertia
right_link_inertia
*/




using namespace diff_drive;
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
    std::vector<ignition::gazebo::Entity> links = _ecm.ChildrenByComponents(this->dataPtr->model.Entity(), ignition::gazebo::components::CanonicalLink());
    if(!links.empty()) this->dataPtr->canonicalLink = links[0];
    if(!this->dataPtr->model.Valid(_ecm)) {
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
    
    this->dataPtr->odomData.wheelSeparation = this->dataPtr->wheelSeparation;
    this->dataPtr->odomData.wheelRadius = this->dataPtr->wheelRadius;
    
    double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50.0).first;
    if(odomFreq > 0.0){
        std::chrono::duration<double> odomPeriod(1 / odomFreq);
        this->dataPtr->odomPubPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPeriod);
    }

    //transport node setup
    std::string inputTopic = _sdf->Get<std::string>("control_input_topic", "/diff_drive/control_input").first; 
    this->dataPtr->node.Subscribe(inputTopic, &DiffDriveData::InputCmdCallback, this->dataPtr.get()); //similar to ros2, arg order, is topic, callbackfunc, then arguments to callback func
    
    std::string odomTopic = _sdf->Get<std::string>("odometry_topic", "/diff_drive/odometry").first;
    this->dataPtr->odomPublisher = this->dataPtr->node.Advertise<ignition::msgs::Odometry>(odomTopic);
    
    ignmsg << "DiffDrive subscribing to twist msgs on [" << inputTopic << "] and publishing odometry on [" << odomTopic << "]" << std::endl;

    //TODO (in header)
    this->dataPtr->leftLinkInertia = _sdf->Get<double>("left_link_inertia", this->dataPtr->leftLinkInertia).first;
    this->dataPtr->rightLinkInertia = _sdf->Get<double>("right_link_inertia", this->dataPtr->rightLinkInertia).first;
}

void DiffDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm)
{
    auto modelName = this->dataPtr->model.Name(_ecm);

    static std::set<std::string> warnedModels;

    if(this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty()){
        bool warned = false;
        for(const std::string &jointName : this->dataPtr->leftJointNames){
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, jointName);
            if(joint != ignition::gazebo::kNullEntity){
                this->dataPtr->leftJoints.push_back(joint);
            } else if(warnedModels.find(modelName) == warnedModels.end()){ //prevents dupe errors
                ignwarn << "Unable to find left joint [" << jointName << "] for model [" << modelName << "]" << std::endl;
                warned = true;
            }
        }
        for(const std::string &jointName : this->dataPtr->rightJointNames){
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, jointName);
            if(joint != ignition::gazebo::kNullEntity){
                this->dataPtr->rightJoints.push_back(joint);
            } else if(warnedModels.find(modelName) == warnedModels.end()){ //prevents dupe errors
                ignwarn << "Unable to find right joint [" << jointName << "] for model [" << modelName << "]" << std::endl;
                warned = true;
            }
        }
        if(warned) warnedModels.insert(modelName);
    }
    if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty()) return;
    if(warnedModels.find(modelName) != warnedModels.end()){
        ignmsg << "Recovered from missing joint(s) for model [" << modelName << "]" << std::endl;
        warnedModels.erase(modelName);
    }
    if(_info.paused) return;
    
    for (ignition::gazebo::Entity leftJoint : this->dataPtr->leftJoints){
        auto leftJointTorquePtr = _ecm.Component<ignition::gazebo::components::	JointForceCmd>(leftJoint);
        if(leftJointTorquePtr == nullptr){
            _ecm.CreateComponent(leftJoint, ignition::gazebo::components::JointForceCmd({this->dataPtr->leftJointTorque}));
        } else {
            *leftJointTorquePtr = ignition::gazebo::components::JointForceCmd({this->dataPtr->leftJointTorque});
        }
    }
    for (ignition::gazebo::Entity rightJoint : this->dataPtr->rightJoints){
        auto rightJointTorquePtr = _ecm.Component<ignition::gazebo::components::JointForceCmd>(rightJoint);
        if(rightJointTorquePtr == nullptr){
            _ecm.CreateComponent(rightJoint, ignition::gazebo::components::JointForceCmd({this->dataPtr->rightJointTorque}));
        } else {
            *rightJointTorquePtr = ignition::gazebo::components::JointForceCmd({this->dataPtr->rightJointTorque});
        }
    }

    //if pose dont exist on the first joints for odom, create it
    auto leftJointPositionPtr = _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->leftJoints[0]);
    if(leftJointPositionPtr == nullptr){
        _ecm.CreateComponent(this->dataPtr->leftJoints[0], ignition::gazebo::components::JointPosition());
    }
    auto rightJointPositionPtr = _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->rightJoints[0]);
    if(rightJointPositionPtr == nullptr){
        _ecm.CreateComponent(this->dataPtr->rightJoints[0], ignition::gazebo::components::JointPosition());
    }
}

// void DiffDrive::PostUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm)
void DiffDrive::PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm)
{
    if(_info.paused) return;
    //update acceleration
    double linearAcceleration;
    double angularAcceleration;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
        linearAcceleration = this->dataPtr->targetAcceleration.linear().x();
        angularAcceleration = this->dataPtr->targetAcceleration.angular().z();
    }

    const double deltaTime = std::chrono::duration<double>(_info.dt).count();

    //history update
    this->dataPtr->prevprevCmd = this->dataPtr->prevCmd;
    this->dataPtr->prevCmd.linearAcceleration = linearAcceleration;
    this->dataPtr->prevCmd.angularAcceleration = angularAcceleration;
    
    this->dataPtr->odomData.linAcc = linearAcceleration;
    this->dataPtr->odomData.angAcc = angularAcceleration;
    //works if acc for joint in phys engine takes in movement, doubt tho
    double leftJointAcceleration = (linearAcceleration + angularAcceleration * this->dataPtr->wheelSeparation / 2) / this->dataPtr->wheelRadius;
    double rightJointAcceleration = (linearAcceleration - angularAcceleration * this->dataPtr->wheelSeparation / 2) / this->dataPtr->wheelRadius;
    
    //acc to force (not rly torque? need to test and rename if needed) div by rad for torque -> force
    this->dataPtr->leftJointTorque = this->dataPtr->leftLinkInertia * leftJointAcceleration / this->dataPtr->wheelRadius;
    this->dataPtr->rightJointTorque = this->dataPtr->rightLinkInertia * leftJointAcceleration / this->dataPtr->wheelRadius;

    //update odom
    if(!this->dataPtr->odomData.initialized) {
        this->dataPtr->odomData.initialized = true;
        this->dataPtr->odomData.time = _info.simTime;
        return;
    } 

    // if(this->leftJoints.empty() || this->rightJoints.empty()) return;

    // auto leftJointPositionPtr = _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->leftJoints[0]);
    // auto rightJointPositionPtr = _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->rightJoints[0]);

    // if(leftJointPositionPtr == nullptr || rightJointPositionPtr == nullptr) return;
    // this->dataPtr->odomData.

    // auto deltaTime =  _info.simTime - this->dataPtr->prevOdomPubTime;
    // if(deltaTime < this->dataPtr->odomPubPeriod && deltaTime > 0) return;
    // this->prevOdomPubTime = _info.simTime;

    ignition::msgs::Odometry odomMsg;
    auto canonicalLinkPositionPtr = _ecm.Component<ignition::gazebo::components::Pose>(this->dataPtr->canonicalLink);
    if(canonicalLinkPositionPtr == nullptr){
        ignerr << "Canonical link pose not found, fail odom update" << std::endl;
        return;
    }
    //canonicalLinkPositionPtr->Data() is ignition::math::Pose3d, which is Pose3<double> type, 
    odomMsg.mutable_pose()->mutable_position()->set_x(canonicalLinkPositionPtr->Data().X());
    odomMsg.mutable_pose()->mutable_position()->set_y(canonicalLinkPositionPtr->Data().Y());
    odomMsg.mutable_pose()->mutable_position()->set_z(canonicalLinkPositionPtr->Data().Z());
    odomMsg.mutable_pose()->mutable_orientation()->set_w(canonicalLinkPositionPtr->Data().Yaw());
    odomMsg.mutable_twist()->mutable_linear()->set_x(this->dataPtr->odomData.angVel);
    odomMsg.mutable_twist()->mutable_angular()->set_z(this->dataPtr->odomData.angAcc);
    odomMsg.mutable_header()->mutable_stamp()->CopyFrom(ignition::msgs::Convert(this->dataPtr->odomData.time));

    //could add more headers and frames if needed

    this->dataPtr->odomPublisher.Publish(odomMsg);
}   

void DiffDriveData::InputCmdCallback(const ignition::msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    this->targetAcceleration = _msg;
}


IGNITION_ADD_PLUGIN(
    DiffDrive,
    ignition::gazebo::System,
    DiffDrive::ISystemConfigure,
    DiffDrive::ISystemPreUpdate,
    DiffDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DiffDrive, "ignition::gazebo::DiffDrive")