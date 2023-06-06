#include <mutex>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Quaternion.hh>
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/Pose.hh" 
#include "ignition/gazebo/components/Name.hh" 
#include <ignition/msgs/pose_v.pb.h>
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "OdometryState.hpp"

using namespace odometry_state;
namespace igz = ignition::gazebo;
OdometryState::OdometryState() {
    this->dataPtr = std::make_unique<OdometryStateData>(); //keep `this`?
}


/*
SDF Parameter tags / usage, filename is from CMake proj name, name is from designated alias 

<plugin filename="odometry_state_plugin" name="odometry_state">
    <world_name>world1</world_name>
    <odometry_topic>/odometry</odometry_topic>
    <odom_publish_frequency>20.0</odom_publish_frequency>
    <joint>joint1</joint>
    <joint>joint2</joint> ... 
</plugin>
*/
void OdometryState::Configure(const igz::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, igz::EntityComponentManager &_ecm, igz::EventManager &/*_eventMgr*/)
{
    this->dataPtr->model = igz::Model(_entity);
    if(!this->dataPtr->model.Valid(_ecm)) {
        ignerr << "plugin should be attached to a model entity, fail init" << std::endl;
        return;
    }

    //Get canonical link
    std::vector<ignition::gazebo::Entity> links = _ecm.ChildrenByComponents(this->dataPtr->model.Entity(), ignition::gazebo::components::CanonicalLink());
    if(!links.empty()) this->dataPtr->canonicalLink = links[0];
    auto canonicalLinkNamePtr = _ecm.Component<igz::components::Name>(this->dataPtr->canonicalLink);
    if(canonicalLinkNamePtr == nullptr) {
        ignerr << "Canonical link name not found" << std::endl;
        return;
    }
    this->dataPtr->canonicalLinkName = canonicalLinkNamePtr->Data();

    //create joint map
    auto sdf_ptr = const_cast<sdf::Element*>(_sdf.get());
    sdf::ElementPtr jointElem = sdf_ptr->GetElement("joint");
    while(jointElem != nullptr){
        std::string jointName = jointElem->Get<std::string>();
        igz::Entity jointEntity = this->dataPtr->model.JointByName(_ecm, jointName);
        if (jointEntity == igz::kNullEntity) {
            ignerr << "Joint [" << jointName << "] not found" << std::endl;
        } else {
            this->dataPtr->jointMap[jointName] = jointEntity;
        }
        _ecm.CreateComponent(jointEntity, igz::components::JointPosition());
        jointElem = jointElem->GetNextElement("joint");
    }

    //position sub
    //future TODO: query for world name for dynamic pose
    std::string worldName = _sdf->Get<std::string>("world_name", "world1").first;
    std::string poseTopic = "/world/" + worldName + "/dynamic_pose/info";
    this->dataPtr->node.Subscribe(poseTopic, &OdometryStateData::PositionCallBack, this->dataPtr.get());
    ignwarn << "Subscribed to " << poseTopic << std::endl;
    //odom publish
    std::string odometryTopic = _sdf->Get<std::string>("odometry_topic", "/odometry").first;
    this->dataPtr->odomPublisher = this->dataPtr->node.Advertise<ignition::msgs::Pose_V>(odometryTopic);
    
    double odometryFreq = _sdf->Get<double>("odom_publish_frequency", 50.0).first;
    if(odometryFreq > 0.0){
        std::chrono::duration<double> odomPeriod(1 / odometryFreq);
        this->dataPtr->odomPubPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPeriod);
    }    

    
}

// void OdometryState::PreUpdate(const igz::UpdateInfo &_info, igz::EntityComponentManager &_ecm)
// {

// }
void OdometryState::PostUpdate(const igz::UpdateInfo &_info, const igz::EntityComponentManager &_ecm)
{
    if(_info.paused) return;
    
    //throttle publishing rate, redundant sanity check if sim paused
    auto deltaTime = _info.simTime - this->dataPtr->prevOdomPubTime;
    if (deltaTime > std::chrono::steady_clock::duration::zero() && deltaTime < this->dataPtr->odomPubPeriod) return;
    this->dataPtr->prevOdomPubTime = _info.simTime;

    //fetch from the auto pub
    ignition::msgs::Pose_V poseMsgs;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
        poseMsgs = this->dataPtr->poseMsg;
    }
    
    //gets main body pose, much easier than accessing each component prop imo
    ignition::msgs::Pose poseMsg;
    if(this->dataPtr->canonicalLinkIndex < 0){
        int poses = poseMsgs.pose_size();
        ignwarn << "poseMsgs size: " << poses << std::endl;
        ignwarn << "canonicalLinkName: " << this->dataPtr->canonicalLinkName << std::endl;
        for(int i = 0; i < poses; ++i){
            if(poseMsgs.pose(i).name() == this->dataPtr->canonicalLinkName){
                this->dataPtr->canonicalLinkIndex = i;
                break;
            }
        }
    }
    if(this->dataPtr->canonicalLinkIndex < 0){
        ignerr << "Canonical link pose not found" << std::endl;
        return;
    }
    ignition::msgs::Pose_V odomMsg;
    odomMsg.mutable_header()->CopyFrom(poseMsg.header());
    odomMsg.add_pose()->CopyFrom(poseMsgs.pose(this->dataPtr->canonicalLinkIndex));
    
    //iterate through joint map and add them as pose
    for(auto const& joint : this->dataPtr->jointMap){
        auto jointAnglePtr = _ecm.Component<igz::components::JointPosition>(joint.second);
        if(jointAnglePtr == nullptr){
            ignerr << "Joint angle not found for " << joint.first << std::endl;
            continue;
        }
        double jointAngle = jointAnglePtr->Data()[0]; //Radians and first idx for revolute joint
        ignition::msgs::Pose pose;
        pose.mutable_position()->set_x(jointAngle);
        pose.set_name(joint.first);
        odomMsg.add_pose()->CopyFrom(pose);
    }

    //Bridge somehow loses time header, so we make time pose as replacement
    ignition::msgs::Pose timePose;
    timePose.mutable_position()->set_x(std::chrono::duration<double>(_info.simTime).count()); //convert time to double
    timePose.set_name("time");
    odomMsg.add_pose()->CopyFrom(timePose);

    //publish
    this->dataPtr->odomPublisher.Publish(odomMsg);
}            

    
void OdometryStateData::PositionCallBack(const ignition::msgs::Pose_V &_msg)
{
    // ignerr << "PositionCallBack" << std::endl;
    std::lock_guard<std::mutex> lock(this->mutex);
    this->poseMsg = _msg;
}

IGNITION_ADD_PLUGIN(
    OdometryState,
    igz::System,
    OdometryState::ISystemConfigure,
    OdometryState::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OdometryState, "odometry_state")