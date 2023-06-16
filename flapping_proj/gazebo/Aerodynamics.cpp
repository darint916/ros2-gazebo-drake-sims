#include <mutex>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Quaternion.hh>
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/Pose.hh" 
#include "ignition/gazebo/components/Name.hh" 
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "Aerodynamics.hpp"

/*
    TODO Considerations:
    Decide whether to use one plugin per link or one plugin for all
    One plugin per link = separate data publication, (could work), need to catch separate topics for data retreival
    One plugin for all = one data publication, data formatting different, prob Pose_V
    Need to implement type matching for aerodynamics equation
    Have another struct type for holding params/info for each link
    Have default equation multiplied by modifiers like cos, default 1
    Fix header vars, no need for mutex? 
    
    TODO next:
    Test SDF get element for child query http://osrf-distributions.s3.amazonaws.com/sdformat/api/3.0/classsdf_1_1Element.html#a5af1be8776ec7f70edc85204e1f76de7
    Test diff Drive coordinate publication info, what it should publish like, worldPose vs Pose for components.
    Fix Parameter input
    Calculate wrench values from aerodynamics equation
    Publish wrench values
*/


using namespace aerodynamics;
namespace igz = ignition::gazebo;
Aerodynamics::Aerodynamics() {
    this->dataPtr = std::make_unique<AerodynamicsData>();
}

void Aerodynamics::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager &)
{
    this->dataPtr->model = Model(_entity);
    if(!this->dataPtr->model.Valid(_ecm)) {
        ignerr << "plugin should be attached to a model entity, fail init" << std::endl;
        return;
    }
    // _ecm.Component<ignition::gazebo::components::Pose>(
    this->dataPtr->sdfConfig = _sdf->Clone();
}

void Aerodynamics::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{ //could slow down as preupdate always called at each timestep?
    IGN_PROFILE("Aerodynamics::PreUpdate");
    if (!this->dataPtr->initialized){
        this->dataPtr->initialized = true;
       
        this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
        if (this->dataPtr->validConfig){
            igz::Link link(this->dataPtr->linkEntity);
            link.EnableVelocityChecks(_ecm, true);
        }
    }

    if (_info.paused) return;

    if (this->dataPtr->initialized && this->dataPtr->validConfig){
        this->dataPtr->Update(_ecm);
    }
}

void AerodynamicsData::Load(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf)
{
    //TODO: Finish Data query from SDF
    std::string linkName = _sdf->Get<std::string>("link_name", "");
    if (linkName.empty())
    {
        ignerr << "Aerodynamics plugin should have a <link_name> child." << std::endl;
        this->validConfig = false;
        return;
    }

    this->linkEntity = this->model.LinkByName(_ecm, linkName);
    this->linkPose = this->model.Pose(_ecm, this->linkEntity);
}

void AerodynamicsData::Update(EntityComponentManager &_ecm)
{
    //Todo calculate aerodynamic equations, query link pose, apply wrench, publish wrench
}
IGNITION_ADD_PLUGIN(Aerodynamics,
                    System,
                    Aerodynamics::ISystemConfigure,
                    Aerodynamics::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LiftDrag, "gz::sim::systems::LiftDrag")
