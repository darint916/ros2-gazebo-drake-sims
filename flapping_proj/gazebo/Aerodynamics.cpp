#include <mutex>
#include <sstream>


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
    //TODO: Add checking for link + joint is actually a link + joint (not that important) check ref for other checks
    // MAKE AOA list?
    auto sdf_ptr = const_cast<sdf::Element*>(_sdf.get());
    sdf::ElementPtr linkElement = sdf_ptr->GetElement("link");
    while(linkElement != nullptr){
        //if below doesnt work, try
        //std::string linkName = linkElement->Get<std::string>("link_name", "").first;
        //std::string linkType = linkElement->Get<std::string>("link_type", "").first;
        std::string linkName = linkElement->GetElement("link_name")->Get<std::string>();
        std::string linkType = linkElement->GetElement("link_type")->Get<std::string>();
        if (linkName.empty() || linkType.empty())
        {
            ignerr << "Aerodynamics plugin should have a <link_name> and <link_type> child." << std::endl;
            this->validConfig = false;
            return;
        }
        if (linkType != "Generic" && linkType != "Wing")
        {
            ignerr << "Aerodynamics plugin should have a <link_type> of Generic or Wing." << std::endl;
            this->validConfig = false;
            return;
        }

        this->linkEntity = this->model.LinkByName(_ecm, linkName);
        this->linkMap[linkName].linkType = linkType;
        this->linkMap[linkName].stallAngle = linkElement->GetElement("stall_angle")->Get<double>();
        this->linkMap[linkName].angleOfAttack = linkElement->GetElement("angle_of_attack")->Get<double>();
        this->linkMap[linkName].fluidDensity = linkElement->GetElement("fluid_density")->Get<double>();
        this->linkMap[linkName].dragCoefficient = linkElement->GetElement("drag_coefficient")->Get<double>();
        this->linkMap[linkName].liftCoefficient = linkElement->GetElement("lift_coefficient")->Get<double>();
        if(linkType == "Wing") {
            this->linkMap[linkName].wingParameters.blades = linkElement->GetElement("int")->Get<int>();
            this->linkMap[linkName].wingParameters.wingSpan = linkElement->GetElement("wing_span")->Get<double>();
            std::string controlJointName = linkElement->GetElement("control_joint_name")->Get<std::string>();
            this->linkMap[linkName].wingParameters.controlJointName = controlJointName;
            igz::Entity jointEntity = this->model.JointByName(_ecm, controlJointName);
            this->linkMap[linkName].wingParameters.controlJointEntity = jointEntity;
            _ecm.CreateComponent(jointEntity, igz::components::JointVelocity()); //Init vel values for query
            //String to values for blade chord list
            std::stringstream ss(linkElement->GetElement("blade_chord_list")->Get<std::string>());
            std::string token;
            while(std::getline(ss, token, ',')){
                token.erase(std::remove(token.begin(), token.end(), ' '), token.end());
                try { //string to double parsing
                    this->linkMap[linkName].wingParameters.bladeChordList.push_back(std::stod(token));
                } catch (const std::invalid_argument& ia) {
                    ignerr << "blade_chord_list STOD Invalid argument: " << ia.what() << std::endl;
                    this->validConfig = false;
                    return;
                } catch (const std::out_of_range& oor) {
                    ignerr << "blade_chord_list STOD Out of Range error: " << oor.what() << std::endl;
                    this->validConfig = false;
                    return;
                }
            }
            if (this->linkMap[linkName].wingParameters.bladeChordList.size() != this->linkMap[linkName].wingParameters.blades){
                ignerr << "blade_chord_list size does not match blades" << std::endl;
                this->validConfig = false;
                return;
            }

            //String to values for center pressure list
            std::stringstream ss2(linkElement->GetElement("center_pressure_list")->Get<std::string>());
            std::string token2;
            while(std::getline(ss2, token2, ',')){
                token2.erase(std::remove(token2.begin(), token2.end(), ' '), token2.end());
                try { //string to double parsing
                    this->linkMap[linkName].centerPressureList.push_back(std::stod(token2));
                } catch (const std::invalid_argument& ia) {
                    ignerr << "center_pressure_list STOD Invalid argument: " << ia.what() << std::endl;
                    this->validConfig = false;
                    return;
                } catch (const std::out_of_range& oor) {
                    ignerr << "center_pressure_list STOD Out of Range error: " << oor.what() << std::endl;
                    this->validConfig = false;
                    return;
                }
            }
            if (this->linkMap[linkName].centerPressureList.size() != this->linkMap[linkName].wingParameters.blades){
                ignerr << "center_pressure_list size does not match blades" << std::endl;
                this->validConfig = false;
                return;
            }

        } else if (linkType == "Generic"){
            this->linkMap[linkName].centerPressure = linkElement->GetElement("center_pressure")->Get<ignition::math::Vector3d>();
        }
        linkElement = linkElement->GetNextElement("link");
    }
}

void AerodynamicsData::Update(EntityComponentManager &_ecm)
{
    //Todo calculate aerodynamic equations, query link pose, apply wrench, publish wrench
    //Change way velocity is retreived, different for each blade, currently using COM vel as approx.
    for (auto &link : this->linkMap){
        ignition::msgs::Pose linkPose = this->model.Pose(_ecm, link.second.linkEntity);
        
        //Link velocities in reference to world frame
        const auto worldLinVelPtr = _ecm.Component<components::WorldLinearVelocity>(link.second.linkEntity);
        const auto worldAngVelPtr = _ecm.Component<components::WorldAngularVelocity>(link.second.linkEntity);
        const auto worldPosePtr = _ecm.Component<components::WorldPose>(link.second.linkEntity);
        if (!worldLinVelPtr || !worldAngVelPtr || !worldPosePtr){
            ignerr << "World velocity or pose not found" << std::endl;
            return;
        }
        const double worldLinVel = worldLinVelPtr->Data();
        const double worldAngVel = worldAngVelPtr->Data();
        const ignition::msgs::Pose &pose = worldPose->Data();

        std::vector<ignition::math::Vector3d> centerPressureWorldList;  //cp relative to world frame
        for(auto &centerPressure : link.second.centerPressureList){
            centerPressureWorldList.push_back(pose.Rot().RotateVector(centerPressure)); //If rotate doesnt work, just add as transformation
        }

        //???????
        // components::JointPosition *controlJointPosition = nullptr;
        // if (this->controlJointEntity != kNullEntity)
        // {
        //     controlJointPosition =
        //         _ecm.Component<components::JointPosition>(this->controlJointEntity);
        // }

        if (link.second.linkType == "Wing"){
            /*
            Could add component testing with:
            if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId))
            */
            //Query joint velocity, pose
            const auto jointPosePtr = _ecm.Component<components::WorldPose>(link.second.wingParameters.controlJointEntity);
            const auto jointVelPtr = _ecm.Component<components::JointVelocity>(link.second.wingParameters.controlJointEntity);
            const ignition::msgs::Pose &jointPose = jointPosePtr->Data();
            const double jointVelocity = jointVelPtr->Data(); // rad/s

            for (int i = 0; i < link.second.wingParameters.blades; i++){
                // const ignition::math::Vector3d velocity = worldLinVel + worldAngVel.Cross(centerPressureWorldList[i]);
                // const auto normalizedVelocity = velocity.Normalize();

                //Linear Velocity of solely flapping motion at CP. w x r;  r = cp - jointPos
                ignition::math::Vector3d flapCenterPressureLinVel =  jointVelocity.Cross(centerPressureWorldList[i] - jointPose.Pos());
                if(flapCenterPressureLinVel.length() != .01){ //negligible small force
                    //A = cos(AOA) * chord * (totalSpan / #blades)
                    double flapWingAreaApplied = std::cos(link.second.angleOfAttack) * link.second.wingParameters.bladeChordList[i] * link.second.wingParameters.wingSpan / link.second.wingParameters.blades;
                    //F = .5 * p * Cd * v^2 * A
                    double flapDragScalar = .5 * link.second.fluidDensity  *  link.second.dragCoefficient * flapCenterPressureLinVel.SquaredLength() * flapWingAreaApplied;
                    //Orient to perpindidcular of wing: F = F_scalar * (-v / |v|) / cos(AOA)
                    ignition::math::Vector3d flapDragForce = flapDragScalar * (-flapCenterPressureLinVel.Normalize()) / cos(link.second.angleOfAttack);
                }
                /*
                Calculating regular drag from non flapping force (previous movement)
                */
                //Linear Velocity of non flapping motion at CP. v_l + w x r - v_f;  
                ignition::math::Vector3d nonFlapCenterPressureLinVel = worldLinVel + worldAngVel.Cross(centerPressureWorldList[i]) - flapCenterPressureLinVel;
                if(nonFlapCenterPressureLinVel.length() != .01){ //negligible small force
                    //A = cos(AOA) * chord * (totalSpan / #blades)
                    double nonFlapWingAreaApplied = std::cos(link.second.angleOfAttack) * link.second.wingParameters.bladeChordList[i] * link.second.wingParameters.wingSpan / link.second.wingParameters.blades;
                    //F = .5 * p * Cd * v^2 * A
                    double nonFlapDragScalar = .5 * link.second.fluidDensity  *  link.second.dragCoefficient * nonFlapCenterPressureLinVel.SquaredLength() * nonFlapWingAreaApplied;
                    //Orient to perpindidcular of wing: F = F_scalar * (-v / |v|) / cos(AOA)
                    ignition::math::Vector3d nonFlapDragForce = nonFlapDragScalar * (-nonFlapCenterPressureLinVel.Normalize()) / cos(link.second.angleOfAttack);
                }
            }
        } else if (link.second.linkType == "Generic"){
            const ignition::math::Vector3d velocity = worldLinVel + worldAngVel.Cross(centerPressureWorldList[0]);
            if(velocity.length() <= .01) return;
            const auto normalizedVelocity = velocity.Normalize();
            
        }
    }

}
IGNITION_ADD_PLUGIN(Aerodynamics,
                    System,
                    Aerodynamics::ISystemConfigure,
                    Aerodynamics::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LiftDrag, "gz::sim::systems::LiftDrag")
