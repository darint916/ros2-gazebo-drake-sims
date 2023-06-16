#ifndef __AERODYNAMICS_HPP__
#define __AERODYNAMICS_HPP__
#include <memory>
#include <string>
#include <ignition/gazebo/System.hh>

namespace aerodynamics 
{
    namespace igz = ignition::gazebo;
    /*
        SDF Parameter Shape
        <plugin filename="aerodynamics_plugin" name="aerodynamics">
            
    */
    struct AerodynamicsData
    {
        igz::Model model{igz::kNullEntity};
        ignition::transport::Node node;
        ignition::transport::Node::Publisher aerodynamicsPublisher;

        std::chrono::steady_clock::duration aerodynamicsPubPeriod;
        std::chrono::steady_clock::duration prevAerodynamicsPubTime = std::chrono::steady_clock::duration::zero();
        std::mutex mutex;

        std::string modelName;
        std::string linkType; //Wing, Generic, etc.

        double stallAngle = 0; //degrees

        bool reportData = false;
        bool validConfig = false;
        bool initialized = false;
        sdf::ElementPtr sdfConfig;

        //Sim parameters
        ignition::msgs::Pose linkPose;
        

        //sdf parameters
        double fluidDensity = 1.293; //kg*m^-3 air density at 273K
        double dragCoefficient = 1; //unitless + variable
        double liftCoefficient = 1; //unitless + variable

        void Load(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf);
        void Update(EntityComponentManager &_ecm);
    }
    
    class Aerodynamics : public igz::System, public igz::ISystemConfigure, igz:: ISystemPostUpdate
    {
        private:
            std::unique_ptr<AerodynamicsData> dataPtr;
        public:
            Aerodynamics();
            ~Aerodynamics() override = default;
            void Configure(const igz::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, igz::EntityComponentManager &_ecm, igz::EventManager &_eventMgr) override;
            void PostUpdate(const igz::UpdateInfo &_info, const igz::EntityComponentManager &_ecm) override;
    };
}
#endif // __AERODYNAMICS_HPP__