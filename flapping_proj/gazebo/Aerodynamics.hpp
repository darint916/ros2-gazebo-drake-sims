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
            <link>
                <link_name>link1</link_name>
                <link_type>Wing</link_type>
                <stall_angle>15</stall_angle>
                <fluid_density>1.293</fluid_density>
                <drag_coefficient>1</drag_coefficient>
                <lift_coefficient>1</lift_coefficient>
            </link>
            <link>
                <link_name>link2</link_name>
                <link_type>Generic</link_type>
                <stall_angle>15</stall_angle>
                <fluid_density>1.293</fluid_density>
                <drag_coefficient>1</drag_coefficient>
                <lift_coefficient>1</lift_coefficient>
            </link>
        </plugin>

            
    */
    struct AerodynamicLinkParameters //Defaults added for override + allocate
    {
        std::string linkType = "Generic"; //Wing, Generic, etc.
        double stallAngle = 0; //degrees
        double fluidDensity = 1.293; //kg*m^-3 air density at 273K
        double dragCoefficient = 1; //unitless + variable
        double liftCoefficient = 1; //unitless + variable
        WingParameters wingParameters; //Wing specific parameters
        std::vector<ignition::math::Vector3d> centerPressureList; //try ::Zero if err //meters from COM, Center of Pressure
    
        
    };

    struct WingParameters
    {
        int blades = 1; //number of blades for Blade Element method
        double wingSpan = 1; //meters (total length of wing)
        std::vector<double> bladeChordList; //meters (chord == width), vector length = blades
    }
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
        // ignition::msgs::Pose linkPose;
        linkMap<std::string, AerodynamicLinkParameters> linkMap;

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