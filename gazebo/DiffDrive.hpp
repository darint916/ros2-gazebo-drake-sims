#ifndef __DIFF_DRIVE_PLUGIN_H__
#define __DIFF_DRIVE_PLUGIN_H__

#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Entity.hh>

namespace new_plugins
{
    struct InputCmd
    {
        double lin = 0.0;
        double ang = 0.0;
    };

    struct OdomData
    {
        ignition::math::Pose3d pose;
        ignition::math::Vector3d linVel;
        ignition::math::Vector3d angVel;
    };
    
    class DiffDriveData
    {
        public:
            Model model{ignition::gazebo::kNullEntity}; //main model, simulator entrypoint set in config

            ignition::transport::Node node; //node for ign topics
            ignition::transport::Node::Publisher odomPub; //publisher for odometry, sub in other class
            std::chrono::steady_clock::duration odomPubPeriod;
            std::chrono::steady_clock::duration prevOdomPubTime = 0;
            ignition::msgs::Twist targetAcc;
            std::mutux mutex; //protect target acc cmd
            //perhaps implement speed limiters in future

            InputCmd prevCmd;
            InputCmd prevprevCmd;

            std::vector<ignition::gazebo::Entity> leftJoints;
            std::vector<ignition::gazebo::Entity> rightJoints;
            std::vector<std::string> leftJointNames;
            std::vector<std::string> rightJointNames;
            double wheelRadius = 0.2;
            double wheelSeparation = 1.0; //rand def values

    };

    class DiffDrivePlugin
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure, 
        public ignition::gazebo::ISystemPreUpdate
    {
        private:
            std::unique_ptr<DiffDriveData> dataPtr;
        
        public:
            DiffDrivePlugin();
            ~DiffDrivePlugin();

            void Configure(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr) override;

            void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

            void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
    };
}

#endif // __DIFF_DRIVE_PLUGIN_H__