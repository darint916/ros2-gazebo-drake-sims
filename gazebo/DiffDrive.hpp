#ifndef __DIFF_DRIVE_PLUGIN_H__
#define __DIFF_DRIVE_PLUGIN_H__

#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Model.hh>

namespace diff_drive
{
    struct InputCmd
    {
        double linearAcceleration = 0.0;
        double angularAcceleration = 0.0;
    };

    struct OdomData
    {
        std::chrono::steady_clock::duration time;
        double wheelSeparation;
        double wheelRadius;
        double leftJointForce;
        double rightJointForce;
        ignition::math::Pose3d pose;
        double linVel;
        double linAcc;
        double angVel;
        double angAcc;
        bool initialized = false;
    };
    
    //Shouldve just used as a struct, but too late now (since no need for methods)
    class DiffDriveData
    {
        public:
            ignition::gazebo::Model model{ignition::gazebo::kNullEntity}; //main model, simulator entrypoint set in config

            ignition::transport::Node node; //node for ign topics
            ignition::transport::Node::Publisher odomPublisher; //publisher for odometry, sub in other class
            std::chrono::steady_clock::duration odomPubPeriod;
            std::chrono::steady_clock::duration prevOdomPubTime = std::chrono::steady_clock::duration::zero();
            OdomData odomData;
            ignition::msgs::Twist targetAcceleration;
            std::mutex mutex; //protect target acc cmd
            //perhaps implement speed limiters in future

            InputCmd prevCmd;
            InputCmd prevprevCmd;

            ignition::gazebo::Entity canonicalLink;
            std::vector<ignition::gazebo::Entity> leftJoints;
            std::vector<ignition::gazebo::Entity> rightJoints;
            std::vector<std::string> leftJointNames;
            std::vector<std::string> rightJointNames;
            double wheelRadius = 0.2;
            double wheelSeparation = 1.0; //rand def values

            //TODO: Should be vector of inertia, or access child link from joint and update from there
            //also create struct that combines all joint data
            double leftLinkInertia = .08;
            double rightLinkInertia = .08;

            double leftJointTorque = 0;
            double rightJointTorque = 0;

            void InputCmdCallback(const ignition::msgs::Twist &_msg);
    };

    class DiffDrive
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure, 
        public ignition::gazebo::ISystemPreUpdate,
        public ignition::gazebo::ISystemPostUpdate
    {
        private:
            std::unique_ptr<DiffDriveData> dataPtr;
        
        public:
            DiffDrive();
            ~DiffDrive();

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