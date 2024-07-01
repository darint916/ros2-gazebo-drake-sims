#ifndef __ODOMETRY_STATE_HPP__
#define __ODOMETRY_STATE_HPP__

#include <memory>
#include <string>
#include <ignition/gazebo/System.hh>

namespace odometry_state
{
    struct OdometryStateData 
    {
        ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
        ignition::transport::Node node; //used directly for subscription, indirectly for publishing
        ignition::transport::Node::Publisher odomPublisher; //subclass to store node.Advertise(), publishing
        // ignition::transport::Node::Publisher jointAnglePublisher;                                                                                                                                                                                                                                                                                                                                                                               

        std::chrono::steady_clock::duration odomPubPeriod;
        std::chrono::steady_clock::duration prevOdomPubTime = std::chrono::steady_clock::duration::zero();
        std::mutex mutex;
        std::string modelName;
        // ignition::gazebo::Entity canonicalLink;
        
        int modelPositionIndex = -1;
        std::map<std::string, ignition::gazebo::Entity> jointMap;
        std::map<std::string, ignition::gazebo::Entity> inputJointMap;
        std::map<std::string, std::string> inputJointTopicMap;
    
        void PositionCallBack(const ignition::msgs::Pose_V &_msg);
        ignition::msgs::Pose_V poseMsg; //for incoming subscribed pose data
    };

    class OdometryState :
        public ignition::gazebo::System, 
        public ignition::gazebo::ISystemConfigure, 
        // public ignition::gazebo::ISystemPreUpdate,
        public ignition::gazebo::ISystemPostUpdate
    {
        private:
            std::unique_ptr<OdometryStateData> dataPtr;

        public:
            OdometryState();
            ~OdometryState() override = default;

            void Configure(const ignition::gazebo::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, ignition::gazebo::EntityComponentManager &_ecm, ignition::gazebo::EventManager &_eventMgr) override;
            // void PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm) override;
            void PostUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm) override;            
    };
}

#endif // __ODOMETRY_STATE_HPP__
