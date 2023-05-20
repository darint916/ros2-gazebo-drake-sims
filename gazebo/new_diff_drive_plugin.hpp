#ifndef __NEW_DIFF_DRIVE_PLUGIN_H__
#define __NEW_DIFF_DRIVE_PLUGIN_H__

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Entity.hh>
using namespace ignition::gazebo;
namespace new_diff_drive_plugin 
{
    class NewDiffDrivePlugin: 
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemUpdate
    {
        public:
            void Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                EventManager &_eventMgr) override;

            void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm)override;
    };
}
        
#endif // __NEW_DIFF_DRIVE_PLUGIN_H__