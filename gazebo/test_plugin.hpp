#ifndef __TEST_PLUGIN_H__
#define __TEST_PLUGIN_H__

#include <ignition/gazebo/System.hh>

namespace test_plugin 
{
    class Testing: public ignition::gazebo::System, public ignition::gazebo::ISystemPostUpdate
    {
        // public: void PostUpdate(const gz::common::UpdateInfo &info, const gz::common::Entity &entity) override;
        public:
            void PostUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm) override;
    };
}

#endif // __TEST_PLUGIN_H__