#include<string>
// <gazebo/common/Console.hh> is for newer versions >:(
//Their current naming convention - they use gz instead of ignition, and then replaced gazebo with sim, so ignition::gazebo -> gz::sim
#include <ignition/common/Console.hh>

#include <ignition/plugin/Register.hh>

#include "test_plugin.hpp"

IGNITION_ADD_PLUGIN(
    test_plugin::Testing, //classname
    ignition::gazebo::System, //gz system
    test_plugin::Testing::ISystemPostUpdate //inheritance of interface
)

using namespace test_plugin;

//implemenetation of interface method
void Testing::PostUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &/*_ecm*/){
    std::string msg = "Test please work D: Simulation is currently ";
    if (!_info.paused) msg += "not ";
    msg += "paused.";

    ignmsg << msg << std::endl; //could need change, only works with running verbosity 3+
}