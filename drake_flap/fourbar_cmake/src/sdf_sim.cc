/* @file
A four bar linkage demo demonstrating the use of a linear bushing as
a way to model a kinematic loop. It shows:
  - How to model a four bar linkage in SDF.
  - Use the multibody::Parser to load a model from an SDF file into a
    MultibodyPlant.
  - Model a revolute joint with a LinearBushingRollPitchYaw to model a closed
    kinematic chain.

Refer to README.md for more details.
*/

#include <cmath>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/revolute_joint.h"
// Removed: #include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake
{

    using Eigen::Vector3d;

    using geometry::SceneGraph;
    using multibody::Frame;
    using multibody::LinearBushingRollPitchYaw;
    using multibody::MultibodyPlant;
    using multibody::Parser;
    using multibody::RevoluteJoint;
    using systems::Context;
    using systems::DiagramBuilder;
    using systems::Simulator;

    namespace examples
    {
        namespace multibody
        {
            namespace four_bar
            {
                namespace
                {

                    int do_main()
                    {
                        // Create a DiagramBuilder and a MultibodyPlant with zero time step (continuous).
                        DiagramBuilder<double> builder;
                        auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
                        auto scene_graph = builder.AddSystem(std::make_unique<SceneGraph<double>>());
                        plant->RegisterAsSourceForSceneGraph(scene_graph);

                        // Keep a reference to the plant for convenience.
                        MultibodyPlant<double> &plant_ref = *plant;
                        // Add the plant into the diagram.
                        builder.AddSystem(std::move(plant));

                        // Load the four bar model from an SDF file.
                        const std::string sdf_url =
                            "package://drake/examples/multibody/four_bar/four_bar.sdf";
                        Parser parser(&plant_ref);

                        return 0;
                    }

                } // namespace
            } // namespace four_bar
        } // namespace multibody
    } // namespace examples
} // namespace drake

int main(int argc, char *argv[])
{
    gflags::SetUsageMessage(
        "A four bar linkage demo demonstrating the use of a linear bushing as "
        "a way to model a kinematic loop. Launch meldis before running this "
        "example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::multibody::four_bar::do_main();
}
