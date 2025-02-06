# SPDX-License-Identifier: MIT-0

# Define the executable target for fourbar_c.
add_executable(sdf_sim sdf_sim.cc)

# Link against Drake (which includes the multibody and systems components) and Threads.
target_link_libraries(sdf_sim drake::drake Threads::Threads)

# (Optional) If you need to link additional Drake targets (e.g., for parsing, plant, tree, etc.),
# you could list them explicitly. For many cases, linking drake::drake suffices.

# Define a test that runs the executable with a command‑line argument similar to the Bazel test_rule_args.
add_test(NAME sdf_sim_test COMMAND sdf_sim --simulation_time=0.1)
set_tests_properties(sdf_sim_test PROPERTIES TIMEOUT 60)
