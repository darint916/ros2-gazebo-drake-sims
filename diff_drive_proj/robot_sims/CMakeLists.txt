cmake_minimum_required(VERSION 3.5)
project(robot_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-transport11 REQUIRED)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${ignition-transport11_INCLUDE_DIRS}
)


add_executable(robot_sim src/GazeboRosPositionPlugin.cpp)
ament_target_dependencies(robot_sim rclcpp geometry_msgs ignition-transport11)

install(TARGETS
    robot_sim
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
