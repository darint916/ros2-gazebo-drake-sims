cmake_minimum_required(VERSION 3.8)
project(robot_controller)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-fcoroutines)
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(drake REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include) # for the header files (pid.h, etc.)

# add_library(pid_controller_lib SHARED
#   src/pid_controller.cpp
# )

# target_include_directories(pid_controller_lib PUBLIC
#   ${PROJECT_SOURCE_DIR}/include
# )


add_executable(controller_node src/robot_controller.cpp src/pid_controller.cpp)
ament_target_dependencies(controller_node rclcpp robot_msgs geometry_msgs nav_msgs)

# target_link_libraries(controller_node pid_controller_lib)



add_executable(trajectory_server src/trajectory_server.cpp)
ament_target_dependencies(trajectory_server rclcpp robot_msgs geometry_msgs nav_msgs)



install(TARGETS
  controller_node
  trajectory_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
