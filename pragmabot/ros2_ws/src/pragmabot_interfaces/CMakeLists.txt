cmake_minimum_required(VERSION 3.8)
project(pragmabot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Decision resolved: ONE collapsed ExecuteSkill.action, not separate
# PandaPick/PandaPlace/PandaPush. The planner emits a single `chosen_skill`
# enum inside one NextBestAction, and bridge_node dispatches all three down
# a shared path - three actions would split what is already one field.
# Rationale in full at the top of action/ExecuteSkill.action.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteSkill.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
