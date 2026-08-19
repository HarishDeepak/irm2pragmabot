cmake_minimum_required(VERSION 3.8)
project(pragmabot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# NOTE: action file list intentionally left as a TODO.
# Open decision (see ../../../session_prompt.md): one collapsed
# ExecuteSkill.action, or PandaPick/PandaPlace/PandaPush kept separate
# (ported from repo_reference/IRM2's ROS1 .action files). Decide before
# uncommenting below and adding the actual .action file(s) under action/.
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/ExecuteSkill.action"
#   DEPENDENCIES geometry_msgs std_msgs
# )

ament_package()
