cmake_minimum_required(VERSION 3.9)

project(pragmabot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  topic_tools
  tf
  actionlib
  actionlib_msgs
  message_generation
)

catkin_python_setup()

add_action_files(
  FILES
  PandaPick.action
  PandaPlace.action
  PandaPush.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    geometry_msgs
    cv_bridge
    message_filters
    topic_tools
    tf
    actionlib actionlib_msgs geometry_msgs message_runtime
)

catkin_install_python(
  PROGRAMS
    nodes/pragmabot_node.py
    nodes/image_decompressor_node.py
    nodes/image_republisher_node.py
    nodes/memory_manager_node.py
    nodes/panda_pick_server.py
    nodes/panda_place_server.py
    nodes/panda_push_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)