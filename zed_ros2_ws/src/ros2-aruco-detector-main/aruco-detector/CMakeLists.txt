cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV ${OpenCV_VERSION} REQUIRED COMPONENTS aruco)
find_package(aruco_detector_msgs)


#add_library(${PROJECT_NAME} SHARED
#  src/aruco_detector.cpp
#)

add_executable(${PROJECT_NAME} src/aruco_detector.cpp)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  #rclcpp_components::component
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  image_transport::image_transport
  opencv_aruco
)

ament_target_dependencies(${PROJECT_NAME}
  aruco_detector_msgs
  cv_bridge
)


install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
