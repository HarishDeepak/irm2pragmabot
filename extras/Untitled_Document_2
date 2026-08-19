ros2 run aruco_detector aruco_detector --ros-args --remap image:=/zed/zed_node/rgb/color/rect/image --remap camera_info:=/zed/zed_node/rgb/color/rect/image/camera_info -p marker_size:='0.05' -p image_is_rectified:=true

ros2 launch easy_handeye2 calibrate.launch.py name:=fr3_zed_right calibration_type:='eye_on_base' tracking_base_frame:='zed_camera_link' tracking_marker_frame:='marker_0' robot_base_frame:='fr3_link0' robot_effector_frame:='fr3_hand'
