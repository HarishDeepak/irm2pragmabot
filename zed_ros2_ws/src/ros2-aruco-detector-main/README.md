# ros2-aruco-detector

In the ARCOS-Lab we recently bought a camera that is able to capture in 2K. We
found that the available aruco trackers for ROS2 were too slow. For the most
part they are implemented in Python (which is very slow) and the one in cpp is
still very slow. Taking into consideration that our Humanoid robot server has a
lot of cores but each core has a mild single thread performance, we tried to
implement an aruco tracker that is able to take advantage of our strengths. This
means that this implementation uses a lot more parallelization and more memory
intensive.

At the moment there are only 3 parameters:
marker_size (double): the size of the aruco marker
marker_dict (string): the dictionary to use, for example 4x4_50
resize_factor (double): if necessary, a resizing factor to change the size of
the incoming camera image.

# Pearl Instructions
```shell
# clone into your src of colcon workspace
git clone git@gitlab.pearl.informatik.tu-darmstadt.de:lab/franka/ros2-aruco-detector.git
colcon build
source install/setup.bash
```

## Understanding the Options

### Parameters
- marker_size (double): the size of the aruco marker in meters
- marker_dict (string): the aruco dictionary to use (default: 4x4_50)
- resize_factor (double): if necessary, a resizing factor to change the size of
the incoming camera image.
- image_is_rectified (bool): if the incoming image is rectified

### Remap Topics
- image: The image topic to subscribe to
- camera_info: The camera info topic to subscribe to

## Usage Tips:

- The aruco marker that you print is not exactly the size you want it to be. Use the exact size. Measure it by hand.
- Use the rectified channel of image.
- Where you stick the marker, make sure it is flat and not tilted.
- The tracker will publish the marker pose in a `MarkerPoseStamped` array. (`/aruco_detection
` topic name)
- The tracker also publishes a tf transform for the same and names the frame of the marker as `marker_x`. Where x is the id of the marker.

- You can pair this with systems like [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)
  - Use Daniilidis algorithm to calibrate the robot's end-effector pose relative to the marker as it is more robust for real world applications.
  - Where you stick the marker does not matter, but the frame you stick it to matters.
  - `tracking_base_frame` is the last link of the camera that get's connected to the scene with a static transform.
  - `tracking_marker_frame` is the frame of the marker that is given by the tracker.
  - Only use samples where you see the pose marker not shaking in the camera frame. (Use rviz camera visualization to check this with `marker_x` tf on )
  

# Working Example for Franka Emika Fr3 Poseidon Robot

The example assumes the aruco marker is stuck on the robot's hand.
```shell
# Right Cam

# Start the tracker
ros2 run aruco_detector aruco_detector --ros-args --remap image:=/zedr/zed_node/left/image_rect_color --remap camera_info:=/zedr/zed_node/left/camera_info -p marker_size:='0.053' -p image_is_rectified:=true

# Start Easy Handeye 2
ros2 launch easy_handeye2 calibrate.launch.py name:=fr3_zed_right calibration_type:='eye_on_base' tracking_base_frame:='zedr_camera_link' tracking_marker_frame:='marker_5' robot_base_frame:='fr3_link0' robot_effector_frame:='fr3_hand'

# Left Cam

# Start the tracker
ros2 run aruco_detector aruco_detector --ros-args --remap image:=/zedl/zed_node/left/image_rect_color --remap camera_info:=/zedl/zed_node/left/camera_info -p marker_size:='0.053' -p image_is_rectified:=true 

# Start Easy Handeye 2
ros2 launch easy_handeye2 calibrate.launch.py name:=fr3_zed_left calibration_type:='eye_on_base' tracking_base_frame:='zedl_camera_link' tracking_marker_frame:='marker_5' robot_base_frame:='fr3_link0' robot_effector_frame:='fr3_hand'
```
