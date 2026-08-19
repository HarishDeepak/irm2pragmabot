# franka_ros2 Workspace — Detailed Structure

This document describes the full folder/file layout of this ROS 2 workspace (`/home/harish/ros2_ws/franka_ros2`), what each package does, and how the pieces fit together.

> Generated: 2026-07-24. Branch: `local-changes` (based on `humble`).

## 1. Overview

This is a ROS 2 (Humble) workspace built around **franka_ros2**, Franka Robotics' official ROS 2 integration of **libfranka** (the vendor SDK for controlling Franka Emika/Franka Robotics research arms). It bundles:

- The core `franka_ros2` ROS packages (hardware interface, controllers, description/URDF, MoveIt config, gripper, msgs, etc.)
- **libfranka** as a vendored/embedded C++ SDK (real-time robot control library).
- Custom/organization-specific additions: `franka_mobile`, `franka_mobile_sensors`, `olvx_descriptions_module` (mobile base + sensor suite integration, likely for a mobile manipulator platform built on top of the Franka arm — "tmr"/"olive" branded hardware).
- A vendored `zed-ros2-wrapper` (Stereolabs ZED camera ROS 2 driver) for depth/RGB camera integration.
- Docker-based development environment (Dockerfile, docker-compose.yml, devcontainer).

## 2. Top-Level Layout

```
franka_ros2/
├── CHANGELOG.rst              # Aggregate changelog for the meta-repo
├── CONTRIBUTING.md            # Contribution guidelines
├── dependency.repos           # vcstool file listing external repo dependencies to clone
├── docker-compose.yml         # Local dev container definition (see §7)
├── Dockerfile                 # Image build for the ROS 2 humble dev/runtime container
├── franka_entrypoint.sh       # Container entrypoint script
├── Jenkinsfile                # CI pipeline definition (Jenkins)
├── LICENSE / NOTICE           # Apache-2.0 licensing
├── limits.conf                # Real-time scheduling limits (rtprio/memlock) mounted into container
├── pyproject.toml             # Python tooling config (formatting/linting) for the repo
├── README.md                  # Main project README (setup, install, troubleshooting)
├── docs/                      # Sphinx/RST documentation source
├── .devcontainer/             # VS Code devcontainer config
├── .github/                   # GitHub Actions workflows
│
├── franka_bringup/            # Launch files & runtime configs to bring up the robot with ros2_control
├── franka_description/        # URDF/xacro + meshes for Franka robots (arm variants, end effectors, accessories)
├── franka_example_controllers/# Example ros2_control controllers (motion generators, gravity comp, etc.)
├── franka_fr3_moveit_config/   # MoveIt2 configuration for the FR3 arm
├── franka_gazebo_bringup/      # Gazebo simulation launch files, worlds, and sim-specific URDF
├── franka_gripper/             # ROS 2 driver/action server for the Franka Hand gripper
├── franka_hardware/            # ros2_control hardware_interface implementation wrapping libfranka
├── franka_mobile/              # Mobile base (swerve drive) controller package — custom addition
├── franka_mobile_sensors/      # Sensor suite (lidars/cameras) launch & config for the mobile base
├── franka_msgs/                # Custom ROS messages, actions, and services (FrankaRobotState, ErrorRecovery, Grasp, ...)
├── franka_robot_state_broadcaster/ # ros2_control broadcaster publishing FrankaRobotState
├── franka_ros2/                # Meta-package (CMakeLists/package.xml only — ties the repo together for colcon)
├── franka_semantic_components/ # Semantic component wrappers (Cartesian pose/velocity, robot model/state) for controllers
├── libfranka/                  # Vendored C++ SDK for real-time robot/gripper control (used by franka_hardware)
├── olvx_descriptions_module/   # URDF/meshes for Olive Robotics sensor/chassis modules (used on the mobile platform)
└── zed-ros2-wrapper/           # Vendored Stereolabs ZED camera ROS 2 driver (components, wrapper node, debug tools)
```

Each ROS package directory generally follows the standard ament/colcon layout: `CMakeLists.txt`, `package.xml`, `src/`, `include/<pkg_name>/`, `launch/`, `config/`, `test/`, `doc/`.

## 3. Core Franka ROS 2 Packages

### `franka_bringup`
Launch files and runtime configuration for bringing up a real or simulated Franka robot with `ros2_control`.
- `config/`: `controllers.yaml`, `franka.config.yaml`, `tmr.config.yaml`, `xbox.config.yaml` (controller manager & robot params, plus Xbox teleop mapping)
- `launch/`: `franka.launch.py` (main bringup), `example.launch.py`, `joint_impedance_with_ik_example_controller.launch.py`, `mobile_teleop.launch.py`
- `franka_bringup/`: Python module (`launch_utils.py`, `testing/`) shared by launch files
- `test/`: hardware-in-the-loop launch tests (`test_hardware_example_controllers.py`, `test_hardware_generic_controller.py`)

### `franka_description`
URDF/xacro descriptions and meshes for all supported robot variants.
- `robots/`: per-variant xacro — `fer`, `fp3`, `fr3`, `fr3_duo`, `fr3v2`, `fr3v2_1`, `mobile_fr3_duo_v0_2`, `tmrv0_2`, plus `common/`
- `end_effectors/`: `franka_hand` (gripper), `cobot_pump`, `common`
- `accessories/`: `fr3_duo_mount_v0_3`, `franka_head_v0_2`, `franka_spine_v0_1` — custom mounting hardware
- `meshes/`: `robots/`, `robot_ee/`, `accessories/` (STL/DAE visual & collision meshes)
- `scripts/`: `create_urdf.py`/`.sh` (xacro→URDF generation), `visualize_franka.sh`
- `launch/visualize_franka.launch.py`, `rviz/visualize_franka.rviz`
- `test/urdf_tests.py`

### `franka_example_controllers`
Example `ros2_control` controllers demonstrating Franka control modes.
- `src/motion_generator.cpp`, `src/async_motions/`, `src/fr3/`
- `include/franka_example_controllers/`
- `franka_example_controllers.xml` — plugin export description for `pluginlib`
- `test/`: gravity compensation and move-to-start controller tests

### `franka_fr3_moveit_config`
MoveIt2 configuration for the FR3 arm.
- `config/`: `fr3_controllers.yaml`, `fr3_ros_controllers.yaml`, `kinematics.yaml`, `ompl_planning.yaml`
- `launch/`: `move_group.launch.py`, `moveit.launch.py`
- `rviz/moveit.rviz`

### `franka_gazebo_bringup`
Gazebo simulation bring-up.
- `launch/`: arm example controller, mobile robot (`gazebo_mobile_robot.launch.py`), TMR example controller, visualization
- `urdf/`: `franka_arm.gazebo.xacro`, `sensors.xacro`, `tmrv0_2_with_sensors.gazebo.urdf.xacro` — Gazebo-specific plugin wrappers
- `worlds/`: `empty_no_gravity.sdf`, `mobile_fr3_duo_sensors.sdf`, `sensor_demo_world.sdf`
- `config/franka_gazebo_controllers.yaml`
- `test/`: `test_gazebo_franka_arm_example_controller.py`, `test_gazebo_tmr_example_controller.py`

### `franka_gripper`
Driver for the Franka Hand gripper (action server exposing Grasp/Move/Homing actions).
- `src/gripper_action_server.cpp`, `include/franka_gripper/`
- `franka_gripper/__init__.py` — Python bindings/module
- `scripts/fake_gripper_state_publisher.py` — simulated gripper state for testing without hardware
- `config/franka_gripper_node.yaml`, `launch/gripper.launch.py`
- `test/test_hardware_franka_gripper_position.py`

### `franka_hardware`
The `ros2_control` `hardware_interface` implementation — the bridge between ROS 2 and `libfranka`.
- `src/franka_hardware_interface.cpp` — main hardware interface (read/write joint states & commands)
- `src/robot.cpp` — wraps libfranka's `Robot` object
- `src/franka_action_server.cpp` — exposes error recovery / PTP motion actions
- `src/franka_executor.cpp`, `src/franka_param_service_server.cpp`, `src/ros_libfranka_logger.cpp`
- `franka_hardware.xml` — hardware_interface plugin export
- `test/`: extensive unit/integration tests (action server, cartesian command interface, hardware interface, robot, PTP motion) including sample URDFs (`fr3.urdf`, `fr3_unsupported_version.urdf`)

### `franka_mobile` (custom/organization package)
Swerve-drive mobile base controller (for a mobile manipulator platform, "tmr").
- `src/swerve_drive_controller.cpp`, `src/swerve_ik_controller.cpp`, `src/swerve_kinematics.cpp`, `src/odometry.cpp`, `src/urdf_utils.cpp/.hpp`
- `controllers.xml` — controller plugin export
- `config/swerve_drive_controller_parameters.yaml`
- `test/test_odometry.cpp`, `test/test_swerve_kinematics.cpp`

### `franka_mobile_sensors` (custom/organization package)
Sensor suite integration (lidars, cameras) for the mobile base.
- `config/`: `default_sensor_suite.yaml`, `cameras/`, `lidars/`
- `launch/`: `franka_mobile_sensors.launch.py`, plus subfolders `cameras/`, `lidars/`, `utils/`, `visualization/`
- `robots/`: `nanoscan3` (lidar model), `tmrv0_2_with_sensors.urdf.xacro`
- `rviz/tmr_sensors.rviz`

### `franka_msgs`
Custom ROS 2 interface definitions.
- `action/`: `ErrorRecovery`, `Grasp`, `Homing`, `Move`, `PTPMotion`
- `msg/`: `CollisionIndicators`, `Elbow`, `Errors`, `FrankaRobotState`, `GraspEpsilon`, `TargetStatus`
- `srv/`: `SetCartesianStiffness`, `SetForceTorqueCollisionBehavior`, `SetFullCollisionBehavior`, `SetJointStiffness`, `SetLoad`, `SetStiffnessFrame`, `SetTCPFrame`

### `franka_robot_state_broadcaster`
`ros2_control` broadcaster that publishes `FrankaRobotState` at real-time rate.
- `src/franka_robot_state_broadcaster.cpp`, `src/franka_robot_state_broadcaster_parameters.yaml`
- `franka_robot_state_broadcaster.xml` — plugin export
- `test/`: parameter loading and broadcaster tests

### `franka_ros2` (meta-package)
Ties the whole repo together as a single colcon/ament meta-package (`CMakeLists.txt`, `package.xml`, `doc/compatibility_matrix.rst`, `scripts/run_hardware_tests.sh`). `CHANGELOG.rst` is a symlink to the top-level changelog.

### `franka_semantic_components`
Reusable "semantic component" wrappers used by controllers/broadcasters to read/write hardware interfaces semantically (Cartesian pose/velocity, robot model, robot state).
- `src/franka_cartesian_pose_interface.cpp`, `franka_cartesian_velocity_interface.cpp`, `franka_robot_model.cpp`, `franka_robot_state.cpp`, `franka_semantic_component_interface.cpp`, `translation_utils.cpp/.hpp`
- `test/`: matching test suite per component, plus `robot_description_test.txt`

## 4. `libfranka` — Vendored SDK

Standalone C++ library (not ROS-specific) providing real-time control of the robot arm and gripper over the network. Consumed by `franka_hardware`.

- `include/franka/` — public API headers
- `src/` — implementation: `robot.cpp`, `robot_impl.cpp`, `robot_control.h`, `robot_model.cpp`, `robot_state.cpp`, `active_control.cpp`, `active_motion_generator.cpp`, `active_torque_control.cpp`, `control_loop.cpp`, `control_tools.cpp`, `control_types.cpp`, `duration.cpp`, `errors.cpp`, `exception.cpp`, `gripper.cpp`, `gripper_state.cpp`, `joint_velocity_limits.cpp`, `load_calculations.cpp`, `lowpass_filter.cpp`, `model.cpp`, `network.cpp`, `rate_limiting.cpp`, `vacuum_gripper.cpp`, plus `async_control/` and `logging/` subfolders
- `common/` — shared headers/lib used across libfranka and its bindings
- `pylibfranka/` — Python bindings (pybind11-style) with its own `src/`, `include/`, `examples/`, `scripts/`, `docs/`
- `examples/` — ~20 standalone C++ example programs (Cartesian/joint impedance, velocity/position motion generation, grasping, echoing robot state, etc.)
- `test/` — extensive GoogleTest suite (robot, gripper, model, control loop, rate limiting, mock server/robot for hardware-free testing)
- `cmake/` — CMake find-modules (`FindEigen3`, `FindPoco`, `FindTinyXML2`), FetchFMT, version-from-git
- `docs/` — Sphinx RST docs (architecture, installation, migration notes, real-time kernel setup, system requirements)
- `doc/` — Doxygen config (`Doxyfile.in`) for API reference generation
- Build/packaging: `CMakeLists.txt`, `package.xml`, `pyproject.toml`, `setup.py`, `setup.cfg`, `requirements.txt`, `codecov.yml`, `Jenkinsfile`

## 5. `olvx_descriptions_module`

URDF/mesh descriptions for individual sensor/chassis modules from Olive Robotics, used to build up the mobile platform's description (referenced by `franka_mobile_sensors`/`franka_description` mobile variants).
- `urdf/`: pairs of `.urdf` + `.urdf.xacro` for each module — camera (`olv-cam01`), chassis, GNSS (`olv-gnx01`), IMU, N5G, radar, servo (`olv-srv01`, incl. `-body`/`-horn`), wheel
- `meshes/`: matching `.dae` visual meshes (plus `olv-chassis01.zip`)
- `launch/`: one `visualize_olive_*.launch.py` per module for standalone RViz visualization
- `rviz/visualize_olive.rviz`

## 6. `zed-ros2-wrapper` (vendored, untracked addition)

Stereolabs' official ZED camera ROS 2 driver, vendored into this workspace (currently untracked in git — see §8) to provide RGB-D camera support for the mobile platform.
- `zed_wrapper/` — main driver node package (`config/`, `launch/`, `urdf/`)
- `zed_components/` — reusable ROS 2 components/nodelets (`src/`)
- `zed_ros2/` — meta-package
- `zed_debug/` — debugging tools (`config/`, `launch/`, `src/`)
- `docker/` — Dockerfiles for desktop (Ubuntu) and Jetson (L4T) builds, entrypoint scripts
- `images/` — README screenshots/GIFs

## 7. Docker / Dev Environment

- **`Dockerfile`** — builds the `ros-humble` based development/runtime image for franka_ros2 (installs ROS deps, libfranka prerequisites, etc.)
- **`docker-compose.yml`** — defines the `franka_ros2_humble` service:
  - Mounts the repo at `/ros2_ws/src`, X11 socket for GUI (RViz/Gazebo), `/dev` for hardware access, and `limits.conf` for real-time scheduling
  - Also bind-mounts a host path `/home/harish/pragmabot` → `/pragmabot` (local, host-machine-specific addition)
  - Runs with `network_mode: host`, `ipc: host`, `privileged: true`, `SYS_NICE` capability, and raised `rtprio`/`memlock` ulimits — required for **real-time control loops** talking to the robot over UDP/TCP
  - `ROS_DOMAIN_ID=7` set explicitly to isolate this workspace's ROS graph
- **`franka_entrypoint.sh`** — container entrypoint (sourcing the workspace overlay, etc.)
- **`.devcontainer/`** — VS Code Dev Containers integration
- **`limits.conf`** — raises `rtprio`/`memlock` limits for the container user (needed for libfranka's real-time thread)

## 8. Documentation, CI, and Misc

- **`docs/`** — top-level Sphinx docs (`index.rst`, `assets/frames.svg`, `assets/move_groups.png`)
- **`.github/`** — GitHub Actions workflow definitions (CI badge referenced in README)
- **`Jenkinsfile`** (top-level and inside `libfranka/`, `franka_description/`) — Jenkins CI pipelines
- **`CHANGELOG.rst`** — combined changelog (ROS `catkin`/`ament`-style, per-package changelogs also exist and many symlink or duplicate entries here)
- **`CONTRIBUTING.md`** — contribution guidelines
- **`.clang-format` / `.clang-tidy`** — C++ style/lint configuration
- **`pyproject.toml`** (top-level and in `libfranka/`) — Python tooling config (e.g. formatting/linting for Python launch files and scripts)
- **`dependency.repos`** — `vcstool` manifest listing external git repos this workspace depends on (fetched via `vcs import` during setup)

### Untracked / local-only items (per `git status` at time of writing)
- `.env~` — empty leftover file (stray backup of `.env`)
- `zed-ros2-wrapper/` — vendored ZED driver, not yet added to git
- Local `docker-compose.yml` changes (host-mount for `pragmabot`, `ipc: host`, env format) are uncommitted-adjacent local container config per recent commit history

## 9. How the Pieces Fit Together (Data Flow)

1. **`libfranka`** talks directly to the robot controller over the network (real-time UDP/TCP), exposing a C++ API for reading state and commanding motion/torque.
2. **`franka_hardware`** implements a `ros2_control` `SystemInterface` on top of `libfranka`, exposing joint state/command interfaces to the ROS 2 control stack.
3. **`franka_robot_state_broadcaster`** and **`franka_semantic_components`** read from that hardware interface to publish rich state (`franka_msgs/FrankaRobotState`) and provide semantic (Cartesian) read/write helpers to controllers.
4. **`franka_example_controllers`** (and any custom controllers) are loaded by `controller_manager` and use the semantic components / hardware interfaces to move the robot.
5. **`franka_gripper`** provides a separate action-based interface for the Franka Hand, independent of the main arm control loop.
6. **`franka_bringup`** and **`franka_gazebo_bringup`** launch all of the above together, either against real hardware or Gazebo simulation, using URDFs from **`franka_description`** (and, for the mobile platform, **`olvx_descriptions_module`** + **`franka_mobile_sensors`** meshes/URDF).
7. **`franka_mobile`** adds swerve-drive base control for a mobile manipulator variant, with **`franka_mobile_sensors`** wiring up lidars/cameras (including the vendored **`zed-ros2-wrapper`**) for navigation/perception.
8. **`franka_fr3_moveit_config`** layers MoveIt2 motion planning on top of the same hardware/controller stack for the FR3 arm variant.
9. Everything is built/run inside the Docker image defined by **`Dockerfile`**/**`docker-compose.yml`**, which grants the real-time scheduling privileges libfranka's control loop requires.
