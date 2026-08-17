# Assignment 2: Gazebo–ROS 2 Bridging and Lane-Following Race

## my name : Sabah Mohammed  Id: 1404-5-35
## the video link:
## A bag file link:
## A video link:
## The lap time achieved, as shown in your video:

This repository contains a ROS 2 Jazzy project for controlling a simulated Toyota Prius in Gazebo Harmonic. The project demonstrates communication between Gazebo and ROS 2 through `ros_gz_bridge`, camera-based lane perception, and a lane-following controller that publishes velocity commands to the simulated vehicle.



## Documentation:
## Project Objectives

The project implements the following pipeline:

```text
Gazebo Harmonic camera
        |
        | ros_gz_bridge
        v
ROS 2 image topic: /prius/front_camera/image_raw
        |
        v
lane_perception / lane_detector
        |
        +--> /lane_error
        +--> /lane_status
        +--> /lane_perception/debug_image
        |
        v
lane_control / lane_controller
        |
        v
/cmd_vel
        |
        | ros_gz_bridge
        v
Gazebo Prius vehicle
```

## Software Requirements

| Component | Required version |
|---|---|
| Operating system | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic / Gazebo Sim 8 |
| Programming language | Python 3 |
| Build system | `colcon` with `ament_python` |
| Main computer-vision library | OpenCV |

The official ROS 2 Jazzy installation guide is available at [ROS 2 Jazzy Ubuntu installation][1]. The official Gazebo ROS installation documentation is available at [Gazebo and ROS 2 integration][2].

## Repository Structure

```text
assignment2_ws/
├── README.md
├── .gitignore
├── config/
│   └── gz_sim_bridge_car.yaml
└── src/
    ├── lane_perception/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── lane_perception
    │   └── lane_perception/
    │       ├── __init__.py
    │       └── lane_detector.py
    └── lane_control/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── lane_control
        └── lane_control/
            ├── __init__.py
            └── lane_controller.py
```

The generated folders `build/`, `install/`, and `log/` are intentionally excluded from GitHub because they are local build products and can be regenerated on another computer.

## Installation and Workspace Setup

Open a terminal and source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Create the workspace if it does not already exist:

```bash
mkdir -p ~/assignment2_ws/src
cd ~/assignment2_ws
```

Install the required build tools if necessary:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions colcon
```

Install package dependencies and build the workspace:

```bash
cd ~/assignment2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launching the Sonoma Raceway World

The project uses the Prius on Sonoma Raceway world. If the world was downloaded through Gazebo Fuel, it may be available at a path similar to the following:

```bash
gz sim -r "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/prius on sonoma raceway/1/sonoma.sdf"
```

The exact path can differ between computers. To search for the world file, use:

```bash
find ~/.gz ~/Downloads -type f \( -iname "sonoma.sdf" -o -iname "*.world" \) 2>/dev/null
```

Keep the Gazebo terminal open while the simulation is running. In the Gazebo GUI, press the play button if the simulation is paused.

## Starting the ROS–Gazebo Bridge

Open a second terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:=$HOME/assignment2_ws/config/gz_sim_bridge_car.yaml
```

Keep this terminal open. Verify that the expected topics are available:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | sort
```

Important topics include:

```text
/clock
/cmd_vel
/odom
/prius/front_camera/image_raw
/prius/left_camera/image_raw
/prius/right_camera/image_raw
/tf
```

Check that the front camera is publishing images:

```bash
timeout 5 ros2 topic hz /prius/front_camera/image_raw
```

A non-zero average rate confirms that images are being transferred from Gazebo to ROS 2.

## Running Lane Perception

Open a third terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/assignment2_ws/install/setup.bash
ros2 run lane_perception lane_detector \
  --ros-args \
  -p image_topic:=/prius/front_camera/image_raw \
  -p publish_debug_image:=True
```

The node subscribes to the front-camera image and publishes the normalized lateral lane error. The message layout is:

```text
[steering_error, confidence, left_detected, right_detected]
```

The perception node publishes:

| Topic | Purpose |
|---|---|
| `/lane_error` | Lane error, confidence, and detection flags |
| `/lane_status` | Human-readable status such as `CENTERED`, `DRIFT_LEFT`, `DRIFT_RIGHT`, or `LOST` |
| `/lane_perception/debug_image` | Image with the detected line overlay and status text |

To view the debug image, open `rqt_image_view` in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/lane_perception/debug_image
```

Useful checks are:

```bash
ros2 topic hz /lane_perception/debug_image
timeout 5 ros2 topic hz /lane_error
ros2 topic echo /lane_status --once
```

## Running the Lane Controller

After confirming that perception is publishing valid data, open another terminal and run only one controller instance:

```bash
source /opt/ros/jazzy/setup.bash
source ~/assignment2_ws/install/setup.bash
ros2 run lane_control lane_controller \
  --ros-args \
  -p base_speed:=0.20 \
  -p min_speed:=0.05 \
  -p max_speed:=0.35 \
  -p kp:=1.20 \
  -p kd:=0.10 \
  -p steering_sign:=1.0
```

The controller subscribes to `/lane_error` and publishes `geometry_msgs/msg/Twist` commands on `/cmd_vel`. It stops the vehicle when lane perception is lost. This safety behavior should be preserved during testing.

Check the command output with:

```bash
timeout 3 ros2 topic echo /cmd_vel
```

A moving vehicle should receive a positive `linear.x` value. If all values are zero, check `/lane_status` and the debug image before increasing the speed.

## Lane-Detector Calibration

The lane detector uses color thresholds, a region of interest, and a probabilistic Hough-line detector. The default values may need calibration for the lighting and camera view of the selected world.

The most important parameters are:

```text
roi_top_ratio
white_saturation_max
white_value_min
yellow_h_min
yellow_h_max
yellow_saturation_min
yellow_value_min
min_line_length
min_abs_slope
assumed_lane_width_ratio
```

For a quick runtime experiment, parameters can be changed while the node is running. For example:

```bash
ros2 param set /lane_detector min_line_length 6.0
ros2 param set /lane_detector white_value_min 60
```

For a permanent change, edit `lane_detector.py`, rebuild the package, and source the workspace again:

```bash
cd ~/assignment2_ws
colcon build --symlink-install --packages-select lane_perception
source install/setup.bash
```

Always validate calibration using `/lane_perception/debug_image` before running the vehicle at higher speed.

## Recommended Testing Procedure

The safest testing sequence is:

1. Start Gazebo and confirm that the Prius model is visible in the Entity Tree.
2. Start `ros_gz_bridge` and verify the camera topic.
3. Start `lane_detector` and confirm that the debug image is publishing.
4. Check `/lane_status` and `/lane_error`.
5. Start one `lane_controller` instance at a low speed.
6. Observe the vehicle for only a few seconds.
7. Press `Ctrl+C` in the controller terminal immediately if the car turns in the wrong direction or leaves the track.
8. Tune `steering_sign`, `kp`, `kd`, and speed only after the perception result is stable.

## Recording the Required Evidence

Create a recording that clearly shows:

- Gazebo Harmonic with the Prius and Sonoma Raceway visible.
- The vehicle moving under ROS 2 control.
- The `lane_perception` debug image or terminal output showing that perception is active.
- The ROS 2 terminal showing the bridge and controller nodes running.

A lightweight desktop recording application such as OBS Studio can be used. If OBS Studio is not installed, it can be installed with:

```bash
sudo apt update
sudo apt install -y obs-studio
```

For the ROS 2 command recording, start a bag before the final run:

```bash
mkdir -p ~/assignment2_bags
ros2 bag record -o ~/assignment2_bags/final_run \
  /cmd_vel \
  /lane_error \
  /lane_status \
  /odom \
  /prius/front_camera/image_raw
```

Run the controller while recording, then press `Ctrl+C` in the bag terminal after the run. Check the recorded topics with:

```bash
ros2 bag info ~/assignment2_bags/final_run
```

Do not upload very large bag files to GitHub unless the instructor explicitly requests them. Submit them through the required course platform or provide a separate download link if required.


