# Assignment 2: Gazebo–ROS 2 Bridging and Lane-Following Race

## my name : Sabah Muhammed  Id: 1404-5-35
## the video link: [https://youtu.be/r3j2uwD6HtU ]
## A bag file link:ROS 2 Bag File (cmd_vel): [  https://drive.google.com/drive/folders/1v3GwPYGYMTnmJZN3MURjKriX9k0N_Xwi?usp=sharing ]
## GitHub link:https://github.com/Muna2026/assignment2-gazebo-ros2-lane-following/edit/main/README.md
## ⏱️ Lap Time Achieved: [  00:15.27]

## Documentation:

1. Project Overview

This project implements an autonomous lane-following system for the Prius vehicle in the Sonoma Raceway world using Gazebo Harmonic and ROS 2 Jazzy. The system is divided into two ROS 2 packages written for this assignment:

•
lane_perception: processes the front-camera image and estimates the vehicle's lateral position relative to the lane.

•
lane_control: converts the lane estimate into velocity commands and publishes them to /cmd_vel.

Gazebo communicates with ROS 2 through ros_gz_bridge. The bridge forwards the Prius camera images from Gazebo to ROS 2 and sends the controller's velocity commands from ROS 2 back to Gazebo.

2. Overall System Architecture

The data flow is:

Plain Text


Gazebo front camera
        |
        |  /front_camera
        v
ros_gz_bridge
        |
        |  /prius/front_camera/image_raw
        v
lane_perception
        |
        |  /lane_error and /lane_status
        v
lane_control
        |
        |  /cmd_vel
        v
ros_gz_bridge
        |
        v
Prius vehicle in Gazebo



The perception node publishes a four-element lane message on /lane_error:

Plain Text:
[steering_error, confidence, left_detected, right_detected]



The steering error is normalized around the image center. A value close to zero means that the vehicle is approximately centered in the lane. The confidence value indicates how reliable the detected lane boundaries are, while the last two values indicate whether the left and right lane boundaries were detected.

3. Lane Perception Package

The lane_perception package subscribes to the bridged camera topic:

Plain Text:
/prius/front_camera/image_raw



The node first converts each image to HSV color space. It then creates masks for the white and yellow lane markings. The image is restricted to a region of interest near the bottom of the camera view because this area contains the road boundaries that are most relevant to immediate steering.

After color filtering, the node applies edge and line detection using the probabilistic Hough transform. Detected line segments are classified according to their position and slope into left and right lane-boundary candidates. The selected boundaries are used to calculate the lane center and its offset from the image center. This offset is normalized and published as the steering error.

The node also publishes:

Plain Text:
/lane_status



Possible status values include CENTERED, DRIFT_LEFT, DRIFT_RIGHT, and LOST. A debug image is published on:

Plain Text:
/lane_perception/debug_image


The debug image overlays the detected lane boundaries, the image center, the estimated lane center, the steering status, the error, and the confidence. This makes it possible to verify the perception result visually in rqt_image_view.

The parameters used during calibration were:

Plain Text:
white_value_min=30
white_saturation_max=180
yellow_h_min=0
yellow_h_max=60
yellow_saturation_min=5
yellow_value_min=10
min_line_length=4.0
min_abs_slope=0.03
roi_top_ratio=0.65



These values allowed the node to detect both lane boundaries with a confidence of approximately 1.00 in the Sonoma Raceway scene.

4. Lane Control Package

The lane_control package subscribes to /lane_error. It uses the normalized steering error to calculate the angular velocity command. The control law is a proportional-derivative controller:

Plain Text:
angular_z = steering_sign * (kp * error + kd * error_rate)



The angular velocity is limited to max_angular_z to avoid excessively sharp steering commands. The controller also adjusts the forward speed according to the configured minimum, base, and maximum speeds. When the perception confidence is too low or the lane is lost for longer than the configured timeout, the controller applies a safe stopping behavior instead of continuing blindly.

The controller publishes a geometry_msgs/msg/Twist message on:

Plain Text:
/cmd_vel



The message is bridged to Gazebo as gz.msgs.Twist and controls the Prius vehicle. During testing, a low speed was selected to keep the vehicle stable in the narrow and visually challenging track section. The steering direction was calibrated using:

Plain Text:
steering_sign=-1.0



The controller parameters can be adjusted without changing the source code. For example:

Plain Text:
base_speed
min_speed
max_speed
kp
kd
max_angular_z
steering_sign
min_confidence
lost_timeout_sec



5. Building the Workspace

The workspace was built with ROS 2 Jazzy using the following commands:

Bash:
source /opt/ros/jazzy/setup.bash
cd ~/assignment2_ws
colcon build --symlink-install --packages-select lane_perception lane_control
source install/setup.bash



Only the two packages written for this assignment are included in the repository. The generated build/, install/, and log/ directories are excluded using .gitignore.

6. Running the Project

First, launch the Sonoma Raceway world with the Prius:

Bash:
source /opt/ros/jazzy/setup.bash
gz sim -r "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/prius on sonoma raceway/1/sonoma.sdf"



In a separate terminal, launch the ROS–Gazebo bridge. The absolute path is used because the bridge configuration parameter does not expand ~ reliably:

Bash:
source /opt/ros/jazzy/setup.bash
source ~/assignment2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:=$(realpath ~/assignment2_ws/config/gz_sim_bridge_car.yaml)



In another terminal, start the lane-perception node with the calibrated parameters:

Bash:
source /opt/ros/jazzy/setup.bash
source ~/assignment2_ws/install/setup.bash
ros2 run lane_perception lane_detector \
  --ros-args \
  -p image_topic:=/prius/front_camera/image_raw \
  -p publish_debug_image:=True \
  -p white_value_min:=30 \
  -p white_saturation_max:=180 \
  -p yellow_h_min:=0 \
  -p yellow_h_max:=60 \
  -p yellow_saturation_min:=5 \
  -p yellow_value_min:=10 \
  -p min_line_length:=4.0 \
  -p min_abs_slope:=0.03 \
  -p roi_top_ratio:=0.65



The debug image can be viewed with:

Bash


rqt_image_view



Select /lane_perception/debug_image from the topic list.

Finally, start the lane controller:

Bash


source /opt/ros/jazzy/setup.bash
source ~/assignment2_ws/install/setup.bash
ros2 run lane_control lane_controller \
  --ros-args \
  -p base_speed:=0.12 \
  -p min_speed:=0.12 \
  -p max_speed:=0.20 \
  -p kp:=0.12 \
  -p kd:=0.0 \
  -p max_angular_z:=0.15 \
  -p steering_sign:=-1.0 \
  -p min_confidence:=0.1 \
  -p lost_timeout_sec:=3.0



The final speed values should be adjusted carefully according to the stability of the vehicle and the quality of the camera detections.

7. Verification Commands

The following commands can be used to verify that the main topics are active:

Bash


ros2 topic list | grep -E 'cmd_vel|lane|camera'
ros2 topic echo /lane_error --once
ros2 topic echo /lane_status --once
ros2 topic info /cmd_vel -v



A successful perception result should contain a non-zero confidence and detections for both lane boundaries, for example:

Plain Text


[steering_error, 1.0, 1.0, 1.0]



8. Recording the ROS 2 Bag

The velocity-command bag was recorded from the same run shown in the submitted video. The recording command was:

Bash


source /opt/ros/jazzy/setup.bash
mkdir -p ~/assignment2_ws/recordings
cd ~/assignment2_ws/recordings
ros2 bag record -o lap1_cmd_vel /cmd_vel



The recording was stopped with Ctrl+C after the driving attempt ended. The bag contains the /cmd_vel topic and includes metadata.yaml together with the MCAP data file.

9. Limitations and Future Improvements

The current lane detector is based on color thresholds and Hough line segments, so its performance depends on illumination, camera exposure, road texture, and the visibility of the yellow and white markings. The controller was intentionally operated at a conservative speed to reduce oscillation. Future improvements could include temporal filtering of the lane error, a moving average for the detected line positions, adaptive color thresholds, and a more advanced vehicle steering model.

