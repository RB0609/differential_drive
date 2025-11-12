# Indoor Navigation of a Differential-Dirve robot using 3D Lidar and Depth camera
<img width="1858" height="1050" alt="image" src="https://github.com/user-attachments/assets/46f1be5a-044b-4f9b-b3ac-7183911c2a52" />

## Watch this below video for Navigation


https://github.com/user-attachments/assets/1b53e178-9909-4a6d-b03d-a3bee1ce38e4



## Features
- Robot simulation in a indoor environment in Gazebo Harmonic
- Navigation (using Nav2 Stack) from Point A to Point B, using "Nav2goal" feature from Nav2

## Requirements
- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Sim Harmonic
- Nav2 Stack
## Installation
1. Git clone the repo for SImulation in Gazebo<br>
NOTE: Replace ros2_ws/with your own folder
```bash
cd ~/ros2_ws/src
git clone https://github.com/RB0609/differential_drive.git
```
2. Build the Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
3. Launch file in Gazebo and Rviz
```bash
ros2 launch dd_robot display.launch.py
```
## Steps to Perform Navigation
In nav2_params.yaml file which is in config folder
```bash
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: "<path_to_file location>/map.yaml" #Here choose your own folder, where you store "map.yaml" file
    #topic_name: "map"
    #frame_id: "map"
```
in Terminal1:(First launch this file)
```bash
ros2 launch nav2_bringup rviz_launch.py 
```
For below code, careful with file location<br>
map:=<path_to_file_location>/map.yaml<br>
params_file:=<path_to_file_location>/nav2_params.yaml<br>
below is the example usage<br>
in Terminal2:
```bash
ros2 launch nav2_bringup bringup_launch.py   use_sim_time:=true   map:=/home/rakesh/map.yaml   params_file:=/home/rakesh/ros2_ws/src/differential_drive/dd_robot/config/nav2_params.yaml
```
## Further work
1. Include Turtlebot3 waffle model
2. Working on Sim2Real
```
