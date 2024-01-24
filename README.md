# line_follower_omnidrive
A ROS based repository for a line follower robot

# Introduction
This ROS package, line_follower_omnidrive, is developed for controlling a line-following robot with omnidirectional drive capabilities. The robot utilizes computer vision techniques provided by OpenCV to navigate along a predefined path using a simple P-controller(proportional).

# Installation

## Pre-Requisites:
- ROS noetic : Install ROS Noetic by following the [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Catkin workspace : Create a catkin_ws by following the [official documentation](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- OpenCV
  ```bash
  pip install opencv-contrib-python
  ```
- Gazebo Classic : Refer to the official [Gazebo installation guide](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b1)

## Clone the line_follower_omnidrive package
Clone the following repository in the "src" folder of your catkin workspace
```bash
git clone git@github.com:atom-robotics-lab/line_follower_omnidrive.git
```
## Build the catkin workspace
```bash
cd ~/catkin_ws
catkin_make
```
## Launch
Launch the Gazebo simulation and start the ROS master
```bash
roslaunch line_follower_omnidrive line.launch
```
## Run the CV script(to get camera feed)
```bash
rosrun line_follower_omnidrive feedback.py
```
## Run the controller script
```bash
rosrun line_follower_omnidrive controller.py
```
The robot will follow the line based on the computer vision processing.
