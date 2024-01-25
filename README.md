# Line_follower_omnidrive

This is line follower robot based on OpenCV(computer vision).
# Introduction
In this project, we find the error between the centroid of the line and the centroid of the robot, and we rotate the robot untill the error is equal to 0.The data is taken from the feedback.py script and using the values provided by feedback.py we provide velocity to cmd_vel node via the controller.py script.

<img src = "https://github.com/atom-robotics-lab/line_follower/blob/main/Assets/work_flow.png" >

#

# Installation

## Pre-Requisites:
- ROS noetic : Refer to the [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) for installation of ROS noetic.
               
- Catkin workspace : A catkin workspace is a folder where you modify, build, and. install catkin packages. Take a look at the [official documentation](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for instructions regarding creation of a catkin workspace


## Installation of Virtualenvwrapper, OpenCV, and CV_bridge

Your can refer to [A.T.O.M's wiki](https://atom-robotics-lab.github.io/wiki/setup/virtualenv.html) for installation of the above mentioned packages and libraries.


## Clone the Line Follower package
Now go ahead and clone this repository inside the "src" folder of the catkin workspace you just created by executing the command given below in your terminal.
```bash
git clone git@github.com:atom-robotics-lab/line_follower_omnidrive.git
```

### Note:

Now out robot does not have camera in this package so we have to chang branch from main to with_camera.   
Now then go inside MR-Robot package you just created by executing the above command then executing the command given below in your terminal.
```bash
git checkout with_camera 
```
__All installation is done__

# Make the package
We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package.  (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
cd ~/catkin_ws
catkin_make
```

# Launch

__launching world, use line.launch for line__

```bash
roslaunch line_follower_omnidrive line.launch
```
__for curve__
```bash
roslaunch line_follower_omnidrive curve.launch
```
__for square__
```bash
roslaunch line_follower_omnidrive square.launch
```
The above command when executed in the terminal will launch the gazebo simulation and will also start ROS Master.

<img src = "https://github.com/atom-robotics-lab/line_follower_omnidrive/blob/ros/assets/first.png" >


__Run script__

```bash
rosrun line_follower_omnidrive feedback.py
```
```bash
rosrun line_follower_omnidrive controller.py
```
<img src = "https://github.com/atom-robotics-lab/line_follower_omnidrive/blob/ros/assets/feedback.png">
<img src = "https://github.com/atom-robotics-lab/line_follower_omnidrive/blob/ros/assets/controller.png">

The given command will run the feedback and the controller script which controls the robot's movements based on the data provided by the feedback script.

<img src = "https://github.com/atom-robotics-lab/line_follower_omnidrive/blob/ros/assets/random.gif" >




