# cassie_description
This repository contains the .urdf model of the CASSIE robot from Agility Robotics. 
It also includes a a way to visualize the robot using ROS and rviz. 

* modification by Erwin Coumans: added collision elements and simplied the mesh 0.1 (90% fewer vertices) *

Installation to view .urdf using rviz
=====================================

- Download and install ROS by following the instructions at http://wiki.ros.org/indigo/Installation/Ubuntu.

- Create a folder for the catkin workspace 
```
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
cd src
catkin_init_workspace
```
- Clone the repository to get the cassie_description package
```
git clone https://github.com/UMich-BipedLab/cassie_description.git
```
- Build the package
```
cd ../
catkin_make
source devel/setup.bash
```
- Launch rviz to visualize the .urdf file
```
roslaunch cassie_description display.launch 
```

