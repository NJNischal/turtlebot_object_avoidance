# turtlebot_object_avoidance
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
The ROS package gives the turtlebot robot the ability to move while avoiding obstacles in it's path.The simulation is done in gazebo with a custom made fully enclosed world.

## Dependencies

The following are the dependencies to run the package:

* ROS distro :Kinetic
* Ubuntu 16.04
* Gazebo
* turtlebot_gazebo

## Installing turtlebot_gazebo

Run the below command to install the package for turtlebot_gazebo

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

## Steps for building package

* Install catkin
```
sudo apt-get install ros-kinetic-catkin
```
* Setup Catkin Workspace
```
mkdir path_to_catkin_workspace
cd path_to_catkin_workspace
mkdir src
cd src
```
* Clone the package repository
```
cd path_to_catkin_workspace/src
git clone --recursive https://github.com/NJNischal/turtlebot_object_avoidance.git
```
* Build package and install using catkin
```
cd path_to_catkin_workspace
catkin_make
source ./devel/setup.bash
```
* Running the package using launch file
```
roslaunch turtlebot_object_avoidance demo.launch 
```
* Running the package using launch file with recording in rosbag off
```
roslaunch turtlebot_object_avoidance demo.launch bagrecord:=0
```
## Playing the rosbag

Run the following command in the rosbag folder in the repo:
```
rosbag play rosbag_2019-11-18-14-10-53.bag
```

The publishing topic can be viewed using the following command in a new terminal:
```
rostopic echo /mobile_base/commands/velocity
```

## Custom world for the gazebo simulation:

</p>
<p align="center">
<img src="/images/Gazebo.png">
</p>
</p>

## Performing checks on the package

For cppcheck, run the following command in the terminal inside the package
```
cppcheck --enable=all --std=c++11 -I ../.. -I ../../../../../../../opt/ros/kinetic/include/ -I ../../../../../../../usr/include/ --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )

```

For cpplint, run the following command in the terminal inside the package
```
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )
```
