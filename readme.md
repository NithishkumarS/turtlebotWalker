## ROS beginner tutorial of Publisher Subscriber

Implementation of Turtlebot walker in gazebo


## Pre-requisite

The project requires ROS kinectic and catkin, and is developed on UBUNTU 16.04 LTS.
```
To install ROS please follow the tutorial on: http://wiki.ros.org/kinetic/Installation

To Install catkin: http://wiki.ros.org/catkin

Gazebo
```

## Build

Before building make sure ROS kinetic and catkin are installed.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/NithishkumarS/turtlebotWalker.git 
cd ..
catkin_make
```

## Running Demo 

To control the bot right and left
'''
roslaunch turtlebot_gazebo turtlebot_world.launch

roslaunch turtlebot_teleop keyboard_teleop.launch
'''
This starts the gazebo simulator and the teleoperation module for it.


To start the behaviour of avoiding obstacles, just do the following commands
```
roscore
```
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker walker.launch
```

To stop the program press ctrl+C in each of the three terminals.
press ctrl+C in each of the terminal to exit the program.


