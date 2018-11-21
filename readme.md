## ROS beginner tutorial of Publisher Subscriber

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Implementation of Turtlebot walker in gazebo


## Pre-requisite

The project requires ROS kinectic and catkin, and is developed on UBUNTU 16.04 LTS.
```
To install ROS please follow the tutorial on: http://wiki.ros.org/kinetic/Installation

To Install catkin: http://wiki.ros.org/catkin

Gazebo
```

| Directory | Description 			    |
| --------- | ------------------------------------- |
| `src`	    | Contains implementation of the nodes  |
| `resuts` | Contains the results and the bag files|
| `launch`  | Holds the xml launch file 	    |
| `include` | COntains the main directories	    |

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

To run the demo open a new terminal and type
```
roscore
```
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker walker.launch
```

To stop the program press ctrl+C in each of the three terminals.

## ROSBAG

To create a new bag file type

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker walker.launch recordBag:=true```

press ctrl+C in each terminal window to exit from the program and stop the recording.

or we could run the following command in the terminal 
```
rosrun rosbag record -O filename.bag topic-names
```

Rosbag file in .bag format is present in the outputs folder. To verify the listener node with bag files first play the bag file using the command
```
cd ~/catkin_ws
source devel/setup.bash
cd src/turtlebot_walker/results
rosbag play walkerOutput.bag
```
press ctrl+C in each of the terminal to exit the program.


