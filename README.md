
# move_basic

This package contains a node that performs very basic navigation.
The path planning consists of rotating in place to face the goal and then
driving straight toward it.  It is designed to provide the same interfaces as 
[move_base](http://wiki.ros.org/move_base).

## Installing software

First you must install ROS (Robot Operating System),
see [install ROS](http://wiki.ros.org/ROS/Installation) for more details.

Currently you must install from source--binaries are not yet available.
First, create a ROS catkin workspace, if you don't already have one:

     $ mkdir -p ~/catkin_ws/src
     $ cd ~/catkin_ws/src
     $ catkin_init_workspace

Then get the source:

     $ git clone https://github.com/UbiquityRobotics/move_basic 
     $ cd ..
     $ catkin_make    

## To run

To run, give the following command:

     $ rosrun move_basic move_basic

## Node details

Please refer to [move_basic](http://wiki.ros.org/move_basic) for node documentation.
