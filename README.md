
# move_basic
Travis:  [![Build Status](https://travis-ci.org/UbiquityRobotics/move_basic.svg?branch=kinetic-devel)](https://travis-ci.org/UbiquityRobotics/move_basic)

Jenkins: [![Build Status](http://build.ros.org/view/Kdev/job/Kdev__move_basic__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/view/Kdev/job/Kdev__move_basic__ubuntu_xenial_amd64/)


This package contains a node that performs very basic navigation.
The path planning consists of rotating in place to face the goal and then
driving straight toward it.  It is designed to provide the same interfaces as 
[move_base](http://wiki.ros.org/move_base).

## Installing software

First you must install ROS (Robot Operating System),
see [install ROS](http://wiki.ros.org/ROS/Installation) for more details.

The package can be installed:

     $ sudo apt-get install ros-kinetic-move-basic
     
## To run

To run, give the following command:

     $ rosrun move_basic move_basic

## Node details

Please refer to [move_basic](http://wiki.ros.org/move_basic) for node documentation.
