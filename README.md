
# move_basic

This package contains a node that performs very basic navigation.
The path planning consists of rotating in place to face the goal and then
driving straight toward it.  It is designed to provide the same interfaces as 
[move_base](http://wiki.ros.org/move_base). If lidar or sonar sensors are 
present, it will attemp to stop to avoid a collision with obstacles.

## Installing software

First you must install ROS (Robot Operating System),
see [install ROS](http://wiki.ros.org/ROS/Installation) for more details.

The package can be installed:

     $ sudo apt-get install ros-kinetic-move-basic
     
## To run

To run, give the following command:

     $ rosrun move_basic move_basic

## Node details

Please refer to [the move_basic wiki page](http://wiki.ros.org/move_basic) for node documentation.
