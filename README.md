
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

New parameters:

`~smooth_follow`: (bool) Toggle preserving velocity when rotating towards a new goal for smooth cornering. May be a bit less accurate, but a lot easier on the wheels.  Default False.

`~forward_obstacle_threshold`: (float) Velocity multiplier for stopping when an obstacle is detected. Higher values should result in earlier braking. Default 1.5.


## New obstacle avoidance behavoir (March 2019)

New behavior was added in the `side_dist` branch.  The behavior is to
follow a wall or similar object at the side.

New subscription:

`/follow_mode`: (int) Follow mode: 0 None, 1 Left, 2 Right.


New parameters:

`~min_side_dist`: (float) Target distance to maintain in side following mode
meters.  Default 0.2.

`~max_side_dist`: (float) Maximum distance in meters at side before determining there is nothing there.  Default 1.0.  

`max_follow_dist_without_wall`: (float) Maximum distance to travel with no side object in following mode before aborting.  Default 0.5.

`~max_lateral_deviation`: (float) Maximum deviation from a straight path
before aborting the current goal. Default 4.0.

`~side_turn_weight`: (float) Weighting of side turn control to avoid obstacles. Default 3.0.

`~side_recover_weight`: (float) Weighting of side turn control to keep on plan. Default 3.0.

`~lateral_kp`: (float) Lateral control P term. Default 0.1.

`~lateral_ki`: (float) Lateral control I term. Default 0.

`~lateral_kd`: (float) Lateral control D term. Default 50.

