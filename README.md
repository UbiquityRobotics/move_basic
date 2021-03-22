
# move_basic

## Overview

This package contains a node that performs very basic navigation.
The path planning consists of rotating in place to face the goal and then
driving straight toward it.  It is designed to provide the same interfaces as
[move_base](http://wiki.ros.org/move_base). If lidar or sonar sensors are
present, it will attemp to stop to avoid a collision with obstacles.

## License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Jim Vaughan <br/>**
**Maintainer: Teodor Janez Podobnik, tp@ubiquityrobotics.org <br/>**
**Affiliation: [Ubiquity Robotics](www.ubiquityrobotics.org)**

The move_basic package has been tested under ROS Kinetic and Melodic on respectively Ubuntu 16.04 and 18.04. This is a research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

First you must install ROS (Robot Operating System),
see [install ROS](http://wiki.ros.org/ROS/Installation) for more details.

### Instalation from Packages

The package can be installed:

    sudo apt-get install ros-kinetic-move-basic

### Building from Source

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_workspace/src
    git clone https://github.com/UbiquityRobotics/move_basic.git
    cd ../
    rosdep install --from-paths . --ignore-src
    catkin build
    source devel/setup.bash

## Usage

To run, give the following command:

    rosrun move_basic move_basic

## Nodes

### move_basic

#### Subscribed Topics

* **`/move_base_simple/goal`** ([geometry_msgs/PoseStamped])
* **`/move_base/stop`** ([std_msgs/Bool])

#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])
* **`/plan`** ([nav_msgs/Path])
* **`/obstacle_distance`** ([geometry_msgs/Vector3])
* **`/lateral_error`** ([geometry_msgs/Vector3])
* **`/move_base/goal`** ([move_base_msgs/MoveBaseActionGoal])

#### Parameters

* **`min_turning_velocity`** (double, default: 0.18, min: 0, max: 1.0)

        Minimum turning velocity on spot.

* **`max_turning_velocity`** (double, default: 1.0, min: 0, max: 4.0)

	Maximum turning velocity on spot.

* **`max_lateral_velocity`** (double, default: 0.5, min: 0, max: 4.0)

	Maximum lateral velocity moving linear.

* **`max_linear_velocity`** (double, default: 0.5, min: 0, max: 1.1)

	Maximum linear velocity.

* **`min_linear_velocity`** (double, default: 0.1, min: 0, max: 1.1)

	Minimum linear velocity.

* **`linear_acceleration`** (double, default: 0.1, min: 0, max: 1.1)

	Maximum linear acceleration.

* **`turning_acceleration`** (double, default: 0.2, min: 0, max: 4.0)

	Maximum angular acceleration.

* **`angular_tolerance`** (double, default: 0.1, min: 0, max: 0.5)

	Within angular tolerance, orientation error is negligible.

* **`linear_tolerance`** (double, default: 0.1, min: 0, max: 1.0)

	Within linear tolerance, linear error is negligible.

* **`lateral_kp`** (double, default: 0, min: 0, max: 100)

        Proportional coefficient for lateral error correction.

* **`lateral_ki`** (double, default: 0, min: 0, max: 100)

        Integral coefficient for lateral error correction.

* **`lateral_kd`** (double, default: 3.0, min: 0, max: 100)

        Differential coefficient for lateral error correction.

* **`runaway_timeout`** (double, default: 1.0, min: 0, max: 10.0)

	Lateral velocity multiplier.

* **`obstacle_wait_threshold`** (double, default: 60.0, min: 0, max: 200.0)

	Maximum duration to wait for obstacle to remove.

* **`forward_obstacle_threshold`** (double, default: 0.5, min: 0, max: 3.0)

	Minimum distance to maintain in front.

* **`min_side_dist`** (double, default: 0.3, min: 0, max: 5.0)

	Minimum distance to maintain at each side.

* **`preferred_planning_frame`** (string, default: None)

	Preferred frame to plan a path in.

* **`alternate_planning_frame`** (string, default: odom)

	Alternative frame to plan a path in.

* **`preferred_driving_frame`** (string, default: map)

	Preferred frame to drive in.

* **`alternate_driving_frame`** (string, default: odom)

	Preferred frame to drive in.

* **`base_frame`** (string, default: base_footprint)

	Base frame of the robot.

For more details refer to [the move_basic wiki page](http://wiki.ros.org/move_basic).

## follow mode (wall following) was removed, the last version to have it was 0.3.2
## backward drive was removed, the last version to have it was 0.4.0
