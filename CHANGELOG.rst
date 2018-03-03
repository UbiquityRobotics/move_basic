^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package move_basic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2018-03-03)
------------------
* Don't rotate to go small distances
* Reduce verbosity when rotating
* Contributors: Jim Vaughan

0.3.1 (2018-02-04)
------------------
* Publish forward obstacle distance (`#31 <https://github.com/UbiquityRobotics/move_basic/issues/31>`_)
* Improve Lidar-based obstacle detection (`#30 <https://github.com/UbiquityRobotics/move_basic/issues/30>`_)
* Contributors: Jim Vaughan

0.3.0 (2018-01-21)
------------------
  * Add ObstacleDetector class to estimate distance to obstacles using range sensors during linear motion and rotation in-place.
* Contributors: Jim Vaughan, Rohan Agrawal

0.2.2 (2017-10-08)
------------------
* Will reverse for small distances without rotating
* Try to operate in the frame the goal is in if no map frame
* Add map_frame param
* Add frane to points in path
* Contributors: Jim Vaughan, Rohan Agrawal

0.2.1 (2017-07-17)
------------------
* Add map_frame param
* Contributors: Jim Vaughan

0.2.0 (2017-05-21)
------------------
* Add move_base_msgs dep
* Contributors: Jim Vaughan

0.1.1 (2017-05-15)
------------------
* Add URL to wiki page to README.md (`#15 <https://github.com/UbiquityRobotics/move_basic/issues/15>`_)
* added all deps in the package xml (`#14 <https://github.com/UbiquityRobotics/move_basic/issues/14>`_)
* Contributors: Jim Vaughan, Rohan Agrawal

0.1.0 (2017-05-10)
------------------
* Initial Release of a package for very basic navigation. The path planning consists of rotating in place to face the goal and then driving straight toward it. It is designed to provide the same interfaces as move_base.
* Contributors: Jim Vaughan, Rohan Agrawal
