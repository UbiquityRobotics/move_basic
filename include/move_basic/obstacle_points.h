/*
 * Copyright (c) 2020, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#ifndef OBSTACLE_POINTS_H
#define OBSTACLE_POINTS_H

#include <vector>
#include <utility>
#include <mutex>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

// lidar sensor
class LidarSensor
{
public:
    std::string frame_id;
    double angle_increment;
    double min_range;
    double max_range;
    double min_angle;
    double max_angle;
    LidarSensor() {};
    void reset(const std::string & _frame,
               const int _increment,
               const double & _min_range,
               const double & _max_range,
               const double & _min_angle,
               const double & _max_angle);
};

// a single sensor with current obstacles
class RangeSensor
{
    tf2::Vector3 left_vec;
    tf2::Vector3 right_vec;

public:
    int id;
    std::string frame_id;
    tf2::Vector3 origin;
    // cone vertices from last Range message
    tf2::Vector3 left_vertex;
    tf2::Vector3 right_vertex;
    ros::Time stamp;

    RangeSensor() {};
    RangeSensor(int id, std::string frame_id,
                const tf2::Vector3& origin,
                const tf2::Vector3& left_vec,
                const tf2::Vector3& right_vec);

    void update(float range, ros::Time stamp);
};

class ObstaclePoints
{
private:
  // Line in polar form
  class PolarLine
  {
  public:
     float radius;
     float theta;

     PolarLine(float radius, float theta);
  };

  std::mutex points_mutex;

  std::string baseFrame;

  LidarSensor lidar;
  std::map<std::string, RangeSensor> sensors;
  ros::Subscriber sonar_sub;
  ros::Subscriber scan_sub;
  tf2_ros::Buffer& tf_buffer;

  bool have_lidar;
  tf2::Vector3 lidar_origin;
  tf2::Vector3 lidar_normal;
  std::vector<PolarLine> lidar_points;
  ros::Time lidar_stamp;

  // Sine / Cosine LookUp Tables
  std::vector<float> sinLUT, cosLUT;

  // Manually added points, used for unit testing things that
  // use ObstaclePoints without having to go through ROS messages
  std::vector<tf2::Vector3> test_points;

public:
  ObstaclePoints(ros::NodeHandle& nh, tf2_ros::Buffer& tf_buffer);

  void range_callback(const sensor_msgs::Range::ConstPtr &msg);
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

  /*
   * Returns a vector of all the points that were detected, filtered
   * by the maximum age.
   *
   */
  std::vector<tf2::Vector3> get_points(ros::Duration max_age);

  /*
   * Returns a vector of lines (expressed as a pair of 2 points).
   * The lines are based on the end of the sonar cones, filtered
   * by the the specified maximum age.
   *
   */
  typedef std::pair<tf2::Vector3, tf2::Vector3> Line;
  std::vector<Line> get_lines(ros::Duration max_age);

  // Used for unit testing things that use ObstaclePoints
  // without having to go through ROS messages
  void add_test_point(tf2::Vector3 p);
  void clear_test_points();

};

#endif
