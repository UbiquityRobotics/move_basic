/*
 * Copyright (c) 2018, Ubiquity Robotics
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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

#include <mutex>


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

// Handle Range messages and computes distance to obstacles
class ObstacleDetector
{
   // Line in polar form
   class PolarLine
   {
   public:
       float radius;
       float theta;
 
       PolarLine(float radius, float theta);
   };

   std::string baseFrame;

   std::map<std::string, RangeSensor> sensors;
   ros::Subscriber sonar_sub;
   ros::Subscriber scan_sub;
   ros::Publisher line_pub;
   tf2_ros::Buffer *tf_buffer;
   int sensor_id;
   // footprint
   float robot_width;
   float robot_front_length;
   float robot_back_length;

   float robot_width_sq;
   float robot_front_length_sq;
   float robot_back_length_sq;
   float front_diag, back_diag;

   float max_age;
   float no_obstacle_dist;
   std::vector<tf2::Vector3> points;
   bool have_test_points;
   std::mutex obstacle_mutex;

   bool have_lidar;
   tf2::Vector3 lidar_origin;
   tf2::Vector3 lidar_normal;
   std::vector<PolarLine> lidar_points;

   void draw_line(const tf2::Vector3 &p1, const tf2::Vector3 &p2,
                  float r, float g, float b, int id);
   void clear_line(int id);

   void get_points();
   void get_lidar_points(std::vector<tf2::Vector3>& points);

   void check_dist(float x, bool forward, float& min_dist) const;
   void check_angle(float theta, float x, float y,
                    bool left, float& min_dist) const;

   float degrees(float radians) const;

public:
   ros::Time stamp;

   ObstacleDetector(ros::NodeHandle& nh, tf2_ros::Buffer *tf_buffer);
   void range_callback(const sensor_msgs::Range::ConstPtr &msg);
   void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

   // return distance in meters to closest obstacle
   float obstacle_dist(bool forward, float &left_dist, float &right_dist,
                       tf2::Vector3 &fl, tf2::Vector3 &fr);

   // return distance in radians to closest obstacle
   float obstacle_angle(bool left);

   // for testing
   void add_test_point(tf2::Vector3 p);

   double min_side_dist;
   double max_side_dist;

   void clear_test_points();
};

