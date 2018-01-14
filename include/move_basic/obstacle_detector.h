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
   std::map<std::string, RangeSensor> sensors;
   std::vector<ros::Subscriber> subscribers;
   ros::Publisher line_pub;
   tf2_ros::Buffer *tf_buffer;
   int sensor_id;
   // footprint
   float W, F, B;
   float front_diag, back_diag;
   std::vector<tf2::Vector3> points;

   void draw_line(const tf2::Vector3 &p1, const tf2::Vector3 &p2,
                  float r, float g, float b, int id);
   void get_points();
   void check_angle(float theta, float x, float y,
                    bool left, float& min_dist);

public:
   ObstacleDetector(ros::NodeHandle& nh, tf2_ros::Buffer *tf_buffer);
   void sensor_callback(const sensor_msgs::Range::ConstPtr &msg);

   float degrees(float radians);

   // return distance in meters to closest obstacle
   float obstacle_dist_forward();
   float obstacle_dist_reverse();

   // return distance in radians to closest obstacle
   float obstacle_angle(bool left);
};

