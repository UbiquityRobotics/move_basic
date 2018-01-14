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

/*

 The `ObstacleDetector` class processes range messages to determine the
 distance to obstacles.  As range messages are received, they are used
 to either create or update a `RangeSensor` object.  `RangeSensor` objects
 are created by looking up the transform from `base_link` to their frame
 and computing a pair of vectors corresponding to the sides of their
 cone.

 The distance to the closest object is calculated based upon the positions
 of the end points of the sensors' cones.  For this purpose, the robot
 footprint is paramatized as having width `W` either side of base_link,
 and length `F` forward of base_link and length `B` behind it.  This could
 be extended to be an abitrary polygon.  When the robot is travelling forward
 or backwards, the distance to the closest point that has an `x` value
 between `-W` and `W` is used as the obstacle distance.

 In the case of rotation in place, the angle that the robot will rotate
 before hitting an obstacle is determined. This is done by converting
 each of the `(x,y)` points into `(r, theta)` and determining how much
 `theta` would change by for the point to intersect with one of the four
 line segments representing the robot footprint.

 Coordinate systems are as specified in http://www.ros.org/reps/rep-0103.html
 x forward, y left

 Jim Vaughan <jimv@mrjim.com> January 2018

*/

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/Marker.h>
#include "move_basic/obstacle_detector.h"


ObstacleDetector::ObstacleDetector(ros::NodeHandle& nh,
                                   tf2_ros::Buffer *tf_buffer)
{
    this->tf_buffer = tf_buffer;
    sensor_id = 0;

    line_pub = ros::Publisher(
                 nh.advertise<visualization_msgs::Marker>("/sonar", 1));

    // Footprint
    W = nh.param<float>("footprint_w", 0.04);
    F = nh.param<float>("footprint_f", 0.05);
    B = nh.param<float>("footprint_r", 0.12);

    // To test if obstacles will intersect when rotating
    front_diag = W*W + F*F;
    back_diag = W*W + B*B;

    // TODO: make into a single topic
    std::string topic_prefix = "sonar_";

    for (int i=0; i<16; i++) {
        std::string topic = str(boost::format{"%1%%2%"} % topic_prefix % i);
        ROS_INFO("Subscribing to %s", topic.c_str());
        subscribers.push_back(nh.subscribe("/bus_server/sensor/" + topic, 1,
            &ObstacleDetector::sensor_callback, this));
    }
    //XXX for testing
    //obstacle_angle(true);
    //obstacle_angle(false);
}

void ObstacleDetector::sensor_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    std::string frame = msg->header.frame_id;
    ROS_INFO("Callback %s %f", frame.c_str(), msg->range);

    // ignore min values
    // XXX do we want to do this?
    if (msg->range <= msg->min_range || msg->range >= msg->max_range) {
       return;
    }

    // create sensor object if this is a new sensor
    std::map<std::string,RangeSensor>::iterator it = sensors.find(frame);
    if (it == sensors.end()) {
        try {
            geometry_msgs::TransformStamped tfs =
                tf_buffer->lookupTransform("base_link", frame, ros::Time(0));

            tf2::Transform tf;
            tf2::Vector3 S, A, B, C;

            // sensor origin
            geometry_msgs::PointStamped origin;
            origin.header.frame_id = frame;
            origin.point.x = 0;
            origin.point.y = 0;
            origin.point.z = 0;
            geometry_msgs::PointStamped base_origin;
            tf2::doTransform(origin, base_origin, tfs);
            fromMsg(base_origin.point, S);
            ROS_INFO("origin %f %f %f", S.x(), S.y(), S.z());

            // vectors at the edges of cone when cone height is 1m
            double theta = msg->field_of_view / 2.0;
            float x = std::cos(theta);
            float y = std::sin(theta);

            geometry_msgs::Vector3Stamped left;
            left.vector.x = x;
            left.vector.y = -y;
            left.vector.z = 0.0;
            geometry_msgs::Vector3Stamped base_left;
            tf2::doTransform(left, base_left, tfs);
            fromMsg(base_left.vector, B);

            geometry_msgs::Vector3Stamped right;
            right.vector.x = x;
            right.vector.y = y;
            right.vector.z = 0.0;
            geometry_msgs::Vector3Stamped base_right;
            tf2::doTransform(right, base_right, tfs);
            fromMsg(base_right.vector, C);

            RangeSensor sensor(sensor_id++, frame, S, B, C);
            sensors[frame] = sensor;
            sensor.update(msg->range, msg->header.stamp);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    else {
        RangeSensor& sensor = sensors[frame];
        sensor.update(msg->range, msg->header.stamp);
        draw_line(sensor.origin, sensor.left_vertex, 0, 0, 1, sensor.id + 200);
        draw_line(sensor.origin, sensor.right_vertex, 0, 1, 0, sensor.id + 300);
        draw_line(sensor.left_vertex, sensor.right_vertex, 0.5, 0.5, 0,
            sensor.id + 400);
    }
}

void ObstacleDetector::draw_line(const tf2::Vector3 &p1, const tf2::Vector3 &p2,
                            float r, float g, float b, int id)
{
    visualization_msgs::Marker line;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::MODIFY;
    line.header.frame_id = "/base_link";
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0f;
    line.id = id;
    line.scale.x = line.scale.y = line.scale.z = 0.01;
    line.pose.position.x = 0;
    line.pose.position.y = 0;
    line.pose.orientation.w = 1;
    geometry_msgs::Point gp1, gp2;
    gp1.x = p1.x();
    gp1.y = p1.y();
    gp1.z = p1.z();
    gp2.x = p2.x();
    gp2.y = p2.y();
    gp2.z = p2.z();
    line.points.push_back(gp1);
    line.points.push_back(gp2);

    line_pub.publish(line);
}

void ObstacleDetector::get_points()
{
    points.clear();
    ros::Time now = ros::Time::now();

    std::map<std::string,RangeSensor>::iterator it;
    for (it = sensors.begin(); it != sensors.end(); it++) {
        RangeSensor& sensor = it->second;

        float age = (now - sensor.stamp).toSec();
        if (age < 1.0) {
           points.push_back(sensor.left_vertex);
           points.push_back(sensor.right_vertex);
        }
    }
}

float ObstacleDetector::obstacle_dist_forward()
{
    float min_dist = 10.0f;
    ros::Time now = ros::Time::now();

    get_points();
    for (int i=0; i<points.size(); i++) {
        tf2::Vector3& p = points[i];
        float x = p.x();
        float y = p.y();
        if (x > F && -W < y && y < W) {
            if (x < min_dist) {
                min_dist = x - F;
            }
        }
    }

    ROS_INFO("min_dist %f", min_dist);
    draw_line(tf2::Vector3(min_dist, -W, 0),
              tf2::Vector3(min_dist, W, 0), 1, 0, 0, 1000);
    return min_dist;
}

float ObstacleDetector::obstacle_dist_reverse()
{
    float min_dist = 10.0f;
    ros::Time now = ros::Time::now();

    get_points();
    for (int i=0; i<points.size(); i++) {
        tf2::Vector3& p = points[i];
        float x = -p.x();
        float y = p.y();
        if (x > B && -W < y && y < W) {
            if (x < min_dist) {
                min_dist = x - B;
            }
        }
    }
    ROS_INFO("min_dist %f", min_dist);
    draw_line(tf2::Vector3(min_dist, -W, 0),
              tf2::Vector3(min_dist, W, 0), 1, 0, 0, 1000);
    return min_dist;
}

float ObstacleDetector::degrees(float radians)
{
    return radians * 180.0 / M_PI;
}

/*
 Determine the rotation required to for to move point from its
 initial rotation of theta to (x, y), and store the smallest
 value
*/
inline void ObstacleDetector::check_angle(float theta, float x, float y,
                                          bool left, float& min_dist)
{
    float theta_int = theta - std::atan2(y, x);
    if (theta_int < -M_PI) {
        theta_int += 2.0 * M_PI;
    }
    if (theta_int > M_PI) {
        theta_int -= 2.0 * M_PI;
    }
    if (left && theta_int > 0 && theta_int < min_dist) {
        min_dist = theta_int;
    }
    if (!left && theta_int < 0 && -theta_int < min_dist) {
        min_dist = -theta_int;
    }
    printf("theta_int %f\n", degrees(theta_int));
}

float ObstacleDetector::obstacle_angle(bool left)
{
    float min_angle = M_PI;
    ros::Time now = ros::Time::now();

    get_points();
    //XXX for testing
    //points.push_back(tf2::Vector3(-0.0, 0.1, 0));

    for (int i=0; i<points.size(); i++) {
        tf2::Vector3& p = points[i];
        float x = p.x();
        float y = p.y();
        // initial orientation wrt base_link
        float theta = std::atan2(y, x);
        float r_squared = x*x + y*y;
        if (r_squared < back_diag) {

           // left side: y = W, -B < x < F
           // right side: y = -W, -B < x < F
           float xi = std::sqrt(r_squared - W*W);
           if (-B < xi && xi < F) {
               check_angle(theta, xi, W, left, min_angle);
               check_angle(theta, xi, -W, left, min_angle);
           }
           if (-B < -xi && -xi < F) {
               check_angle(theta, -xi, W, left, min_angle);
               check_angle(theta, -xi, -W, left, min_angle);
           }

           // back side: x = -B, -W < y < W
           float yi = std::sqrt(r_squared - B*B);
           if (-W < yi && yi < W) {
               check_angle(theta, -B, yi, left, min_angle);
           }
           if (-W < -yi && -yi < W) {
               check_angle(theta, -B, -yi, left, min_angle);
           }

           // front side: x = F, -W < y < W
           if (r_squared < front_diag) {
               float yi = std::sqrt(r_squared - B*B);
               if (-W < yi && yi < W) {
                   check_angle(theta, F, yi, left, min_angle);
               }
               if (-W < -yi && -yi < W) {
                   check_angle(theta, F, -yi, left, min_angle);
               }
           }
        }
    }
    printf("min angle %f\n", degrees(min_angle));
    return min_angle;
}

RangeSensor::RangeSensor(int id, std::string frame_id,
                         const tf2::Vector3& origin,
                         const tf2::Vector3& left_vec,
                         const tf2::Vector3& right_vec)
{
    this->id = id;
    this->frame_id = frame_id;
    this->origin = origin;
    this->left_vec = left_vec;
    this->right_vec = right_vec;
    ROS_INFO("Adding sensor %s", frame_id.c_str());
}

void RangeSensor::update(float range, ros::Time stamp)
{
    this->stamp = stamp;
    left_vertex = origin + left_vec * range;
    right_vertex = origin + right_vec * range;
}
