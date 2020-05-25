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

#include "move_basic/obstacle_points.h"
#include <sensor_msgs/Range.h>

ObstaclePoints::ObstaclePoints(ros::NodeHandle& nh) : tf_listener(tf_buffer) {
    sonar_sub = nh.subscribe("/sonars", 1,
        &ObstaclePoints::range_callback, this);
    scan_sub = nh.subscribe("/scan", 1,
        &ObstaclePoints::scan_callback, this);
    
    nh.param<std::string>("base_frame", baseFrame, "base_link");
}

void ObstaclePoints::range_callback(const sensor_msgs::Range::ConstPtr &msg) {
    std::string frame = msg->header.frame_id;
    ROS_DEBUG("Callback %s %f", frame.c_str(), msg->range);

    const std::lock_guard<std::mutex> lock(points_mutex);
    
    // create sensor object if this is a new sensor
    std::map<std::string,RangeSensor>::iterator it = sensors.find(frame);
    if (it == sensors.end()) {
        try {
	    ROS_WARN("lookup %s %s", baseFrame.c_str(), frame.c_str());
            geometry_msgs::TransformStamped sensor_to_base_tf =
                tf_buffer.lookupTransform(baseFrame, frame, ros::Time(0));

            tf2::Transform tf;
            tf2::Vector3 origin, left_vector, right_vector;

            // sensor origin
            geometry_msgs::PointStamped sensor_origin;
            sensor_origin.point.x = 0;
            sensor_origin.point.y = 0;
            sensor_origin.point.z = 0;
            geometry_msgs::PointStamped base_origin;
            tf2::doTransform(sensor_origin, base_origin, sensor_to_base_tf);
            fromMsg(base_origin.point, origin);
            ROS_INFO("Obstacle: origin %f %f %f", origin.x(), origin.y(), origin.z());

            // vectors at the edges of cone when cone height is 1m
            double theta = msg->field_of_view / 2.0;
            float x = std::cos(theta);
            float y = std::sin(theta);

            geometry_msgs::Vector3Stamped sensor_left;
            sensor_left.vector.x = x;
            sensor_left.vector.y = -y;
            sensor_left.vector.z = 0.0;
            geometry_msgs::Vector3Stamped base_left;
            tf2::doTransform(sensor_left, base_left, sensor_to_base_tf);
            fromMsg(base_left.vector, left_vector);

            geometry_msgs::Vector3Stamped sensor_right;
            sensor_right.vector.x = x;
            sensor_right.vector.y = y;
            sensor_right.vector.z = 0.0;
            geometry_msgs::Vector3Stamped base_right;
            tf2::doTransform(sensor_right, base_right, sensor_to_base_tf);
            fromMsg(base_right.vector, right_vector);

            RangeSensor sensor(sensors.size(), frame, origin,
                               left_vector, right_vector);
            sensors[frame] = sensor;
            sensor.update(msg->range, msg->header.stamp);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    else {
        RangeSensor& sensor = it->second;
        sensor.update(msg->range, msg->header.stamp);
    }
}

void ObstaclePoints::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    float theta = msg->angle_min;
    float increment = msg->angle_increment;
    float range_min = msg->range_min;
    
    const std::lock_guard<std::mutex> lock(points_mutex);
    lidar_stamp = msg->header.stamp;

    if (!have_lidar) {
        try {
            geometry_msgs::TransformStamped laser_to_base_tf =
                tf_buffer.lookupTransform(baseFrame, msg->header.frame_id, ros::Time(0));

            tf2::Transform tf;

            // lidar origin
            geometry_msgs::PointStamped origin;
            origin.point.x = 0;
            origin.point.y = 0;
            origin.point.z = 0;
            geometry_msgs::PointStamped base_origin;
            tf2::doTransform(origin, base_origin, laser_to_base_tf);
            fromMsg(base_origin.point, lidar_origin);

            // normal vector
            geometry_msgs::Vector3Stamped normal;
            normal.vector.x = 1.0;
            normal.vector.y = 0.0;
            normal.vector.z = 0.0;
            geometry_msgs::Vector3Stamped base_normal;
            tf2::doTransform(normal, base_normal, laser_to_base_tf);
            fromMsg(base_normal.vector, lidar_normal);

            have_lidar = true;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

    lidar_points.clear();
    for (const auto& r : msg->ranges) {
        theta += increment;

        // ignore bogus samples
        if (std::isnan(r) || r < range_min) {
            continue;
        }

        lidar_points.push_back(PolarLine(r, theta));
    }
}
  
std::vector<tf2::Vector3> ObstaclePoints::get_points(ros::Duration max_age) {
    ros::Time now = ros::Time::now();
    std::vector<tf2::Vector3> points;

    const std::lock_guard<std::mutex> lock(points_mutex);
    ros::Duration lidar_age = now - lidar_stamp;
    if (lidar_age < max_age) {
	for (const auto& p : lidar_points) {
	    float sin_theta = std::sin(p.theta);
	    float cos_theta = std::cos(p.theta);

	    float x = lidar_origin.x() + p.radius * (lidar_normal.x() * cos_theta -
		 lidar_normal.y() * sin_theta);

	    float y = lidar_origin.y() + p.radius * (lidar_normal.y() * cos_theta +
		 lidar_normal.x() * sin_theta);

	    points.push_back(tf2::Vector3(x, y, 0));
	}
    }

    for (const auto& kv : sensors) {
        const RangeSensor& sensor = kv.second;

	ros::Duration age = now - sensor.stamp;
        if (age < max_age) {
           points.push_back(sensor.left_vertex);
           points.push_back(sensor.right_vertex);
        }
    }

    // Add all the test points
    points.insert(points.end(), test_points.begin(), test_points.end());

    return points;
}

std::vector<ObstaclePoints::Line> ObstaclePoints::get_lines(ros::Duration max_age) {
    ros::Time now = ros::Time::now();
    
    const std::lock_guard<std::mutex> lock(points_mutex);
    std::vector<ObstaclePoints::Line> lines;
    for (const auto& kv : sensors) {
        const RangeSensor& sensor = kv.second;
	ros::Duration age = now - sensor.stamp;
	if (age < max_age) {
	    lines.emplace_back(sensor.left_vertex, sensor.right_vertex);
	}
    }

    return lines;
}

void ObstaclePoints::add_test_point(tf2::Vector3 p) {
    const std::lock_guard<std::mutex> lock(points_mutex);
    test_points.push_back(p);
}

void ObstaclePoints::clear_test_points() {
    const std::lock_guard<std::mutex> lock(points_mutex);
    test_points.clear();
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

ObstaclePoints::PolarLine::PolarLine(float radius, float theta)
{
    this->radius = radius;
    this->theta = theta;
}

