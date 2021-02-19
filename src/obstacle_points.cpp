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

ObstaclePoints::ObstaclePoints(ros::NodeHandle& nh, tf2_ros::Buffer& tf_buffer) : tf_buffer(tf_buffer) {
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
	    ROS_INFO("lookup %s %s", baseFrame.c_str(), frame.c_str());
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
            sensor.update(msg->range, msg->header.stamp);
            sensors[frame] = sensor;
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
    size_t array_size = (msg->ranges).size();

    const std::lock_guard<std::mutex> lock(points_mutex);
    lidar_stamp = msg->header.stamp;

    if (!have_lidar) {
        try {
            std::string laserFrame = msg->header.frame_id;
            geometry_msgs::TransformStamped laser_to_base_tf =
                tf_buffer.lookupTransform(baseFrame, laserFrame, ros::Time(0));

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

            double min_range = msg->range_min;
            double max_range = msg->range_max;
            double min_angle = msg->angle_min;
            double max_angle = msg->angle_max;
            double angle_increment = msg->angle_increment;
            lidar.reset(laserFrame, angle_increment, min_range, max_range, min_angle, max_angle);
            have_lidar = true;

            // Sine / Cosine Look Up Tables
            cosLUT.reserve(array_size);
            sinLUT.reserve(array_size);
            double angle = msg->angle_min;
            for (unsigned int i = 0 ; i < array_size ; i++) {
                double cosine = std::cos(angle);
                double sine = std::sin(angle);
                cosLUT.push_back(std::move(cosine));
                sinLUT.push_back(std::move(sine));
                angle += increment;
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

    lidar_points.clear();
    lidar_points.reserve(array_size);
    for (const auto& r : msg->ranges) {
        lidar_points.push_back(PolarLine(r, theta));
        theta += increment;
    }
}

std::vector<tf2::Vector3> ObstaclePoints::get_points(ros::Duration max_age) {
    ros::Time now = ros::Time::now();
    std::vector<tf2::Vector3> points;

    const std::lock_guard<std::mutex> lock(points_mutex);
    points.reserve(lidar_points.size());
    ros::Duration lidar_age = now - lidar_stamp;
    if (lidar_age < max_age) {
	for (unsigned int i = 0 ; i < lidar_points.size() ; i++) {
            float radius = lidar_points[i].radius;

            // ignore bogus samples
            if (std::isnan(radius) || std::isinf(radius) || radius < lidar.min_range) continue;

	    float x = lidar_origin.x() + radius * (lidar_normal.x() * cosLUT[i] -
		 lidar_normal.y() * sinLUT[i]);

	    float y = lidar_origin.y() + radius * (lidar_normal.y() * cosLUT[i] +
		 lidar_normal.x() * sinLUT[i]);

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
	if (age < max_age) lines.emplace_back(sensor.left_vertex, sensor.right_vertex);
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

void LidarSensor::reset(const std::string & _frame,
                         const int _increment,
                         const double & _min_range,
                         const double & _max_range,
                         const double & _min_angle,
                         const double & _max_angle)
{
    this->frame_id = _frame;
    this->angle_increment = _increment;
    this->min_range = _min_range;
    this->max_range = _max_range;
    this->min_angle = _min_angle;
    this->max_angle = _max_angle;
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
