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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <string>

#include "move_basic/obstacle_points.h"

class ObstaclePointsTests : public ::testing::Test {
protected:
    virtual void SetUp() {
        listener = new tf2_ros::TransformListener(tf_buffer);
        obstacle_points = new ObstaclePoints(nh, tf_buffer);

        sonar_pub = nh.advertise<sensor_msgs::Range>("/sonars", 1);
    }

    virtual void TearDown() { 
	delete listener; 
	delete obstacle_points; 
    }

    ObstaclePoints* obstacle_points;

    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf2_ros::TransformListener *listener;

    ros::Publisher sonar_pub;
};

using ::testing::ElementsAre;
using ::testing::UnorderedElementsAre;
using ::testing::Pair;

TEST_F(ObstaclePointsTests, noSensors) {
    auto points = obstacle_points->get_points(ros::Duration(10));
    auto lines = obstacle_points->get_lines(ros::Duration(10));
    ASSERT_EQ(points.size(), 0u);
    ASSERT_EQ(lines.size(), 0u);
}

TEST_F(ObstaclePointsTests, testPoints) {
    obstacle_points->add_test_point(tf2::Vector3(1.0,0.0,0.0));
    auto points = obstacle_points->get_points(ros::Duration(10));
    auto lines = obstacle_points->get_lines(ros::Duration(10));
    ASSERT_EQ(points.size(), 1u);
    ASSERT_EQ(points[0], tf2::Vector3(1.0,0.0,0.0));
    ASSERT_EQ(lines.size(), 0u);


    obstacle_points->add_test_point(tf2::Vector3(2.0,0.0,0.0));
    points = obstacle_points->get_points(ros::Duration(10));
    lines = obstacle_points->get_lines(ros::Duration(10));
    ASSERT_EQ(points.size(), 2u);
    ASSERT_THAT(points, ElementsAre(tf2::Vector3(1,0,0), tf2::Vector3(2,0,0)));
    ASSERT_EQ(lines.size(), 0u);

    obstacle_points->clear_test_points();
    points = obstacle_points->get_points(ros::Duration(10));
    lines = obstacle_points->get_lines(ros::Duration(10));
    ASSERT_EQ(points.size(), 0u);
    ASSERT_EQ(lines.size(), 0u);
}

TEST_F(ObstaclePointsTests, singleSonar) {
    ros::Duration(0.1).sleep(); // If we don't do this the publish never happens for some reason
    sensor_msgs::Range msg;
    msg.field_of_view = 1.5708;
    msg.min_range = 0.05;
    msg.max_range = 10;
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.range = 1.0;
    sonar_pub.publish(msg);

    // TODO fix this to be less ugly
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    auto points = obstacle_points->get_points(ros::Duration(10));
    auto lines = obstacle_points->get_lines(ros::Duration(10));
    ASSERT_EQ(points.size(), 2u);
    ASSERT_NEAR(points[0].x(), sqrt(2)/2.0, 0.01);
    ASSERT_NEAR(points[0].y(), -sqrt(2)/2.0, 0.01);
    ASSERT_NEAR(points[1].x(), sqrt(2)/2.0, 0.01);
    ASSERT_NEAR(points[1].y(), sqrt(2)/2.0, 0.01);
    
    // We expect one line that consists of those 2 points
    ASSERT_EQ(lines.size(), 1u);
    ASSERT_THAT(lines, ElementsAre(Pair(points[0], points[1])));

    // Do the same tests with a different range
    msg.header.stamp = ros::Time::now();
    msg.range = 2.0;
    sonar_pub.publish(msg);

    // TODO fix this to be less ugly
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // We also want to make sure that the same sonar just updates the same points,
    // instead of adding new ones
    points = obstacle_points->get_points(ros::Duration(10));
    lines = obstacle_points->get_lines(ros::Duration(10));
    ASSERT_EQ(points.size(), 2u);
    ASSERT_NEAR(points[0].x(), sqrt(2), 0.01);
    ASSERT_NEAR(points[0].y(), -sqrt(2), 0.01);
    ASSERT_NEAR(points[1].x(), sqrt(2), 0.01);
    ASSERT_NEAR(points[1].y(), sqrt(2), 0.01);
    ASSERT_EQ(lines.size(), 1u);
    ASSERT_THAT(lines, ElementsAre(Pair(points[0], points[1])));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_points_test");
    ros::start();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
