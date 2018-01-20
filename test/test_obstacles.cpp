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

#include <ros/ros.h>
#include <move_basic/obstacle_detector.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#if defined(__linux__)
#include <pty.h>
#else
#include <util.h>
#endif

class ObstacleTests : public ::testing::Test {
protected:
    virtual void SetUp() {
        listener = new tf2_ros::TransformListener(tf_buffer);

        obstacle_detector = new ObstacleDetector(nh, &tf_buffer);
    }

    virtual void TearDown() { delete obstacle_detector; }

    ObstacleDetector* obstacle_detector;

    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener *listener;
};

/*
TEST_F(ObstacleTests, noObstacles) {
    float forward_dist = obstacle_detector->obstacle_dist(true);
    float backward_dist = obstacle_detector->obstacle_dist(false);
    ASSERT_FLOAT_EQ(forward_dist, 10.0);
    ASSERT_FLOAT_EQ(backward_dist, 10.0);
}

TEST_F(ObstacleTests, forward) {
    obstacle_detector->add_test_point(tf2::Vector3(1, 0, 0));
    float forward_dist = obstacle_detector->obstacle_dist(true);
    float backward_dist = obstacle_detector->obstacle_dist(false);
    ASSERT_FLOAT_EQ(forward_dist, 0.95);
    ASSERT_FLOAT_EQ(backward_dist, 10.0);
}

TEST_F(ObstacleTests, backward) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(-1, 0, 0));
    float forward_dist = obstacle_detector->obstacle_dist(true);
    float backward_dist = obstacle_detector->obstacle_dist(false);
    ASSERT_FLOAT_EQ(forward_dist, 10.0);
    ASSERT_FLOAT_EQ(backward_dist, 0.88);
}
*/

TEST_F(ObstacleTests, noObstaclesRot) {
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, M_PI);
    ASSERT_FLOAT_EQ(right_angle, M_PI);
}

TEST_F(ObstacleTests, obstaclesRotLeft) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(0, .15, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, M_PI);
    ASSERT_FLOAT_EQ(right_angle, 1.0082601);
}

TEST_F(ObstacleTests, obstaclesRotRight) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(0, -.15, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 1.0082601);
    ASSERT_FLOAT_EQ(right_angle, M_PI);
}

TEST_F(ObstacleTests, obstaclesRotLeftBack) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(-0.05, .15, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, M_PI);
    ASSERT_FLOAT_EQ(right_angle, 0.71854615);
}

TEST_F(ObstacleTests, obstaclesRotRightBack) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(-0.05, -.15, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.71854615);
    ASSERT_FLOAT_EQ(right_angle, M_PI);
}

TEST_F(ObstacleTests, obstaclesRotFrontCenter) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(0.1, 0.0, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.45102686);
    ASSERT_FLOAT_EQ(right_angle, 0.45102686);
}

TEST_F(ObstacleTests, obstaclesRotFrontLeft) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(0.1, 0.01, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.56083643);
    ASSERT_FLOAT_EQ(right_angle, 0.36149913);
}

TEST_F(ObstacleTests, obstaclesRotFrontRight) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(0.1, -0.01, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.36149913);
    ASSERT_FLOAT_EQ(right_angle, 0.56083643);
}

TEST_F(ObstacleTests, obstaclesRotBackCenter) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(-0.145, 0.0, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.58442998);
    ASSERT_FLOAT_EQ(right_angle, 0.58443004);
}

TEST_F(ObstacleTests, obstaclesRotBackLeft) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(-0.145, 0.01, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.51400685);
    ASSERT_FLOAT_EQ(right_angle, 0.65171939);
}

TEST_F(ObstacleTests, obstaclesRotBackRight) {
    obstacle_detector->clear_test_points();
    obstacle_detector->add_test_point(tf2::Vector3(-0.145, -0.01, 0));
    float left_angle = obstacle_detector->obstacle_angle(true);
    float right_angle = obstacle_detector->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.65171939);
    ASSERT_FLOAT_EQ(right_angle, 0.51400685);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
