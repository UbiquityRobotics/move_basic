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
#include <move_basic/collision_checker.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

class CollisionCheckerTests : public ::testing::Test {
protected:
    virtual void SetUp() {
        listener = new tf2_ros::TransformListener(tf_buffer);

        obstacle_points = new ObstaclePoints(nh, tf_buffer);
        collision_checker = new CollisionChecker(nh, tf_buffer, *obstacle_points);
    }

    virtual void TearDown() { 
	delete listener; 
	delete collision_checker; 
	delete obstacle_points; 
    }

    CollisionChecker* collision_checker;
    ObstaclePoints* obstacle_points;

    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener *listener;
};


TEST_F(CollisionCheckerTests, noObstacles) {
    tf2::Vector3 fl;
    tf2::Vector3 fr;
    float left;
    float right;
    float forward_dist = collision_checker->obstacle_dist(true, left, right, fl,fr);
    float backward_dist = collision_checker->obstacle_dist(false, left, right, fl,fr);
    ASSERT_FLOAT_EQ(forward_dist, 10.0 - 0.09);
    ASSERT_FLOAT_EQ(backward_dist, 10.0 - 0.19);
}

TEST_F(CollisionCheckerTests, forward) {
    tf2::Vector3 fl;
    tf2::Vector3 fr;
    float left;
    float right;
    obstacle_points->add_test_point(tf2::Vector3(1, 0, 0));
    float forward_dist = collision_checker->obstacle_dist(true, left, right, fl,fr);
    float backward_dist = collision_checker->obstacle_dist(false, left, right, fl,fr);
    ASSERT_FLOAT_EQ(forward_dist, 1.0 - 0.09);
    ASSERT_FLOAT_EQ(backward_dist, 10.0 - 0.19);
}

TEST_F(CollisionCheckerTests, backward) {
    tf2::Vector3 fl;
    tf2::Vector3 fr;
    float left;
    float right;
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(-1, 0, 0));
    float forward_dist = collision_checker->obstacle_dist(true, left, right, fl,fr);
    float backward_dist = collision_checker->obstacle_dist(false, left, right, fl,fr);
    ASSERT_FLOAT_EQ(forward_dist, 10.0 - 0.09);
    ASSERT_FLOAT_EQ(backward_dist, 1.0 - 0.19);
}

TEST_F(CollisionCheckerTests, noObstaclesRot) {
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, M_PI);
    ASSERT_FLOAT_EQ(right_angle, M_PI);
}

TEST_F(CollisionCheckerTests, obstaclesRotLeft) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(0, .15, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, M_PI);
    ASSERT_FLOAT_EQ(right_angle, 1.0082601);
}

TEST_F(CollisionCheckerTests, obstaclesRotRight) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(0, -.15, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 1.0082601);
    ASSERT_FLOAT_EQ(right_angle, M_PI);
}

TEST_F(CollisionCheckerTests, obstaclesRotLeftBack) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(-0.05, .15, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, M_PI);
    ASSERT_FLOAT_EQ(right_angle, 0.71854615);
}

TEST_F(CollisionCheckerTests, obstaclesRotRightBack) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(-0.05, -.15, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.71854615);
    ASSERT_FLOAT_EQ(right_angle, M_PI);
}

TEST_F(CollisionCheckerTests, obstaclesRotFrontCenter) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(0.1, 0.0, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.45102686);
    ASSERT_FLOAT_EQ(right_angle, 0.45102686);
}

TEST_F(CollisionCheckerTests, obstaclesRotFrontLeft) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(0.1, 0.01, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.56083643);
    ASSERT_FLOAT_EQ(right_angle, 0.36149913);
}

TEST_F(CollisionCheckerTests, obstaclesRotFrontRight) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(0.1, -0.01, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.36149913);
    ASSERT_FLOAT_EQ(right_angle, 0.56083643);
}

TEST_F(CollisionCheckerTests, obstaclesRotBackCenter) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(-0.145, 0.0, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.58442998);
    ASSERT_FLOAT_EQ(right_angle, 0.58443004);
}

TEST_F(CollisionCheckerTests, obstaclesRotBackLeft) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(-0.145, 0.01, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.51400685);
    ASSERT_FLOAT_EQ(right_angle, 0.65171939);
}

TEST_F(CollisionCheckerTests, obstaclesRotBackRight) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(-0.145, -0.01, 0));
    float left_angle = collision_checker->obstacle_angle(true);
    float right_angle = collision_checker->obstacle_angle(false);
    ASSERT_FLOAT_EQ(left_angle, 0.65171939);
    ASSERT_FLOAT_EQ(right_angle, 0.51400685);
}

TEST_F(CollisionCheckerTests, arcNoObstacles) {
    obstacle_points->clear_test_points();
    float t = collision_checker->obstacle_arc_angle(0.0,0.0);
    EXPECT_FLOAT_EQ(t, M_PI);
    t = collision_checker->obstacle_arc_angle(0.0,1.0);
    EXPECT_FLOAT_EQ(t, M_PI);
}

TEST_F(CollisionCheckerTests, arcForward) {
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(0.1,0,0));
    float left = collision_checker->obstacle_arc_angle(1.0,1.0);
    float right = collision_checker->obstacle_arc_angle(1.0,-1.0);
    EXPECT_FLOAT_EQ(left, -1.471128);
    EXPECT_FLOAT_EQ(right, 1.471128);

    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(1,1,0));
    obstacle_points->add_test_point(tf2::Vector3(1,-1,0));
    left = collision_checker->obstacle_arc_angle(1.0,1.0);
    right = collision_checker->obstacle_arc_angle(1.0,-1.0);
    EXPECT_FLOAT_EQ(left, 0.0);
    EXPECT_FLOAT_EQ(right, 0.0);

    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(0,0,0));
    left = collision_checker->obstacle_arc_angle(1.0,1.0);
    right = collision_checker->obstacle_arc_angle(1.0,-1.0);
    EXPECT_FLOAT_EQ(left, -M_PI/2);
    EXPECT_FLOAT_EQ(right, M_PI/2);

    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(-1,0,0));
    left = collision_checker->obstacle_arc_angle(1.0,1.0);
    right = collision_checker->obstacle_arc_angle(1.0,-1.0);
    EXPECT_FLOAT_EQ(left, M_PI);
    EXPECT_FLOAT_EQ(right, M_PI);
    
    obstacle_points->clear_test_points();
    obstacle_points->add_test_point(tf2::Vector3(1,-1,0));
    left = collision_checker->obstacle_arc_angle(1.0,1.0);
    right = collision_checker->obstacle_arc_angle(1.0,-1.0);
    EXPECT_FLOAT_EQ(left, M_PI);
    EXPECT_FLOAT_EQ(right, 0.0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "collision_checker_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
