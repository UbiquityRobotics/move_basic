/*
 * Copyright (c) 2017-9, Ubiquity Robotics
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

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "move_basic/collision_checker.h"
#include "move_basic/queued_action_server.h"

#include <assert.h>
#include <string>
#include <condition_variable>
#include <mutex>
#include <chrono>

typedef actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBasic {
  private:
    ros::Subscriber goalSub;
    ros::Subscriber phGoalSub;
    ros::Subscriber nextGoalSub;

    ros::Publisher goalPub;
    ros::Publisher cmdPub;
    ros::Publisher pathPub;
    ros::Publisher obstacle_dist_pub;

    std::unique_ptr<MoveBaseActionServer> actionServer;
    std::unique_ptr<CollisionChecker> collision_checker;
    std::unique_ptr<ObstaclePoints> obstacle_points;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;

    std::string preferredDrivingFrame;
    std::string alternateDrivingFrame;
    std::string baseFrame;

    double maxAngularVelocity;
    double maxAngularAcceleration;
    double maxLinearVelocity;
    double maxLinearAcceleration;
    double linearTolerance;

    double maxIncline;
    double gravityConstant;
    double maxLateralDev;

    int goalId;
    int phantomCounter;
    bool nextGoalAvailable;
    bool abortPhantom;
    bool phantomGoalReceived;
    bool phantom;
    tf2::Transform nextGoalPose;
    std::string frameIdNext;

    double lateralKp;
    double lateralKi;
    double lateralKd;

    double abortTimeoutSecs;
    double velMult;
    double velThreshold;

    double obstacleWaitLimit;
    double forwardObstacleThreshold;

    double minSideDist;
    double minRequestDistance;
    double angleRecoverWeight;

    double reverseWithoutTurningThreshold;

    float forwardObstacleDist;
    float leftObstacleDist;
    float rightObstacleDist;
    tf2::Vector3 forwardLeft;
    tf2::Vector3 forwardRight;

    void phantomGoalCallback(const std_msgs::BoolConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void nextGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& goal);
    void executeAction(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    void drawLine(double x0, double y0, double x1, double y1);
    void sendCmd(double angular, double linear);
    void abortGoal(const std::string msg);

    bool getTransform(const std::string& from, const std::string& to,
                      tf2::Transform& tf);
    bool transformPose(const std::string& from, const std::string& to,
                       const tf2::Transform& in, tf2::Transform& out);

  public:
    MoveBasic();

    void run();

    bool smoothControl(double requestedDistance,
                       bool reverseWithoutTurning,
                       const std::string& drivingFrame,
                       tf2::Transform& goalInDriving);
};


// Radians to degrees

static double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}


// Degrees to radians

static double deg2rad(double deg)
{
    return deg / 180.0 * M_PI;
}

// Adjust angle to be between -PI and PI

static void normalizeAngle(double& angle)
{
    if (angle < -M_PI)
        angle += 2 * M_PI;
        assert (angle > -M_PI);

    if (angle > M_PI)
        angle -= 2 * M_PI;
        assert (angle < M_PI);
}


// retreive the 3 DOF we are interested in from a Transform

static void getPose(const tf2::Transform& tf, double& x, double& y, double& yaw)
{
    tf2::Vector3 trans = tf.getOrigin();
    x = trans.x();
    y = trans.y();

    double roll, pitch;
    tf.getBasis().getRPY(roll, pitch, yaw);
}


// Constructor

MoveBasic::MoveBasic(): tfBuffer(ros::Duration(3.0)),
                        listener(tfBuffer)
{
    ros::NodeHandle nh("~");
    nh.param<double>("max_angular_velocity", maxAngularVelocity, 2.0);
    nh.param<double>("angular_acceleration", maxAngularAcceleration, 5.0);
    nh.param<double>("max_linear_velocity", maxLinearVelocity, 0.5);
    nh.param<double>("linear_acceleration", maxLinearAcceleration, 1.1);
    nh.param<double>("linear_tolerance", linearTolerance, 0.2);

    // Parameters for turn PID
    nh.param<double>("lateral_kp", lateralKp, 4.0);
    nh.param<double>("lateral_ki", lateralKi, 0.0);
    nh.param<double>("lateral_kd", lateralKd, 0.0);

    // To prevent sliping and tipping over when turning
    nh.param<double>("max_incline_without_slipping", maxIncline, 0.005);
    nh.param<double>("gravity_acceleration", gravityConstant, 9.81);
    nh.param<double>("max_lateral_deviation", maxLateralDev, 0.4);

    // After a goal is reached isNexGoalAvailable() on action server needs some time to update
    // Afterwards in order to achieve smooth toggling between the goals a bool variable is set
    nh.param<bool>("next_goal_available", nextGoalAvailable, false);
    nh.param<int>("goal_id", goalId, 1);
    nh.param<int>("phantom_counter", phantomCounter, -1);

    // TODO: Description
    nh.param<bool>("phantom", phantom, false);
    nh.param<bool>("phantom_goal_received", phantomGoalReceived, false);
    nh.param<bool>("abort_phantom", abortPhantom, false);

    // Minimum distance to maintain at each side
    nh.param<double>("min_side_dist", minSideDist, 0.3);
    nh.param<double>("minimum_requested_distance", minRequestDistance, 0.2);

    // Weighting of turning to recover from avoiding side obstacles
    nh.param<double>("side_recover_weight", angleRecoverWeight, 0.3);

    // how long to wait for an obstacle to disappear
    nh.param<double>("obstacle_wait_limit", obstacleWaitLimit, 10.0);

    // if distance <  velocity times this we stop
    nh.param<double>("forward_obstacle_threshold", forwardObstacleThreshold, 1.5);

    // Reverse distances for which rotation won't be performed
    nh.param<double>("reverse_without_turning_threshold",
                        reverseWithoutTurningThreshold, 0.5);

    nh.param<double>("abort_timeout", abortTimeoutSecs, 60.0);

    // Proportional controller for following the goal
    nh.param<double>("velocity_threshold", velThreshold, 0.01);
    nh.param<double>("velocity_multiplier", velMult, 1.0);

    nh.param<std::string>("preferred_driving_frame",
                         preferredDrivingFrame, "map");
    nh.param<std::string>("alternate_driving_frame",
                          alternateDrivingFrame, "odom");
    nh.param<std::string>("base_frame", baseFrame, "base_footprint");

    cmdPub = ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    pathPub = ros::Publisher(nh.advertise<nav_msgs::Path>("/plan", 1));

    obstacle_dist_pub =
        ros::Publisher(nh.advertise<geometry_msgs::Vector3>("/obstacle_distance", 1));

    goalSub = nh.subscribe("/move_base_simple/goal", 1,
                            &MoveBasic::goalCallback, this);

    phGoalSub = nh.subscribe("/phantom", 1,
                            &MoveBasic::phantomGoalCallback, this);
    nextGoalSub = nh.subscribe("/move_base/goal", 1,
                             &MoveBasic::nextGoalCallback, this);

    ros::NodeHandle actionNh("");

    actionServer.reset(new MoveBaseActionServer(actionNh, "move_base",
	        boost::bind(&MoveBasic::executeAction, this, _1)));

    actionServer->start();
    goalPub = actionNh.advertise<move_base_msgs::MoveBaseActionGoal>(
      "/move_base/goal", 1);

    obstacle_points.reset(new ObstaclePoints(nh, tfBuffer));
    collision_checker.reset(new CollisionChecker(nh, tfBuffer, *obstacle_points));

    ROS_INFO("Move Basic ready");
}


// Lookup the specified transform, returns true on success

bool MoveBasic::getTransform(const std::string& from, const std::string& to,
                             tf2::Transform& tf)
{
    try {
        geometry_msgs::TransformStamped tfs =
            tfBuffer.lookupTransform(to, from, ros::Time(0));
        tf2::fromMsg(tfs.transform, tf);
        return true;
    }
    catch (tf2::TransformException &ex) {
         return false;
    }
}


// Transform a pose from one frame to another, returns true on success

bool MoveBasic::transformPose(const std::string& from, const std::string& to,
                              const tf2::Transform& in, tf2::Transform& out)
{
    tf2::Transform tf;
    if (!getTransform(from, to, tf))
        return false;

    out = tf * in;
    return true;
}

// Called when a phantom goal message is received

void MoveBasic::phantomGoalCallback(const std_msgs::BoolConstPtr &msg)
{
    phantomGoalReceived = msg->data;
    if (phantomGoalReceived) {
        ROS_INFO("MoveBasic: Received phantom goal");
        phantom = true;
    }
}


// Called when a simple goal message is received

void MoveBasic::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!phantomGoalReceived)
        ROS_INFO("MoveBasic: Received simple goal");

    ROS_INFO("if = 1 that is phantom --> %d", phantomGoalReceived);
    // send the goal to the action server
    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.header.stamp = ros::Time::now();
    if (phantom) {
        actionGoal.goal_id.id = std::to_string(phantomCounter);
        phantomCounter = phantomCounter - 1;
        phantom = false;
    }
    else {
        actionGoal.goal_id.id = std::to_string(goalId);
        goalId = goalId + 1;
    }
    actionGoal.goal.target_pose = *msg;
    goalPub.publish(actionGoal);

    // isNewGoalAvailable() needs to update on an actionServer
    std::condition_variable newgoal_cv;
    std::mutex cv_m;
    std::unique_lock<std::mutex> newgoal_lk(cv_m);
    auto now = std::chrono::system_clock::now();
    auto timeout = std::chrono::milliseconds(50);
    newgoal_cv.wait_until(newgoal_lk,
            now + timeout,
            [this](){return actionServer->isNewGoalAvailable();}
    );

    // If there is a new goal store it
    if(actionServer->isNewGoalAvailable()) {
        tf2::fromMsg(msg->pose, nextGoalPose);
        frameIdNext = msg->header.frame_id;
        // Needed for RobotCommander
        if (frameIdNext[0] == '/')
            frameIdNext = frameIdNext.substr(1);

        nextGoalAvailable = true;
    }
}

void MoveBasic::nextGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& msg)
{
    // isNewGoalAvailable() needs to update on an actionServer
    std::condition_variable newgoal_cv;
    std::mutex cv_m;
    std::unique_lock<std::mutex> newgoal_lk(cv_m);
    auto now = std::chrono::system_clock::now();
    auto timeout = std::chrono::milliseconds(50);
    newgoal_cv.wait_until(newgoal_lk,
            now + timeout,
            [this](){return actionServer->isNewGoalAvailable();}
    );

    // If there is a new goal store it
    if(actionServer-> isNewGoalAvailable()) {
        ROS_INFO("MoveBasic: Next goal received");
        tf2::fromMsg(msg->goal.target_pose.pose, nextGoalPose);
        frameIdNext = msg->goal.target_pose.header.frame_id;
        // Needed for RobotCommander
        if (frameIdNext[0] == '/')
            frameIdNext = frameIdNext.substr(1);

        nextGoalAvailable = true;
    }
}


// Abort goal and print message

void MoveBasic::abortGoal(const std::string msg)
{
    actionServer->setAborted(move_base_msgs::MoveBaseResult(), msg);
    ROS_ERROR("%s", msg.c_str());
}


// Called when an action goal is received

void MoveBasic::executeAction(const move_base_msgs::MoveBaseGoalConstPtr& msg)
{
    /*
      It is assumed that we are dealing with imperfect localization data:
         map->base_link is accurate but may be delayed and is at a slow rate
         odom->base_link is frequent, but drifts, particularly after rotating
    */

    tf2::Transform goal;
    tf2::fromMsg(msg->target_pose.pose, goal);
    std::string frameId = msg->target_pose.header.frame_id;

    // Needed for RobotCommander
    if (frameId[0] == '/')
        frameId = frameId.substr(1);

    double x, y, yaw;
    getPose(goal, x, y, yaw);
    ROS_INFO("MoveBasic: Received goal %f %f %f %s", x, y, rad2deg(yaw), frameId.c_str());
    if (std::isnan(yaw)) {
        abortGoal("MoveBasic: Aborting goal because an invalid orientation was specified");
        return;
    }

    // Driving frame
    std::string drivingFrame;
    tf2::Transform goalInDriving;
    tf2::Transform currentDrivingBase;
    if (!getTransform(preferredDrivingFrame, baseFrame, currentDrivingBase)) {
         ROS_WARN("MoveBasic: %s not available, attempting to drive using %s frame",
                  preferredDrivingFrame.c_str(), alternateDrivingFrame.c_str());
         if (!getTransform(alternateDrivingFrame,
                           baseFrame, currentDrivingBase)) {
             abortGoal("MoveBasic: Cannot determine robot pose in driving frame");
             return;
         }
         else
             drivingFrame = alternateDrivingFrame;
    }
    else {
      drivingFrame = preferredDrivingFrame;
    }

    // Publish our planned path
    nav_msgs::Path path;
    geometry_msgs::PoseStamped p0, p1;
    path.header.frame_id = frameId;
    p0.pose.position.x = x;
    p0.pose.position.y = y;
    p0.header.frame_id = frameId;
    path.poses.push_back(p0);

    tf2::Transform poseFrameId;
    if (!getTransform(baseFrame, frameId, poseFrameId)) {
         abortGoal("MoveBasic: Cannot determine robot pose in goal frame");
         return;
    }
    getPose(poseFrameId, x, y, yaw);
    p1.pose.position.x = x;
    p1.pose.position.y = y;
    p1.header.frame_id = frameId;
    path.poses.push_back(p1);

    pathPub.publish(path);


    // Current goal in driving frame
    if (!transformPose(frameId, drivingFrame, goal, goalInDriving)) {
         abortGoal("MoveBasic: Cannot determine goal pose in driving frame");
         return;
    }
    tf2::Transform goalInBase = currentDrivingBase * goalInDriving;
    {
       double x, y, yaw;
       getPose(goalInBase, x, y, yaw);
       ROS_INFO("MoveBasic: Goal in %s  %f %f %f", baseFrame.c_str(),
             x, y, rad2deg(yaw));
    }

    // Driving distance
    tf2::Vector3 linear = goalInBase.getOrigin();
    double requestedDistance = sqrt(linear.x() * linear.x() + linear.y() * linear.y());

    // Driving direction
    bool reverseWithoutTurning = (reverseWithoutTurningThreshold > requestedDistance && linear.x() < 0.0);

    // Send control commands
    if (requestedDistance > minRequestDistance) {
        if (!smoothControl(requestedDistance, reverseWithoutTurning, drivingFrame, goalInDriving)) {
            return;
        }
    }
    else {
        ROS_WARN("Requested distance is lower than the minimum distance threshold ( %f meters)", minRequestDistance);
        abortGoal("MoveBasic: Aborting due to goal being already close enough.");
        return;
    }

    actionServer->setSucceeded();
}

// Send a motion command

void MoveBasic::sendCmd(double angular, double linear)
{
   geometry_msgs::Twist msg;
   msg.angular.z = angular;
   msg.linear.x = linear;

   cmdPub.publish(msg);
}


// Main loop

void MoveBasic::run()
{
    ros::Rate r(20);


    while (ros::ok()) {
        ros::spinOnce();
        collision_checker->min_side_dist = minSideDist;
        forwardObstacleDist = collision_checker->obstacle_dist(true,
                                                               leftObstacleDist,
                                                               rightObstacleDist,
                                                               forwardLeft,
                                                               forwardRight);
        geometry_msgs::Vector3 msg;
        msg.x = forwardObstacleDist;
        msg.y = leftObstacleDist;
        msg.z = rightObstacleDist;
        obstacle_dist_pub.publish(msg);

        r.sleep();
    }
}


// Smooth control drive

bool MoveBasic::smoothControl(double requestedDistance,
                              bool reverseWithoutTurning,
                              const std::string& drivingFrame,
                              tf2::Transform& goalInDriving)
{
        // Driving direction
        bool forward = true;

        // For Abort check
        ros::Time last = ros::Time::now();
        ros::Duration abortTimeout(abortTimeoutSecs);

        // For de/acceleration constraint
        double previousLinearVelocity = 0.0;
        double prevDistance = requestedDistance;
        ros::Time previousTime = ros::Time::now();

        // For lateral control
        double lateralIntegral = 0.0;
        double lateralError = 0.0;
        double prevLateralError = 0.0;
        double lateralDiff = 0.0;

        // For Obstacle Avoidance
        bool waitingForObstacle = false;
        int  waitingLoops = 0;
        ros::Time obstacleTime;

        bool done = false;
        ros::Rate r(50);

        while(!done && ros::ok()){
            ros::spinOnce();
            r.sleep();

            tf2::Transform poseDriving;
            if (!getTransform(drivingFrame, baseFrame, poseDriving)) {
                 ROS_WARN("MoveBasic: Cannot determine robot pose for driving");
                 continue;
            }

            // Current goal state in base frame
            tf2::Transform goalInBase = poseDriving * goalInDriving;
            tf2::Vector3 remaining = goalInBase.getOrigin();
            double distRemaining = sqrt(remaining.x() * remaining.x() + remaining.y() * remaining.y());
            ROS_INFO("distRemaining: %f", distRemaining);
            double angleRemaining = atan2(remaining.y(), remaining.x());
            normalizeAngle(angleRemaining);

            // If the goal is right behing the robot, it drives backwards to it
            if (reverseWithoutTurning) {
                if (angleRemaining > 0.0)
                    angleRemaining = -M_PI + angleRemaining;
                else
                    angleRemaining = M_PI - angleRemaining;
                forward = false;
            }

            // Next goal state in base frame, if available
            double angleToNextGoal;
            double distanceToNextGoal;
            if (nextGoalAvailable) {
                tf2::Transform nextGoalInDriving;
                // Next goal in driving frame
                if (!transformPose(frameIdNext, drivingFrame, nextGoalPose, nextGoalInDriving)) {
                     abortGoal("MoveBasic: Cannot determine next goal pose in driving frame");
                     done = false;
                     goto FinishWithStop;
                }
                tf2::Transform nextGoalInBase = poseDriving * nextGoalInDriving;
                tf2::Vector3 nextRemaining = nextGoalInBase.getOrigin();
                angleToNextGoal = atan2(nextRemaining.y(), nextRemaining.x());
                normalizeAngle(angleToNextGoal);
                distanceToNextGoal = sqrt(nextRemaining.x() * nextRemaining.x() + nextRemaining.y() * nextRemaining.y());
            }


            // Control to reference pose //

            // Rotational part
            double pidAngularVelocity = 0.0;
            lateralError = angleRecoverWeight * angleRemaining;
            lateralDiff = lateralError - prevLateralError;
            prevLateralError = lateralError;
            lateralIntegral += lateralError;
            pidAngularVelocity = (lateralKp * lateralError) + (lateralKi * lateralIntegral) +
                       (lateralKd * lateralDiff);

            double obstacle = collision_checker->obstacle_angle(angleRemaining > 0);
            double obstacleAngle = std::min(std::abs(angleRemaining), std::abs(obstacle));
            double angularVelocity = std::max(-maxAngularVelocity,
                                        std::min(maxAngularVelocity,
                                        std::min(pidAngularVelocity, sqrt(2.0 * maxAngularAcceleration * obstacleAngle))));

            // Distance to obstacle
            double obstacleDist = forwardObstacleDist;
            if (requestedDistance < 0.0) { // Reverse
                obstacleDist = collision_checker->obstacle_dist(false,
                                                        leftObstacleDist,
                                                        rightObstacleDist,
                                                        forwardLeft,
                                                        forwardRight);
            }
            // TODO: ROS_INFO("MoveBasic: %f L %f, R %f\n",
                    // TODO: forwardObstacleDist, leftObstacleDist, rightObstacleDist);

            // Turning Algorithm that calculates the maximum allowed speed when cornering in order not to slip or tip over
            double maxTurnVelocity = sqrt(maxLateralDev * gravityConstant * maxIncline/(1-cos(angleRemaining/2)));
            double linearVelocity = std::max(-maxLinearVelocity,
                                        std::min(std::min(maxLinearVelocity,
                                        std::min(std::min(velMult * distRemaining, maxTurnVelocity),
                                        sqrt(2.0 * maxLinearAcceleration * obstacleDist))),
                                        velMult * (previousLinearVelocity + 2.0 * maxLinearAcceleration * distRemaining)));

            // In order to achieve smooth toggling between going linear and into the turn, minimum output velocity
            // of the Turning Algorithm is calculated so that the linear velocity(going into the turn) does not get lower than that
            if (nextGoalAvailable) {
                double startTurnVelocity = sqrt(gravityConstant * maxIncline * maxLateralDev/(1-cos(angleToNextGoal/2)));
                double nextGoalVelocity = velMult * distanceToNextGoal;
                linearVelocity = std::min(nextGoalVelocity,
                                    std::min(maxLinearVelocity,
                                    std::max(linearVelocity, startTurnVelocity)));
            }
            if (!forward)
                linearVelocity = -linearVelocity;

            // TODO: Delete this!
            linearVelocity = std::min(linearVelocity, 0.7);

            previousLinearVelocity = linearVelocity;
            previousTime = ros::Time::now();

            // Send control command
            sendCmd(angularVelocity, linearVelocity);


            // Obstacle Avoidance //

            // Stop if there is an obstacle in the distance we would hit in given time
            bool obstacleDetected = (obstacleDist <= linearVelocity * forwardObstacleThreshold);
            if (obstacleDetected) {
                linearVelocity = 0;
                if (!waitingForObstacle) {
                    ROS_INFO("MoveBasic: PAUSING for OBSTACLE");
                    obstacleTime = ros::Time::now();
                    waitingForObstacle = true;
                    waitingLoops = 0;
                }
                else {
                    waitingLoops += 1;
                    if ((waitingLoops % 10) == 1)
                        ROS_INFO("MoveBasic: Still waiting for obstacle at %f meters!", obstacleDist);
                    ros::Duration waitTime = ros::Time::now() - obstacleTime;
                    if (waitTime.toSec() > obstacleWaitLimit) {
                        abortGoal("MoveBasic: Aborting due to obstacle");
                        done = false;
                        goto FinishWithStop;
                    }
                }
            }

            if (waitingForObstacle && ! obstacleDetected) {
                ROS_INFO("MoveBasic: Resuming after obstacle has gone");
                waitingForObstacle = false;
                waitingLoops = 0;
            }


            // Abort Checks //

            if (distRemaining < prevDistance) {
                prevDistance = distRemaining;
                if (ros::Time::now() - last > abortTimeout) {
                        abortGoal("MoveBasic: No progress towards goal for longer than timeout");
                        done = false;
                        goto FinishWithStop;
                }
            }

            if (actionServer->isPreemptRequested() && phantomGoalReceived) {
                ROS_INFO("MoveBasic: Preempting phantom goal");
                phantomGoalReceived = false;
                actionServer->setPreempted();
                done = false;
                goto FinishWithoutStop;
            }

            if (actionServer->isPreemptRequested()) {
                ROS_INFO("MoveBasic: Stopping due to preempt request");
                actionServer->setPreempted();
                done = false;
                goto FinishWithStop;
            }

            // Check navigation complete //

            // Checks if intermediate reference pose reached
            if (distRemaining < linearTolerance && nextGoalAvailable) { // Checks if in tolerance range
                ROS_INFO("MoveBasic: Goal reached - ERROR: x: %f meters, y: %f meters, yaw: %f degrees", remaining.x(), remaining.y(), rad2deg(angleRemaining));
                nextGoalAvailable = false;
                phantomGoalReceived = false;
                done = true;
                goto FinishWithoutStop;
            }

            // Checks if final reference pose reached
            if ((std::abs(linearVelocity) < velThreshold) && !nextGoalAvailable && !waitingForObstacle && (prevDistance+linearTolerance < requestedDistance)) {
                ROS_DEBUG("Remaining velocity: %f", linearVelocity);
                sendCmd(0, 0);
                nextGoalAvailable = false;
                if (distRemaining < linearTolerance) { // Checks if in tolerance range
                    ROS_INFO("MoveBasic: Goal reached - ERROR: x: %f meters, y: %f meters, yaw: %f degrees", remaining.x(), remaining.y(), rad2deg(angleRemaining));
                    done = true;
                    phantomGoalReceived = false;
                }
                else {
                    abortGoal("MoveBasic: Aborting due to missing the goal");
                    actionServer->setAborted();
                    done = false;
                }
                goto FinishWithStop;
            }
        }
   FinishWithStop:
        sendCmd(0, 0);

   FinishWithoutStop:

   return done;
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "move_basic");
    MoveBasic mb_node;
    mb_node.run();

    return 0;
}
