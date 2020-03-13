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
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <move_basic/FollowMode.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "move_basic/obstacle_detector.h"

#include <string>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

enum {
    DRIVE_STRAIGHT,
    FOLLOW_LEFT,
    FOLLOW_RIGHT
};

class MoveBasic {
  private:
    ros::Subscriber goalSub;
    ros::Subscriber followSub;

    ros::Publisher goalPub;
    ros::Publisher cmdPub;
    ros::Publisher pathPub;
    ros::Publisher obstacle_dist_pub;
    ros::Publisher errorPub;

    std::unique_ptr<MoveBaseActionServer> actionServer;
    std::unique_ptr<ObstacleDetector> obstacle_detector;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;

    bool verboseInfo;
    bool smoothFollow;

    double smoothBreakingMin;
    double smoothBreakingMax; 
    double prevSmoothVelocity;

    double maxAngularVelocity;
    double minAngularVelocity;
    double angularAcceleration;
    double angularTolerance;

    double minLinearVelocity;
    double maxLinearVelocity;
    double linearAcceleration;
    double linearTolerance;

    // PID parameters for controlling lateral error
    double lateralKp;
    double lateralKi;
    double lateralKd;
    double lateralMaxRotation;

    int rotationAttempts;
    double localizationLatency;

    double robotWidth;
    double frontToLidar;
    double obstacleWaitLimit;

    std::string preferredPlanningFrame;
    std::string alternatePlanningFrame;
    std::string preferredDrivingFrame;
    std::string alternateDrivingFrame;
    std::string baseFrame;

    volatile int followMode;

    double forwardObstacleThreshold;

    double minSideDist;
    double maxSideDist;
    double maxLateralDev;
    double maxAngularDev;
    double sideTurnOutWeight;
    double sideTurnInWeight;
    double sideRecoverWeight;
    double maxFollowDistWithoutWall;

    float forwardObstacleDist;
    float leftObstacleDist;
    float rightObstacleDist;
    tf2::Vector3 forwardLeft;
    tf2::Vector3 forwardRight;
    double reverseWithoutTurningThreshold;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void executeAction(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    void drawLine(double x0, double y0, double x1, double y1);
    void sendCmd(double angular, double linear);
    void abortGoal(const std::string msg);

    bool getTransform(const std::string& from, const std::string& to,
                      tf2::Transform& tf);
    bool transformPose(const std::string& from, const std::string& to,
                       const tf2::Transform& in, tf2::Transform& out);

    void followModeCallback(const move_basic::FollowMode::ConstPtr &msg);

  public:
    MoveBasic();

    void run();

    bool moveLinear(const tf2::Transform& goalInDriving,
                    std::string drivingFrame);
    bool rotate(double requestedYaw, std::string drivingFrame);
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

// Get the sign of a number

static int sign(double n)
{
    return (n <0 ? -1 : 1);
}


// Adjust angle to be between -PI and PI

static void normalizeAngle(double& angle)
{
    if (angle < -M_PI) {
         angle += 2 * M_PI;
    }
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
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

MoveBasic::MoveBasic(): tfBuffer(ros::Duration(30.0)),
                        listener(tfBuffer), followMode(DRIVE_STRAIGHT)
{
    ros::NodeHandle nh("~");

    nh.param<double>("min_angular_velocity", minAngularVelocity, 0.05);
    nh.param<double>("max_angular_velocity", maxAngularVelocity, 1.0);
    nh.param<double>("angular_acceleration", angularAcceleration, 0.3);
    nh.param<double>("angular_tolerance", angularTolerance, 0.01);

    nh.param<double>("min_linear_velocity", minLinearVelocity, 0.1);
    nh.param<double>("max_linear_velocity", maxLinearVelocity, 0.5);
    nh.param<double>("linear_acceleration", linearAcceleration, 0.25);
    nh.param<double>("linear_tolerance", linearTolerance, 0.03);

    // Parameters for turn PID
    nh.param<double>("lateral_kp", lateralKp, 10.0);
    nh.param<double>("lateral_ki", lateralKi, 0.0);
    nh.param<double>("lateral_kd", lateralKd, 0.0);

    // Minimum distance to maintain at each side
    nh.param<double>("min_side_dist", minSideDist, 0.3);

    // Maximum distance to side before deciding there is nothing to follow
    nh.param<double>("max_side_dist", maxSideDist, 1.0);

    // Maximum deviation from linear path before aborting
    nh.param<double>("max_lateral_deviation", maxLateralDev, 4.0);

    // Maximum allowed deviation from straight path
    nh.param<double>("max_angular_deviation", maxAngularDev, deg2rad(20.0));

    // Maximum angular velocity during linear portion
    nh.param<double>("max_lateral_rotation", lateralMaxRotation, 0.5);

    // Weighting of turning towards followed wall
    nh.param<double>("side_turn_in_weight", sideTurnInWeight, 0.3);

    // Weighting of turning away from followed wall
    nh.param<double>("side_turn_out_weight", sideTurnOutWeight, 0.3);

    // Weighting of turning to recover from avoiding side obstacles
    nh.param<double>("side_recover_weight", sideRecoverWeight, 0.3);

    // How long to drive in follow mode without a wall
    nh.param<double>("max_follow_dist_without_wall", maxFollowDistWithoutWall, 0.5);

    // how long to wait after moving to be sure localization is accurate
    nh.param<double>("localization_latency", localizationLatency, 0.5);

    nh.param<int>("rotation_attempts", rotationAttempts, 1);

    // if distance < velocity times this we stop
    nh.param<double>("forward_obstacle_threshold", forwardObstacleThreshold, 1.5);

    // how long to wait for an obstacle to disappear
    nh.param<double>("obstacle_wait_limit", obstacleWaitLimit, 10.0);

    // Reverse distances for which rotation won't be performed
    nh.param<double>("reverse_without_turning_threshold",
                      reverseWithoutTurningThreshold, 0.5);

    nh.param<std::string>("preferred_planning_frame",
                          preferredPlanningFrame, "map");
    nh.param<std::string>("alternate_planning_frame",
                          alternatePlanningFrame, "odom");
    nh.param<std::string>("preferred_planning_frame",
                          preferredDrivingFrame, "map");
    nh.param<std::string>("alternate_planning_frame",
                          alternateDrivingFrame, "odom");
    nh.param<std::string>("base_frame", baseFrame, "base_link");

    nh.param<bool>("verbose_info", verboseInfo, false);

    nh.param<bool>("smooth_follow", smoothFollow, false);
    nh.param<double>("smooth_breaking_min", smoothBreakingMin, 0.005);
    nh.param<double>("smooth_breaking_max", smoothBreakingMax, 0.0075);

    prevSmoothVelocity = 0;

    cmdPub = ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    pathPub = ros::Publisher(nh.advertise<nav_msgs::Path>("/plan", 1));

    obstacle_dist_pub =
        ros::Publisher(nh.advertise<geometry_msgs::Vector3>("/obstacle_distance", 1));
    errorPub =
        ros::Publisher(nh.advertise<geometry_msgs::Vector3>("/lateral_error", 1));

    followSub = nh.subscribe("/follow_mode", 1, &MoveBasic::followModeCallback,
                             this);

    goalSub = nh.subscribe("/move_base_simple/goal", 1,
                            &MoveBasic::goalCallback, this);

    ros::NodeHandle actionNh("");
    actionServer.reset(new MoveBaseActionServer(actionNh,
        "move_base", boost::bind(&MoveBasic::executeAction, this, _1), false));

    actionServer->start();
    goalPub = actionNh.advertise<move_base_msgs::MoveBaseActionGoal>(
      "/move_base/goal", 1);

    obstacle_detector.reset(new ObstacleDetector(nh, &tfBuffer));

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
         ROS_WARN("%s", ex.what());
         return false;
    }
}


// Transform a pose from one frame to another

bool MoveBasic::transformPose(const std::string& from, const std::string& to,
                              const tf2::Transform& in, tf2::Transform& out)
{
    tf2::Transform tf;
    if (!getTransform(from, to, tf)) {
        return false;
    }
    out = tf * in;
    return true;
}


// Called when a follow mode message is received

void MoveBasic::followModeCallback(const move_basic::FollowMode::ConstPtr &msg)
{
    followMode = msg->follow_mode;
    ROS_INFO("MoveBasic: Received follow mode %d dist %f speed %f",
             followMode, msg->follow_dist, msg->speed);

    if (msg->follow_dist != 0) {
        minSideDist = msg->follow_dist;
    }
    if (msg->speed != 0) {
        maxLinearVelocity = msg->speed;
    }
}


// Called when a simple goal message is received

void MoveBasic::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("MoveBasic: Received simple goal");
    // send the goal to the action server
    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.header.stamp = ros::Time::now();
    actionGoal.goal.target_pose = *msg;

    goalPub.publish(actionGoal);
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
      Plan a path that involves rotating to face the goal, going straight towards it,
      and then rotating for the final orientation.

      It is assumed that we are dealing with imperfect localization data:
         map->base_link is accurate but may be delayed and is at a slow rate
         odom->base_link is frequent, but drifts, particularly after rotating

      To counter these issues, we plan in the map frame, and wait localizationLatency
      after each step, and execute in the odom frame.
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

    std::string planningFrame;
    double goalYaw;

    tf2::Transform goalPlanning;
    if (!transformPose(frameId, preferredPlanningFrame, goal, goalPlanning)) {
        ROS_WARN("MoveBasic: Will attempt to plan in %s frame", frameId.c_str());
        if (!transformPose(frameId, alternatePlanningFrame, goal,
            goalPlanning)) {
            abortGoal("MoveBasic: No localization available for planning");
            return;
        }
        planningFrame = alternatePlanningFrame;
        goalYaw = yaw;
    }
    else {
        planningFrame = preferredPlanningFrame;
    }

    getPose(goalPlanning, x, y, goalYaw);
    ROS_INFO("MoveBasic: Goal in %s  %f %f %f", planningFrame.c_str(),
             x, y, rad2deg(goalYaw));

    // publish our planned path
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

    std::string drivingFrame;
    tf2::Transform goalInDriving;
    tf2::Transform currentDrivingBase;
    // Should be at time of goal message
    if (!getTransform(preferredDrivingFrame, baseFrame, currentDrivingBase)) {
         ROS_WARN("MoveBasic: Attempting to drive using %s frame",
                  alternateDrivingFrame.c_str());
         if (!getTransform(alternateDrivingFrame,
                           baseFrame, currentDrivingBase)) {
             abortGoal("MoveBasic: Cannot determine robot pose in driving frame");
             return;
         }
         else {
             drivingFrame = alternateDrivingFrame;
         }
    }
    else {
         drivingFrame = preferredDrivingFrame;
    }

    if (!transformPose(frameId, drivingFrame, goal, goalInDriving)) {
         abortGoal("MoveBasic: Cannot determine robot pose in driving frame");
         return;
    }

    tf2::Transform goalInBase = currentDrivingBase * goalInDriving;

    tf2::Vector3 linear = goalInBase.getOrigin();
    bool reverseWithoutTurning =
        (reverseWithoutTurningThreshold > linear.length() && linear.x() < 0.0);

    // Initial rotation to face goal
    for (int i=0; i<rotationAttempts; i++) {
        tf2::Transform goalInBase;
        if (!transformPose(frameId, baseFrame, goal, goalInBase)) {
            ROS_WARN("MoveBasic: Cannot determine robot pose for rotation");
            return;
        }

        if (linear.length() > linearTolerance) {
            double requestedYaw = atan2(linear.y(), linear.x());
            if (reverseWithoutTurning) {
                if (requestedYaw > 0.0) {
                    requestedYaw = -M_PI + requestedYaw;
                }
                else {
                    requestedYaw = M_PI - requestedYaw;
                }
            }

            if (std::abs(requestedYaw) < angularTolerance) {
                break;
            }
            if (!rotate(requestedYaw, drivingFrame)) {
                return;
            }
            sleep(localizationLatency);
        }
    }

    // XXX bogus rotation to test linear recovery
    // TODO: incorporate this into testing
#ifdef TEST_PID
    ROS_INFO("MoveBasic: TEST rotation of 45 degrees!");
    rotate(deg2rad(45));
#endif

    // Do linear portion of goal
    double dist = linear.length();
    ROS_INFO("MoveBasic: Requested distance %f", dist);

    if (std::abs(dist) > linearTolerance) {
        if (reverseWithoutTurning) {
            dist = - dist;
        }
        if (!moveLinear(goalInDriving, drivingFrame)) {
            return;
        }
        {
            tf2::Transform poseFrameIdFinal;
            if (!getTransform(baseFrame, frameId, poseFrameIdFinal)) {
                 ROS_WARN("MoveBasic: Cannot determine robot pose in goal frame");
            }
            tf2::Vector3 distTravelled = poseFrameIdFinal.getOrigin() -
                                         poseFrameId.getOrigin();
            printf("MoveBasic: Travelled %f %f\n", distTravelled.x(), distTravelled.y());
        }

        sleep(localizationLatency);
    }

    // Final rotation as specified in goal
    tf2::Transform finalPose;
    if (!getTransform(baseFrame, drivingFrame, finalPose)) {
         abortGoal("MoveBasic: Cannot determine robot pose for final rotation");
         return;
    }

    if (followMode==DRIVE_STRAIGHT) {
        getPose(finalPose, x, y, yaw);
        rotate(goalYaw - yaw, drivingFrame);
    }

/*
    sleep(10);
    // Final sanity check
    {
        tf2::Transform poseFrameIdFinal;
        if (!getTransform(baseFrame, frameId, poseFrameIdFinal)) {
             ROS_WARN("Cannot determine robot pose in goal frame");
        }
        tf2::Vector3 distTravelled = poseFrameIdFinal.getOrigin() -
                                     poseFrameId.getOrigin();
        printf("Travelled %f %f\n", distTravelled.x(), distTravelled.y());
    }
*/

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
    std_msgs::Float32 msg;

    while (ros::ok()) {
        ros::spinOnce();
        obstacle_detector->min_side_dist = minSideDist;
        forwardObstacleDist = obstacle_detector->obstacle_dist(true,
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


// Rotate relative to current orientation

bool MoveBasic::rotate(double yaw, std::string drivingFrame)
{
    ROS_INFO("MoveBasic: Requested rotation %f", rad2deg(yaw));

    tf2::Transform poseDriving;
    if (!getTransform(baseFrame, drivingFrame, poseDriving)) {
         abortGoal("MoveBasic: Cannot determine robot pose for rotation");
         return false;
    }

    double x, y, currentYaw;
    getPose(poseDriving, x, y, currentYaw);
    double requestedYaw = currentYaw + yaw;
    normalizeAngle(requestedYaw);

    bool done = false;
    ros::Rate r(50);

    int oscillations = 0;
    double prevAngleRemaining = 0;

    while (!done && ros::ok()) {
        ros::spinOnce();
        r.sleep();

        double x, y, currentYaw;
        tf2::Transform poseDriving;
        if (!getTransform(baseFrame, drivingFrame, poseDriving)) {
            abortGoal("MoveBasic: Cannot determine robot pose for rotation");
            return false;
        }
        getPose(poseDriving, x, y, currentYaw);

        double angleRemaining = requestedYaw - currentYaw;
        normalizeAngle(angleRemaining);

        double obstacle = obstacle_detector->obstacle_angle(angleRemaining > 0);
        double remaining = std::min(std::abs(angleRemaining), std::abs(obstacle));
        double speed = std::max(minAngularVelocity,
            std::min(maxAngularVelocity,
              std::sqrt(2.0 * angularAcceleration *
                (remaining - angularTolerance))));

        double velocity = 0;

        if (angleRemaining < 0.0) {
            velocity = -speed;
        }
        else {
            velocity = speed;
        }

        if (sign(prevAngleRemaining) != sign(angleRemaining)) {
            oscillations++;
        }
        prevAngleRemaining = angleRemaining;

        if (actionServer->isNewGoalAvailable()) {
            ROS_INFO("MoveBasic: Stopping rotation due to new goal");
            done = true;
            velocity = 0;
        }

        //ROS_INFO("%f %f %f", rad2deg(angleRemaining), angleRemaining, velocity);

        if (std::abs(angleRemaining) < angularTolerance || oscillations > 2) {
            velocity = 0;
            done = true;
            ROS_INFO("MoveBasic: Done rotation, error %f degrees", rad2deg(angleRemaining));
        }

        // apply linear velocity for smooth slowdown and stop instantly if there's an obstacle
        if (smoothFollow) {

            if (forwardObstacleDist <= prevSmoothVelocity * forwardObstacleThreshold) {
                prevSmoothVelocity = 0;
            } else {
                double factor = std::min(std::abs(yaw)/2.0,1.0);
                prevSmoothVelocity -= smoothBreakingMin + (smoothBreakingMax - smoothBreakingMin) * factor;
                if (prevSmoothVelocity < 0) {
                    prevSmoothVelocity = 0;
                }
            }
            sendCmd(velocity, prevSmoothVelocity);
        }
        else {
            sendCmd(velocity, 0);
        }        
    }
    return done;
}

// Move forward specified distance

bool MoveBasic::moveLinear(const tf2::Transform& goalInDriving,
                           std::string drivingFrame)
{
    bool done = false;
    ros::Rate r(50);

    bool waitingForObstacle = false;
    int  waitingLoops = 0; 
    ros::Time obstacleTime;

    tf2::Transform poseDrivingInitial;
    if (!getTransform(baseFrame, drivingFrame, poseDrivingInitial)) {
         abortGoal("MoveBasic: Cannot determine robot pose for linear");
         return false;
    }

    double requestedDistance = (poseDrivingInitial.getOrigin() -
                                goalInDriving.getOrigin()).length();

    tf2::Transform poseDriving;
    if (!getTransform(drivingFrame, baseFrame, poseDriving)) {
         abortGoal("MoveBasic: Cannot determine robot pose for linear");
         return false;
    }

    tf2::Transform goalInBase = poseDriving * goalInDriving;
    tf2::Vector3 remaining = goalInBase.getOrigin();
    printf("MoveBasic: remaining %f %f\n", remaining.x(), remaining.y());
    bool forward = (remaining.x() > 0);

    // For lateral control
    double lateralIntegral = 0.0;
    double lateralError = 0.0;
    double prevLateralError = 0.0;
    double lateralDiff = 0.0;
    ros::Time sensorTime;

    bool hasWall = true;

    while (!done && ros::ok()) {
        ros::spinOnce();
        r.sleep();

        if (!getTransform(drivingFrame, baseFrame, poseDriving)) {
             ROS_WARN("MoveBasic: Cannot determine robot pose for linear");
             continue;
        }

        goalInBase = poseDriving * goalInDriving;
        remaining = goalInBase.getOrigin();
        if (!forward) {
           remaining = -remaining;
        }

        tf2::Transform initialBaseToCurrent = poseDrivingInitial * poseDriving;
        double cx, cy, cyaw;
        getPose(initialBaseToCurrent, cx, cy, cyaw);

        double distRemaining = remaining.x();
        double distTravelled = std::abs(requestedDistance) - std::abs(distRemaining);
        double firstDistWithoutWall = 0;

        double velMult = 1.0;

        if (followMode == FOLLOW_LEFT) {
            // Check encroachment vectors for a side obstacle in the future
            // Only pay attention to them if they are more restrictive than
            // the current side obstacles.
            if (forwardLeft.y() < minSideDist &&
                leftObstacleDist > minSideDist) {
                leftObstacleDist = forwardLeft.y();
                velMult = 0.5;
            }
            if (minSideDist > 0 && leftObstacleDist < 1.0) {
                if (leftObstacleDist < minSideDist) {
                    printf("out ");
                    lateralError = sideTurnOutWeight *
                        (leftObstacleDist - minSideDist);
                }
                else {
                    printf("in ");
                    lateralError = sideTurnInWeight *
                        (leftObstacleDist - minSideDist);
                }
            }
            else {
                lateralError = 0.0;
            }
        }
        else if (followMode == FOLLOW_RIGHT) {
            // Check encroachment vectors for a side obstacle in the future
            // Only pay attention to them if they are more restrictive than
            // the current side obstacles.
            if (forwardRight.y() < minSideDist &&
                rightObstacleDist > minSideDist) {
                rightObstacleDist = forwardRight.y();
                velMult = 0.5;
            }
            if (minSideDist > 0 && rightObstacleDist < 1.0) {
                if (leftObstacleDist < minSideDist) {
                    printf("out ");
                    lateralError = sideTurnOutWeight *
                        (minSideDist - rightObstacleDist);
                }
                else {
                    printf("in ");
                    lateralError = sideTurnInWeight *
                        (minSideDist - rightObstacleDist);
                }
            }
            else {
                lateralError = 0.0;
            }
        }
        else { // stick to planned path
            lateralError = sideRecoverWeight * remaining.y();
        }
/*
        Future enhancement: turn to avoid a forward obstacle

        bool canTurn = std::abs(cyaw) <= maxTurn;
        if (canTurn && forwardObstacleDist < 1.0) {
            if (leftObstacleDist > rightObstacleDist) {
               lateralError = lateralMaxRotation;
            }
            else {
               lateralError = -lateralMaxRotation;
            }
        }
*/
        // Abort if following a wall that is not there
        if (followMode != DRIVE_STRAIGHT && maxFollowDistWithoutWall > 0) {
            bool prevHasWall = hasWall;
            if (followMode == FOLLOW_LEFT) {
                hasWall = leftObstacleDist <= maxSideDist;
            }
            else if (followMode == FOLLOW_RIGHT) {
                hasWall = rightObstacleDist <= maxSideDist;
            }
            if (!hasWall && prevHasWall) {
                firstDistWithoutWall = distTravelled;
            }
            if (!hasWall && std::abs(firstDistWithoutWall - distTravelled) > maxFollowDistWithoutWall) {
                abortGoal("Aborting since no wall to follow");
                sendCmd(0, 0);
                return false;
            }
        }
        if (std::abs(remaining.y()) >= maxLateralDev) {
            abortGoal("MoveBasic: Aborting since max deviation reached");
            sendCmd(0, 0);
            return false;
        }

        // PID loop to control rotation to keep robot on path
        double rotation = 0.0;

        ros::Time t = obstacle_detector->stamp;
        if (t != sensorTime || followMode==DRIVE_STRAIGHT) {
            lateralDiff = lateralError - prevLateralError;
            prevLateralError = lateralError;
            sensorTime = t;
        }

        lateralIntegral += lateralError;

        rotation = (lateralKp * lateralError) + (lateralKi * lateralIntegral) +
                   (lateralKd * lateralDiff);

        // Clamp rotation
        rotation = std::max(-lateralMaxRotation, std::min(lateralMaxRotation,
                                                          rotation));

        // Limit angular deviation from planned path to prevent turning around
        if (cyaw > maxAngularDev && rotation < 0) {
            printf("limit right\n");
            rotation = 0;
        }
        else if (cyaw < -maxAngularDev && rotation > 0) {
            printf("limit left\n");
            rotation = 0;
        }

        if (verboseInfo) {
            printf("MoveBasic: Debug F %f L %f, R %f %f %f %f %f %f\n",
               forwardObstacleDist, leftObstacleDist, rightObstacleDist,
               remaining.x(), remaining.y(), lateralError,
               rotation, rad2deg(cyaw));
        }

        // Publish messages for PID tuning
        geometry_msgs::Vector3 pid_debug;
        pid_debug.x = remaining.x();
        pid_debug.y = lateralError;
        pid_debug.z = rotation;
        errorPub.publish(pid_debug);

        double obstacleDist = forwardObstacleDist;
        if (requestedDistance < 0.0) { // Reverse
            obstacleDist = obstacle_detector->obstacle_dist(false,
                                                            leftObstacleDist,
                                                            rightObstacleDist,
                                                            forwardLeft,
                                                            forwardRight);
        }

        double velocity = std::max(minLinearVelocity,
            std::min(maxLinearVelocity, std::min(
              std::sqrt(2.0 * linearAcceleration * std::abs(distTravelled)),
              std::sqrt(0.5 * linearAcceleration *
                 std::min(obstacleDist, distRemaining)))));

        // Stop if there is an obstacle in the distance we would hit in given time
        bool obstacleDetected = obstacleDist <= velocity * forwardObstacleThreshold;
        if (obstacleDetected) {
            velocity = 0;
            if (!waitingForObstacle) {
                ROS_INFO("MoveBasic: PAUSING for OBSTACLE");
                obstacleTime = ros::Time::now();
                waitingForObstacle = true;
                waitingLoops = 0;
            }
            else {
                waitingLoops += 1;
                if ((waitingLoops % 10) == 1) {
                    ROS_INFO("MoveBasic: Still waiting for obstacle at %f meters!", obstacleDist);
                }
                ros::Duration waitTime = ros::Time::now() - obstacleTime;
                if (waitTime.toSec() > obstacleWaitLimit) {
                    abortGoal("MoveBasic: Aborting due to obstacle");
                    sendCmd(0, 0);
                    return false;
                }
            }
        }

        if (waitingForObstacle && ! obstacleDetected) {
            ROS_INFO("MoveBasic: Resuming after obstacle has gone");
            waitingForObstacle = false;
            waitingLoops = 0;
            // start off again smoothly
            requestedDistance = distRemaining;
            distTravelled = 0.0;
        }

        if (actionServer->isNewGoalAvailable()) {
            ROS_INFO("MoveBasic: Stopping rotation due to new goal");
            done = true;

            if (!smoothFollow) {
                velocity = 0;
            }
        }

        // TODO: is this the best test of completeness?
        if (remaining.x() < linearTolerance) {
            velocity = 0;
            done = true;
            ROS_INFO("MoveBasic: Done linear, error %f, %f meters",
                     remaining.x(), remaining.y());
        }
        if (!forward) {
            velocity = -velocity;
        }

        // record linear velocity for smooth slowdown
        if (smoothFollow) {
            prevSmoothVelocity = velocity;
        }

        sendCmd(rotation, velMult * velocity);
    }
    return done;
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "move_basic");
    MoveBasic mb_node;
    mb_node.run();

    return 0;
}
