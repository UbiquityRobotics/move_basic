/*
 * Copyright (c) 2017, Ubiquity Robotics
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
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <string>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBasic {
  private:
    ros::Subscriber goalSub;
    ros::Subscriber scanSub;

    ros::Publisher goalPub;
    ros::Publisher cmdPub;
    ros::Publisher pathPub;
    ros::Publisher linePub;

    std::unique_ptr<MoveBaseActionServer> actionServer;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;

    double maxAngularVelocity;
    double minAngularVelocity;
    double angularAcceleration;
    double angularTolerance;

    double minLinearVelocity;
    double maxLinearVelocity;
    double linearAcceleration;
    double linearTolerance;

    int rotationAttempts;
    double localizationLatency;

    double robotWidth;
    double frontToLidar;
    double obstacleWaitLimit;

    double obstacleDist;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
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

    bool moveLinear(double requestedDistance);
    bool rotate(double requestedYaw);
};


// Radians to degrees

static double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
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
                        listener(tfBuffer), obstacleDist(10.0)
{
    ros::NodeHandle nh("~");

    nh.param<double>("min_angular_velocity", minAngularVelocity, 0.05);
    nh.param<double>("max_angular_velocity", maxAngularVelocity, 1.0);
    nh.param<double>("angular_acceleration", angularAcceleration, 0.3);
    nh.param<double>("angular_tolerance", angularTolerance, 0.01);

    nh.param<double>("min_linear_velocity", minLinearVelocity, 0.1);
    nh.param<double>("max_linear_velocity", maxLinearVelocity, 0.5);
    nh.param<double>("linear_acceleration", linearAcceleration, 0.25);
    nh.param<double>("linear_tolerance", linearTolerance, 0.01);

    // how long to wait after moving to be sure localization is accurate
    nh.param<double>("localization_latency", localizationLatency, 0.5);
    nh.param<int>("rotation_attempts", rotationAttempts, 2);

    nh.param<double>("robot_width", robotWidth, 0.35);
    // distance from lidar center to front-most part of robot
    nh.param<double>("front_to_lidar", frontToLidar, 0.11);
    // how long to wait for an obstacle to disappear
    nh.param<double>("obstacle_wait_limit", obstacleWaitLimit, 10.0);

    cmdPub = ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    pathPub = ros::Publisher(nh.advertise<nav_msgs::Path>("/plan", 1));
    linePub = ros::Publisher(nh.advertise<visualization_msgs::Marker>("/obstacle", 1));

    scanSub = nh.subscribe("/scan", 1, &MoveBasic::scanCallback, this);

    goalSub = nh.subscribe("/move_base_simple/goal", 1,
                            &MoveBasic::goalCallback, this);

    ros::NodeHandle actionNh("");
    actionServer.reset(new MoveBaseActionServer(actionNh,
        "move_base", boost::bind(&MoveBasic::executeAction, this, _1), false));

    actionServer->start();
    goalPub = actionNh.advertise<move_base_msgs::MoveBaseActionGoal>(
      "/move_base/goal", 1);

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


// Called when a laser scan is received assumes laser scanner is
// mounted at around base_link. Sets obstacleDetected

void MoveBasic::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    float angle = msg->angle_min;
    float increment = msg->angle_increment;
    float width_2 = robotWidth / 2.0;
    float rangeMin = msg->range_min;
    float minDist = msg->range_max;

    for (int i=0; i<msg->ranges.size(); i++) {
        angle += increment;

        float r = msg->ranges[i];

        // ignore bogus samples
        if (std::isnan(r) || r < rangeMin) {
            continue;
        }

        float y = r * sin(angle);

        // ignore anything outside width of robot
        if (std::abs(y) > width_2) {
            continue;
        }

        float x = r * cos(angle);

        // ignore anything behind center of lidar
        if (x < 0) {
            continue;
        }

        if (x < minDist) {
            minDist = x;
        }
    }
    obstacleDist = minDist - frontToLidar;

    drawLine(minDist, -width_2, minDist, width_2);
}


// Called when a simple goal message is received

void MoveBasic::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("Received simple goal");
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

    ROS_INFO("Received goal %f %f %f %s", x, y, rad2deg(yaw), frameId.c_str());

    if (std::isnan(yaw)) {
        abortGoal("Aborting goal because an invalid orientation was specified");
        return;
    }

    tf2::Transform goalMap;
    if (!transformPose(frameId, "map", goal, goalMap)) {
        abortGoal("Cannot determine robot pose");
        return;
    }

    double goalYaw;
    getPose(goalMap, x, y, goalYaw);
    ROS_INFO("Goal in map  %f %f %f", x, y, rad2deg(goalYaw));

    // publish our planned path
    nav_msgs::Path path;
    geometry_msgs::PoseStamped p0, p1;
    path.header.frame_id = "map";
    p0.pose.position.x = x;
    p0.pose.position.y = y;
    path.poses.push_back(p0);

    tf2::Transform poseMap;
    if (!getTransform("base_link", "map", poseMap)) {
         abortGoal("Cannot determine robot pose");
         return;
    }
    getPose(poseMap, x, y, yaw);
    p1.pose.position.x = x;
    p1.pose.position.y = y;
    path.poses.push_back(p1);

    pathPub.publish(path);

    // Initial rotation to face goal
    for (int i=0; i<rotationAttempts; i++) {
        tf2::Transform goalInBase;
        if (!transformPose(frameId, "base_link", goal, goalInBase)) {
            ROS_WARN("Cannot determine robot pose for rotation");
            return;
        }

        tf2::Vector3 offset = goalInBase.getOrigin();
        if (offset.length() != 0.0) {
            double requestedYaw = atan2(offset.y(), offset.x());

            if (std::abs(requestedYaw) < angularTolerance) {
                break;
            }
            if (!rotate(requestedYaw)) {
                return;
            }
            sleep(localizationLatency);
        }
    }

    // Do linear portion of goal
    tf2::Transform goalInBase;
    if (!transformPose(frameId, "base_link", goal, goalInBase)) {
         ROS_WARN("Cannot determine robot pose for linear");
         return;
    }

    tf2::Vector3 linear = goalInBase.getOrigin();
    // we could check for linear.y being within linearTolerance, however
    // if linear.x is large, then that requires a high degree of angular accuracy
    ROS_INFO("Requested distance %f %f", linear.x(), linear.y());

    if (linear.x() > linearTolerance)
        if (!moveLinear(linear.x())) {
            return;
        }
        sleep(localizationLatency);

    // Final rotation as specified in goal
    if (!getTransform("base_link", "map", poseMap)) {
         abortGoal("Cannot determine robot pose for final rotation");
         return;
    }

    getPose(poseMap, x, y, yaw);
    rotate(goalYaw - yaw);

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


// publish visualization

void MoveBasic::drawLine(double x0, double y0, double x1, double y1)
{
    visualization_msgs::Marker line;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.header.frame_id = "/base_link";
    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;
    line.id = 0x42;
    line.ns = "distance";
    line.scale.x = line.scale.y = line.scale.z = 0.01;
    line.pose.position.x = 0;
    line.pose.position.y = 0;
    geometry_msgs::Point gp0, gp1;
    gp0.x = x0;
    gp0.y = y0;
    gp0.z = 0;
    gp1.x = x1;
    gp1.y = y1;
    gp1.z = 0;
    line.points.push_back(gp0);
    line.points.push_back(gp1);

    linePub.publish(line);
}


// Main loop

void MoveBasic::run()
{
    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}


// Rotate relative to current orientation

bool MoveBasic::rotate(double yaw)
{
    ROS_INFO("Requested rotation %f", rad2deg(yaw));

    tf2::Transform poseOdom;
    if (!getTransform("base_link", "odom", poseOdom)) {
         abortGoal("Cannot determine robot pose for rotation");
         return false;
    }

    double x, y, currentYaw;
    getPose(poseOdom, x, y, currentYaw);
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
        tf2::Transform poseOdom;
        if (!getTransform("base_link", "odom", poseOdom)) {
            abortGoal("Cannot determine robot pose for rotation");
            return false;
        }
        getPose(poseOdom, x, y, currentYaw);

        double angleRemaining = requestedYaw - currentYaw;
        normalizeAngle(angleRemaining);

        double speed = std::max(minAngularVelocity,
            std::min(maxAngularVelocity,
              std::sqrt(2.0 * angularAcceleration *
                (std::abs(angleRemaining) - angularTolerance))));

        double velocity = 0;

        if (angleRemaining < 0) {
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
            ROS_INFO("Stopping rotation due to new goal");
            done = true;
            velocity = 0;
        }

        //ROS_INFO("%f %f %f", rad2deg(angleRemaining), angleRemaining, velocity);

        if (std::abs(angleRemaining) < angularTolerance || oscillations > 2) {
            velocity = 0;
            done = true;
            ROS_INFO("Done rotation, error %f degrees", rad2deg(angleRemaining));
        }
        sendCmd(velocity, 0);
    }
    return done;
}

// Move forward specified distance

bool MoveBasic::moveLinear(double requestedDistance)
{
    bool done = false;
    ros::Rate r(50);

    bool waitingForObstacle = false;
    ros::Time obstacleTime;

    tf2::Transform poseOdomInitial;
    if (!getTransform("base_link", "odom", poseOdomInitial)) {
         abortGoal("Cannot determine robot pose for linear");
         return false;
    }

    while (!done && ros::ok()) {
        ros::spinOnce();
        r.sleep();

        tf2::Transform poseOdom;
        if (!getTransform("base_link", "odom", poseOdom)) {
             ROS_WARN("Cannot determine robot pose for linear");
             continue;
        }

        tf2::Vector3 travelled = poseOdomInitial.getOrigin() - poseOdom.getOrigin();
        double distTravelled = travelled.length();;

        double distRemaining = requestedDistance - distTravelled;

        double velocity = std::max(minLinearVelocity,
            std::min(maxLinearVelocity, std::min(
              std::sqrt(2.0 * linearAcceleration * std::abs(distTravelled)),
              std::sqrt(2.0 * linearAcceleration * std::min(obstacleDist,
                std::abs(distRemaining))))));

        // Stop if there is an obstacle in the distance we would travel in
        // 1 second
        bool obstacleDetected = obstacleDist <= velocity * 1.0;
        if (obstacleDetected) {
            velocity = 0;
            if (!waitingForObstacle) {
                ROS_INFO("Pausing for obstacle");
                obstacleTime = ros::Time::now();
                waitingForObstacle = true;
            }
            else {
                ros::Duration waitTime = ros::Time::now() - obstacleTime;
                if (waitTime.toSec() > obstacleWaitLimit) {
                    abortGoal("Aborting due to obstacle");
                    return false;
                }
            }
        }

        if (waitingForObstacle && ! obstacleDetected) {
            ROS_INFO("Resuming after obstacle has gone");
            waitingForObstacle = false;
            // start off again smoothly
            requestedDistance = distRemaining;
            poseOdomInitial = poseOdom;
        }

        if (actionServer->isNewGoalAvailable()) {
            ROS_INFO("Stopping rotation due to new goal");
            done = true;
            velocity = 0;
        }

        //ROS_INFO("%f %f %f", distTravelled, distRemaining, velocity);

        if (distTravelled > requestedDistance - linearTolerance) {
            velocity = 0;
            done = true;
            ROS_INFO("Done linear, error %f meters", distRemaining);
        }
        sendCmd(0, velocity);
    }
    return done;
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "move_basic");
    MoveBasic mb_node;
    mb_node.run();

    return 0;
}
