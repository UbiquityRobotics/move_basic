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

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <list>
#include <string>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBasic {
  private:
    ros::Subscriber goalSub;
    ros::Publisher cmdPub;
    ros::Publisher pathPub;

//    MoveBaseActionServer actionServer;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;

    double angularVelocity;
    double angularTolerance;

    double linearVelocity;
    double linearTolerance;

    tf2::Transform goalOdom;
    bool haveGoal;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void executeAction(const move_base_msgs::MoveBaseGoalConstPtr& goal,
                       MoveBaseActionServer* as);

    void sendCmd(double angular, double linear);

    bool getTransform(const std::string& from, const std::string& to,
                      tf2::Transform& tf);

    bool handleRotation();
    bool handleLinear();

  public:
    MoveBasic();

    void run();

    bool moveLinear(double requestedDistance);
    bool rotateTo(double requestedYaw);
    bool rotateRel(double yaw);
};


// Radians to degrees

static double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
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
                                           listener(tfBuffer), haveGoal(false)
{
    ros::NodeHandle nh("~");

    nh.param<double>("angular_velocity", angularVelocity, 0.3);
    nh.param<double>("angular_tolerance", angularTolerance, 0.01);

    nh.param<double>("linear_velocity", linearVelocity, 0.4);
    nh.param<double>("linear_tolerance", linearTolerance, 0.01);

    cmdPub = ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));

    pathPub = ros::Publisher(nh.advertise<nav_msgs::Path>("/plan", 1));

    goalSub = nh.subscribe("/move_base_simple/goal", 1,
                            &MoveBasic::goalCallback, this);

    MoveBaseActionServer* actionServer = new MoveBaseActionServer(nh,
        "move_basic", boost::bind(&MoveBasic::executeAction, this, _1, actionServer), false);


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


void MoveBasic::executeAction(const move_base_msgs::MoveBaseGoalConstPtr& goal,
                              MoveBaseActionServer* as)
{
}


// Called when a simple goal message is received

void MoveBasic::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf2::Transform goal;
    tf2::fromMsg(msg->pose, goal);
    std::string frameId = msg->header.frame_id;

    double x, y, yaw;
    getPose(goal, x, y, yaw);
    ROS_INFO("Received goal %f %f %f", x, y, rad2deg(yaw));

    tf2::Transform tfMapOdom;
    if (!getTransform(frameId, "odom", tfMapOdom)) {
        ROS_WARN("Cannot determine robot pose");
        return;
    }
    goalOdom = tfMapOdom * goal;

    getPose(goalOdom, x, y, yaw);
    ROS_INFO("Goal in odom  %f %f %f", x, y, rad2deg(yaw));
    haveGoal = true;

    nav_msgs::Path path;
    geometry_msgs::PoseStamped p0, p1;
    path.header.frame_id = "odom";
    p0.pose.position.x = x;
    p0.pose.position.y = y;
    path.poses.push_back(p0);

    tf2::Transform poseOdom;
    if (!getTransform("base_link", "odom", poseOdom)) {
         ROS_WARN("Cannot determine robot pose for rotation");
         return;
    }
    getPose(poseOdom, x, y, yaw);
    p1.pose.position.x = x;
    p1.pose.position.y = y;
    path.poses.push_back(p1);

    pathPub.publish(path);
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
        r.sleep();

        if (haveGoal) {
            haveGoal = false;
            if (!handleRotation()) {
                continue;
            }
            if (!handleLinear()) {
                continue;
            }
            double x, y, yaw;
            getPose(goalOdom, x, y, yaw);
            rotateTo(yaw);
        }
    }
}


// Do angular part of goal

bool MoveBasic::handleRotation()
{
    tf2::Transform poseOdom;
    if (!getTransform("base_link", "odom", poseOdom)) {
         ROS_WARN("Cannot determine robot pose for rotation");
         return false;
    }

    tf2::Vector3 linear = goalOdom.getOrigin() - poseOdom.getOrigin();
    double requestedYaw = atan2(linear.y(), linear.x());
 
    return rotateTo(requestedYaw);
}


// Rotate relative to current orientation

bool MoveBasic::rotateRel(double yaw)
{
    tf2::Transform poseOdom;
    if (!getTransform("base_link", "odom", poseOdom)) {
         ROS_WARN("Cannot determine robot pose for rotation");
         return false;
    }

    double x, y, currentYaw;
    getPose(poseOdom, x, y, currentYaw);
    double requestedYaw = currentYaw + yaw;
    normalizeAngle(requestedYaw);

    return rotateTo(requestedYaw);
}


// Rotate to specified orientation (in radians)

bool MoveBasic::rotateTo(double requestedYaw)
{
    bool done = false;
    ros::Rate r(50);

    while (!done && ros::ok()) {
        ros::spinOnce();
        r.sleep();

        double x, y, currentYaw;
        tf2::Transform poseOdom;
        if (!getTransform("base_link", "odom", poseOdom)) {
             ROS_WARN("Cannot determine robot pose for rotation");
             return false;
        }
        getPose(poseOdom, x, y, currentYaw);

        double demand = requestedYaw - currentYaw;
        normalizeAngle(demand);

        double velocity = 0;
        if (demand < 0) {
            velocity = -angularVelocity;
        }
        else {
            velocity = angularVelocity;
        }

        if (haveGoal) { // new goal received
            ROS_INFO("Stopping rotation due to new goal");
            done = true;
            velocity = 0;
        }
//        ROS_INFO("Demand %f %f %f", rad2deg(demand), demand, velocity);
        if (std::abs(demand) < angularTolerance) {
            velocity = 0;
            done = true;
            ROS_INFO("Done rotation, error %f radians %f degrees", demand, rad2deg(demand));
        }
        sendCmd(velocity, 0);
    }
    return done;
}


// Do linear part of goal

bool MoveBasic::handleLinear()
{
    bool done = false;

    tf2::Transform poseOdomInitial;
    if (!getTransform("base_link", "odom", poseOdomInitial)) {
         ROS_WARN("Cannot determine robot pose for linrar");
         return false;
    }

    tf2::Vector3 linear = goalOdom.getOrigin() - poseOdomInitial.getOrigin();
    double requestedDistance = linear.length();;
    ROS_INFO("Requested distance %f", requestedDistance);

    return moveLinear(requestedDistance);
}


// Move foreward specified distance

bool MoveBasic::moveLinear(double requestedDistance)
{
    bool done = false;
    ros::Rate r(50);

    tf2::Transform poseOdomInitial;
    if (!getTransform("base_link", "odom", poseOdomInitial)) {
         ROS_WARN("Cannot determine robot pose for linrar");
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

        double demand = requestedDistance - distTravelled;
        double velocity = linearVelocity;

        if (haveGoal) { // new goal received
            ROS_INFO("Stopping rotation due to new goal");
            done = true;
            velocity = 0;
        }

//        ROS_INFO("Demand %f %f", requestedDist-distTravelled, velocity);

        if (distTravelled > requestedDistance - linearTolerance) {
            velocity = 0;
            done = true;
            ROS_INFO("Done linear, error %f meters", demand);
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
