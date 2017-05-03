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

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>

#include <list>
#include <string>

using namespace std;

static double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}


class MoveBasic {
  private:
    ros::Subscriber goal_sub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;
    tf2::Transform goal;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  public:

    MoveBasic(ros::NodeHandle &nh);
};


void MoveBasic::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf2::fromMsg(msg->pose, goal);
    tf2::Vector3 transGoal = goal.getOrigin();
    double rollGoal, pitchGoal, yawGoal;
    goal.getBasis().getRPY(rollGoal, pitchGoal, yawGoal);

    ROS_INFO("Received goal  %f %f %f", 
             transGoal.x(), transGoal.y(), rad2deg(yawGoal));

    geometry_msgs::TransformStamped transform;
    tf2::Transform pose;

    try {
        transform = tfBuffer.lookupTransform("base_link", "map",
                                             ros::Time(0));

        tf2::fromMsg(transform.transform, pose);
    }
    catch (tf2::TransformException &ex) {
         ROS_WARN("Cannot determine robot pose: %s", ex.what());
         return;
    }

    tf2::Vector3 transPose = pose.getOrigin();
    double rollPose, pitchPose, yawPose;
    pose.getBasis().getRPY(rollPose, pitchPose, yawPose);
 
    ROS_INFO("Current position %f %f %f", 
             transPose.x(), transPose.y(), rad2deg(yawPose));

    tf2::Vector3 linear = transGoal - transPose;
    double angle = atan2(linear.y(), linear.x());

    ROS_INFO("Rotation %f", rad2deg(angle));
    ROS_INFO("Distance %f", linear.length());
}

MoveBasic::MoveBasic(ros::NodeHandle &nh): tfBuffer(ros::Duration(30.0)),
                                           listener(tfBuffer)
{
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, 
                          &MoveBasic::goalCallback, this); 

    ROS_INFO("Move Basic ready");
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "move_basic");
    ros::NodeHandle nh("~");
    MoveBasic node(nh);

    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce(); 
        r.sleep();
    }

    return 0;
}
