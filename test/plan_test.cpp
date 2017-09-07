/*
 Test move_basic by sending a goal and looking at the planned path
*/

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PlanTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  { 
    plan_sub = nh.subscribe("/plan", 1, &PlanTest::plan_callback, this);
    cmd_sub = nh.subscribe("/cmd_vel", 1, &PlanTest::cmd_callback, this);
    result_sub = nh.subscribe("/move_base/result", 1,
                              &PlanTest::result_callback, this);
    
    publish_global_pose = false;
    got_plan = false;
    set_odom(tf2::Transform(I));
    set_global_pose(tf2::Transform(I));

    tf_thread = boost::thread(boost::bind(&PlanTest::tf_thread_func, this));
    spinner = new ros::AsyncSpinner(3);
    spinner->start();
  }

  virtual void TearDown()
  {
    spinner->stop();
    delete spinner;
  }
 
  void send_goal(const std::string frame, const tf2::Transform& goal)
  {
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal_;

    goal_.target_pose.header.frame_id = frame;
    goal_.target_pose.header.stamp = ros::Time::now();

    tf2::Quaternion q = goal.getRotation();
    goal_.target_pose.pose.orientation.x = q.x();
    goal_.target_pose.pose.orientation.y = q.y();
    goal_.target_pose.pose.orientation.z = q.z();
    goal_.target_pose.pose.orientation.w = q.w();
    tf2::Vector3 t = goal.getOrigin();
    goal_.target_pose.pose.position.x = t.x();
    goal_.target_pose.pose.position.y = t.y();
    goal_.target_pose.pose.position.z = t.z();

    ROS_INFO("Sending goal");
    ac.sendGoal(goal_);
  }

  void result_callback(const move_base_msgs::MoveBaseActionResultConstPtr& result)
  {
    done = true;
  }

  void cmd_callback(const geometry_msgs::Twist& msg)
  {
    got_cmd = true;
  }

  void plan_callback(const nav_msgs::Path& msg)
  {
    got_plan = true;
    plan = msg;
  }

  void set_odom(tf2::Transform tf)
  {
    odom_tf.transform = tf2::toMsg(tf);
  }

  void set_global_pose(tf2::Transform tf)
  {
    global_tf.transform = tf2::toMsg(tf);
  }

  ros::NodeHandle nh;

  ros::Subscriber plan_sub;
  ros::Subscriber cmd_sub;
  ros::Subscriber result_sub;

  volatile bool publish_global_pose;
  volatile bool got_plan;
  volatile bool got_cmd;
  volatile bool done;
  geometry_msgs::TransformStamped odom_tf;
  geometry_msgs::TransformStamped global_tf;
  nav_msgs::Path plan;

  tf2_ros::TransformBroadcaster broadcaster;
  tf2::Quaternion I = tf2::Quaternion::getIdentity();

  boost::thread tf_thread;
  boost::thread spin_thread;
  ros::AsyncSpinner* spinner;

public:
  void tf_thread_func()
  {
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    global_tf.header.frame_id = "map";
    global_tf.child_frame_id = "odom";

    ros::Rate rate(10);

    while(nh.ok()) {
      odom_tf.header.stamp = ros::Time::now();
      broadcaster.sendTransform(odom_tf);
      if (publish_global_pose) {
        global_tf.header.stamp = ros::Time::now();
        broadcaster.sendTransform(global_tf);
      }
      rate.sleep();
    }
  }
};

TEST_F(PlanTest, forward_1_m)
{
  ros::Rate rate(5);
  tf2::Vector3 one_meter(1, 0, 0);

  done = false;
  got_cmd = false;
  bool moved = false;
  publish_global_pose = false;

  set_odom(tf2::Transform(I));

  rate.sleep();

  send_goal("base_link", tf2::Transform(I, one_meter));

  while (nh.ok() && !done) {
    if (got_cmd && !moved) {
       // pretend we did it
       set_odom(tf2::Transform(I, one_meter));
       moved = true;
    }
    rate.sleep();
  }
  ASSERT_EQ(true, got_plan);
  ASSERT_EQ("odom", plan.header.frame_id);
}

TEST_F(PlanTest, global_1_m)
{
  ros::Rate rate(5);
  tf2::Vector3 one_meter(1, 0, 0);

  done = false;
  got_cmd = false;
  bool moved = false;
  publish_global_pose = true;

  set_odom(tf2::Transform(I));
  set_global_pose(tf2::Transform(I));

  rate.sleep();

  send_goal("map", tf2::Transform(I, one_meter));

  while (nh.ok() && !done) {
    if (got_cmd && !moved) {
       // pretend we did it
       set_odom(tf2::Transform(I, one_meter));
       moved = true;
    }
    rate.sleep();
  }
  ASSERT_EQ(true, got_plan);
  ASSERT_EQ("map", plan.header.frame_id);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PlanTest");
  return RUN_ALL_TESTS();
}
