#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_basic/queued_action_server.h>

class GoalQueueSuite : public ::testing::Test {
protected:
	virtual void SetUp() {
		// variables here?
	//initialization, declaration!!!
		got_goal = false;
	}
	virtual void TearDown() {}
public:
	move_base_msgs::MoveBaseGoalConstPtr current_goal;
	bool got_goal = true;
	void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr &msg){
		got_goal = false;
		current_goal = msg;
	}
};


TEST_F(GoalQueueSuite, establishDuplex) {
	// TODO:  Make this more beautiful
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> cli("queue_server", true); // true -> don't need ros::spin(), required separate thread
	actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> qserv(nh, "queue_server", boost::bind(&GoalQueueSuite::executeCallback, this, _1));
	qserv.start();
	
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	cli.sendGoal(goal);
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::Duration(5.0).sleep();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();

	ASSERT_TRUE(got_goal);
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);
	
	// ROS API
	//ASSERT_TRUE(cli.getState() == actionlib::SimpleClientGoalState::ACTIVE ? true : false);
	//actionlib::SimpleClientGoalState state = cli.getState();

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_queueing_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
