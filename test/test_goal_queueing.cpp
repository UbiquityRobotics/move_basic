#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <gtest/gtest.h>


class GoalQueueSuite : public ::testing::Test {
protected:
	virtual void SetUp() {}
	virtual void TearDown() {}
};


TEST_F(GoalQueueSuite, executionDelay) {	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> cli("move_base", true); // true -> don't need ros::spin(), required separate thread
	while(!cli.waitForServer(ros::Duration(5.0))){ROS_INFO("Waiting for the move_base action server to come up");}
	if (cli.getState() == actionlib::SimpleClientGoalState::PENDING) { // PENDING = 0
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "base_link";	
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = 3.0;
		goal.target_pose.pose.position.y = 4.0; 
		goal.target_pose.pose.orientation.x = 1.0;	
		goal.target_pose.pose.orientation.y = 1.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;
		ROS_DEBUG("Goal created.");	
		cli.sendGoal(goal);
		EXPECT_EQ(cli.getState(), actionlib::SimpleClientGoalState::ACTIVE); // Get the state information about the current goal
	}
}

// Five more TEST_F and a PITFALL missing

int main(int argc, char** argv) {
	ros::init(argc, argv, "goal_queueing_test");
	testing::InitGoogleTest(&argc, argv);
	//return (RUN_ALL_TESTS() ? "Testing failed." : "Testing successful.");
	return RUN_ALL_TESTS();
}



