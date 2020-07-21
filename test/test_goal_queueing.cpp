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
	}
	virtual void TearDown() {}
public:
	GoalQueueSuite() {}
	void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr &msg){
		ROS_INFO("Working.");
	}
};


TEST_F(GoalQueueSuite, executionDelay) {
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> cli("queue_server", true); // true -> don't need ros::spin(), required separate thread
	actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> qserv(nh, "queue_server", boost::bind(&GoalQueueSuite::executeCallback, this, _1));	

	// defining a goal
	move_base_msgs::MoveBaseGoal goal; // goal type not okay?
	goal.target_pose.pose.position.x = 3.0;
	goal.target_pose.pose.position.y = 4.0; 
	goal.target_pose.pose.orientation.x = 1.0;	
	goal.target_pose.pose.orientation.y = 1.0;
	goal.target_pose.pose.orientation.z = 1.0;
	goal.target_pose.pose.orientation.w = 1.0;
	qserv.goalCallback(goal);
	/*
	while(!cli.waitForServer(ros::Duration(5.0))){ROS_INFO("Waiting for the move_base action server to come up");}
	if (cli.getState() == actionlib::SimpleClientGoalState::PENDING) { // PENDING = 0
	actionlib::SimpleClientGoalState state = cli.getState();
    	ROS_INFO("Action finished: %s",state.toString().c_str());
	*/
	}


int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_queueing_test");
    //ros::start();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
