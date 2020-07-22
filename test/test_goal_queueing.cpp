#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_basic/queued_action_server.h>
#include <queue>

class GoalQueueSuite : public ::testing::Test {
protected:
	virtual void SetUp() {
		got_goal = false;
		goal_preempted = false;
		current_goal = NULL;
		next_goal = NULL;
	}
	virtual void TearDown() {}
public:	
	bool got_goal;
	bool goal_preempted;
	move_base_msgs::MoveBaseGoalConstPtr current_goal;
	move_base_msgs::MoveBaseGoalConstPtr next_goal;
	//std::queue<move_base_msgs::MoveBaseGoal> qgoals;	
	ros::NodeHandle nh;
	actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> qserv;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> cli;	

	GoalQueueSuite() : qserv(nh, "queue_server", boost::bind(&GoalQueueSuite::executeCallback, this, _1)),
			   cli("queue_server", true) // true -> don't need ros::spin(), required separate thread 
	{
		qserv.start();
	}

	~GoalQueueSuite() {
		qserv.shutdown();
	}

	void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr &msg){
		got_goal = true;
		if (current_goal == NULL) current_goal = msg;
		else if (next_goal == NULL) next_goal = msg;
		else {
			qserv.setPreempted();
			goal_preempted = true; //needed?
		} 
		// TODO: Queueing behaviour
		//if (qgoals.size() > 2) { // start index?
		//	qgoals.push(*current_goal);
		//}		
	}
};


TEST_F(GoalQueueSuite, establishDuplex) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	cli.sendGoal(goal);

	ros::spinOnce();
	ros::Duration(1.0).sleep();

	ASSERT_TRUE(got_goal);
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);
	
	// ROS API
}


TEST_F(GoalQueueSuite, queueAdding) { // probably now I should be sending to goalCallback
	/* 
	- if another goal is received add it to the queue
	- if the queue full, set the current goal as preempted
	- start executing the next goal in queue (the one after the preempted)
	- 
	*/	

	//ASSERT_TRUE(qserv.isPreemptRequested());
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_queueing_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
