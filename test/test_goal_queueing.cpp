#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_basic/queued_action_server.h>
#include <queue>

class GoalQueueSuite : public ::testing::Test {
protected:
	virtual void SetUp() {}
	virtual void TearDown() {}

	//SHARED POINTER
	std::shared_ptr<actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction>> qserv(new  actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction>>)
	qserv->

	bool got_goal1;
	bool got_goal2;
	bool goal_preempted;
	move_base_msgs::MoveBaseGoalConstPtr current_goal;
	move_base_msgs::MoveBaseGoalConstPtr next_goal;
	ros::NodeHandle nh;
	actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> qserv;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> cli;	

	// TODO: Sharing between objects? (Isolation!)
	GoalQueueSuite() : got_goal1(false),
			   got_goal2(false),
			   goal_preempted(false),
			   current_goal(nullptr),
			   next_goal(nullptr),
			   qserv(nh, "queue_server", boost::bind(&GoalQueueSuite::executeCallback, this, _1)),		   
			   cli("queue_server", true) // true -> don't need ros::spin(), required separate thread 
	{
		qserv.start();
	}
	// ADD GOALCALLBACK

	~GoalQueueSuite() {
		qserv.shutdown();
	}

	// TODO: Making sure it gets called at the right time (flags!)
	void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr &msg){
		if (current_goal == nullptr) {
			got_goal1 = true;
			current_goal = msg;
		}
		else if (next_goal == nullptr) {
			got_goal2 = true;
			next_goal = msg;
		}
		else {
			current_goal = next_goal;
			goal_preempted = true; 
			next_goal = msg;
		} 
	}
};


TEST_F(GoalQueueSuite, establishDuplex) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	cli.sendGoal(goal);

	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 

	ASSERT_TRUE(got_goal1);
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);
}


TEST_F(GoalQueueSuite, queueAdding) { // Probably now I should be sending to goalCallback?
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;

	cli.sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 

	ASSERT_TRUE(got_goal1);
	
	cli.sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 

	ASSERT_TRUE(got_goal2);
	
	cli.sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 

	ASSERT_TRUE(goal_preempted);
	/* 
	- if another goal is received add it to the queue
	- if the queue full, set the current goal as preempted
	- start executing the next goal in queue (the one after the preempted)
	*/	

	//ASSERT_TRUE(qserv.isPreemptRequested());
}


TEST_F(GoalQueueSuite, goalPreempting) {
	/*
	- if a cancel request is received for the current goal, set it as preempted
	- if there another goal, start executing it
	- if no goal, stop
	*/
}

TEST_F(GoalQueueSuite, goalCancelling) {
	/*
	- if a cancel request on the "next_goal" received, remove it from the queue and set it as cancelled
	*/
}

// Two more TEST_F missing and a pitfall

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_queueing_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
