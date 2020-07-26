#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_basic/queued_action_server.h>
#include <queue>
#include <memory>

class GoalQueueSuite : public ::testing::Test {
protected:
	virtual void SetUp() {
		got_goal1 = false;
		got_goal2 = false;
		goal_preempted = false;
		current_goal = nullptr;
		next_goal = nullptr;
		ros::NodeHandle actionNh("");
		qserv.reset(new actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> (actionNh, "queue_server", boost::bind(&GoalQueueSuite::executeCallback, this, _1))); 
		cli.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("queue_server", true)); // true -> don't need ros::spin() 
		qserv->start();
	}

	virtual void TearDown() {
		qserv->shutdown();
	}

	// TODO: Making sure it gets called at the right time (flags!) - in its own thread
	void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr &msg){
		if (current_goal == nullptr) {
			got_goal1 = true;
			current_goal = msg;
			ros::Duration(3.0).sleep();
		}
		else if (next_goal == nullptr) {
			got_goal2 = true;
			next_goal = msg;
			ros::Duration(3.0).sleep();
		}
		else {
			goal_preempted = true; 
			current_goal = next_goal;
			next_goal = msg;
			ros::Duration(3.0).sleep();
		} 
	}
	
	bool got_goal1;
	bool got_goal2;
	bool goal_preempted;
	move_base_msgs::MoveBaseGoalConstPtr current_goal;
	move_base_msgs::MoveBaseGoalConstPtr next_goal;
	std::unique_ptr<actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction>> qserv;
	std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> cli;
};

// IMPORTANT: Time delays and ros::spinOnce() sequence should stay the same, otherwise, strange things happen
TEST_F(GoalQueueSuite, establishDuplex) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ASSERT_TRUE(qserv->isNewGoalAvailable()); // If moved, strange things happen
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(got_goal1);
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);
	// - start executing goal as soon as one is available (DONE)
}

//Cancelling done on the client side
//Because Queue full, current goal should be preempted by Actionserver internally
// TODO: Achieve execution of multiple goals at a time, HOW????
TEST_F(GoalQueueSuite, queueAdding) { 
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	
	// First goal
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ASSERT_TRUE(qserv->isNewGoalAvailable()); // If moved, strange things happen
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(got_goal1);
	ASSERT_TRUE(qserv->isActive());
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);

	// Second goal - TODO: Troubles with the second goal
	goal.target_pose.pose.position.x = 7.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ASSERT_TRUE(qserv->isNewGoalAvailable()); // If moved, strange things happen
	ros::Duration(2.0).sleep(); // TODO: Make this disappear 
	ros::spinOnce(); 
	ASSERT_TRUE(got_goal2); // Sometimes fails, run it again
	ASSERT_TRUE(qserv->isActive()); 
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);
	ASSERT_EQ(7.0, next_goal->target_pose.pose.position.x);
	
/*
	// Third goal
	goal.target_pose.pose.position.x = 13.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 
*/

/*
	- if another goal is received add it to the queue (DONE) 
	- if the queue full, set the current goal as preempted - explicitly called! - so cancelling the current goal and start executing the next one
	- start executing the next goal in queue (the one after the preempted)
*/
}


TEST_F(GoalQueueSuite, goalPreempting) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	
	// One goal -> Cancel request -> Stop
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ASSERT_TRUE(qserv->isNewGoalAvailable()); // If moved, strange things happen
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(got_goal1);
	ASSERT_TRUE(qserv->isActive());
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);

	cli->cancelGoal();
	ros::Duration(1.0).sleep(); 
	ros::spinOnce(); 
	ASSERT_TRUE(qserv->isPreemptRequested());	
	ros::Duration(1.0).sleep(); 
	ASSERT_FALSE(qserv->isActive());

	//TODO: Two goals -> Cancel request(current goal) -> Execute next goal
	
	/*
	- if a cancel request is received for the current goal, set it as preempted (DONE)
	- if there another goal, start executing it
	- if no goal, stop (DONE)
	*/
}

// TODO: How can I cancel the second goal??? CANCEL REQUEST CANCELS THE LAST GOAL SENT
TEST_F(GoalQueueSuite, goalCancelling) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;

	cli->sendGoal(goal);
	ros::spinOnce(); 
	ASSERT_TRUE(qserv->isNewGoalAvailable()); // If moved, strange things happen
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(got_goal1);
	ASSERT_TRUE(qserv->isActive());
	ASSERT_EQ(3.0, current_goal->target_pose.pose.position.x);

	/*
	- if a cancel request on the "next_goal" received, remove it from the queue and set it as cancelled
	- TODO: How to check next goal is executing? It becomes current, so isActive()
	*/
}

// Two more TEST_F missing and a pitfall

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_queueing_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
