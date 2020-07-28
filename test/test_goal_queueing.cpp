#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_basic/queued_action_server.h>
#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>

class GoalQueueSuite : public ::testing::Test {
protected:
	virtual void SetUp() {
		resetFlags();
		ros::NodeHandle actionNh("");
		qserv.reset(new actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> (actionNh, "queue_server", boost::bind(&GoalQueueSuite::executeCallback, this, _1))); 
		cli.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("queue_server", true)); // true -> don't need ros::spin() 
		
		qserv->start();
	}

	virtual void TearDown() {
		// Kill the executeCallback if it is running
		// New Scope so that lock_guard destructs and unlocks
		{
			std::lock_guard<std::mutex> lk1(execute_lock);
			finish_executing = true;
			execute_cv.notify_one(); // Releases one waiting thread
		}
		qserv->shutdown();
	}

	// TODO: Making sure it gets called at the right time (flags!) - in its own thread
	void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr &msg) {
		resetFlags();
		got_goal = true;
		received_goal = msg;
		while (true) {
			if (qserv->isPreemptRequested()) {
				goal_preempted = true;
			} else {
				goal_preempted = false;
			}
			if (qserv->isNewGoalAvailable()) {
				next_goal_available = true;
			} else {
				next_goal_available = false;
			}

			// Wait until signalled to end
			std::unique_lock<std::mutex> lk(execute_lock);
			execute_cv.wait(lk, 
				//lambda function to wait on our bool variables 
				[this](){return finish_executing || resume_executing;} // blocks only if lambda returns false
			);
			// We were requested to stop, so we stop
			if (finish_executing) {
				break;
			}
		}
	
		// Signal that we are done here 
		std::lock_guard<std::mutex> lk(execute_done_lock);
		execute_done = true;
		execute_done_cv.notify_all(); // Releases all waiting thread
	}

	// Helper function to signal the executeCallback to end
	void finishExecuting() {
		// New Scope so that lock_guard destructs and unlocks
		{
			std::lock_guard<std::mutex> lk1(execute_lock);
			finish_executing = true;
			execute_cv.notify_one(); //unlocks lambda waiting function in ExecuteCallback
		}

		// Wait for execute callback to actually finish
		std::unique_lock<std::mutex> lk2(execute_done_lock);
		execute_done_cv.wait(lk2, [this]() {return execute_done;}); // at the end of ExecuteCallback
	}
	
	// Helper function to signal the executeCallback to resume
	// This is useful for seeing if the callback code sees a preempt
	void resumeExecuting() {
		std::lock_guard<std::mutex> lk(execute_lock);
		resume_executing = true;
		execute_cv.notify_one();
	}

	void resetFlags() {
		got_goal = false;
		goal_preempted = false;
		next_goal_available = false;
		received_goal = nullptr;
		
		// Signal flags
		finish_executing = false;
		resume_executing = false;
		execute_done = false;
	}

	// Flags for assertions
	bool got_goal;
	bool goal_preempted;
	bool next_goal_available;
	move_base_msgs::MoveBaseGoalConstPtr received_goal;

	// Signaling variables
	bool finish_executing;
	bool resume_executing;
	bool execute_done;
	std::mutex execute_lock;
	std::mutex execute_done_lock;
	std::condition_variable execute_cv;
	std::condition_variable execute_done_cv;

	std::unique_ptr<actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction>> qserv;
	std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> cli;
};

// IMPORTANT: Time delays and ros::spinOnce() sequence should stay the same, otherwise, strange things happen
TEST_F(GoalQueueSuite, establishDuplex) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(0.5).sleep(); // TODO: Make this disappear 
	finishExecuting();
	ASSERT_TRUE(got_goal);
	ASSERT_FALSE(goal_preempted);
	ASSERT_FALSE(next_goal_available);
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);
}

TEST_F(GoalQueueSuite, addGoalWhileExecuting) { 
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;
	
	// First goal
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(0.5).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(qserv->isActive());
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);

	// Cancelling the first goal - PITFALL
	resumeExecuting();
	cli->cancelGoal();
	ros::Duration(1.0).sleep(); 
	ros::spinOnce(); 
	ASSERT_TRUE(goal_preempted);
	// Finish the preempted goal
	finishExecuting();
	
	// "Second" goal
	goal.target_pose.pose.position.x = 7.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(0.5).sleep(); // TODO: Make this disappear
	ASSERT_TRUE(qserv->isActive());
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(7.0, received_goal->target_pose.pose.position.x); 
	
	// Third goal
	goal.target_pose.pose.position.x = 13.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(0.5).sleep(); // TODO: Make this disappear
	resumeExecuting();
	// Make sure that the "second" goal is still executing, but a new goal is seen
	ASSERT_TRUE(qserv->isActive()); 
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(7.0, received_goal->target_pose.pose.position.x);
	ASSERT_FALSE(goal_preempted);
	ASSERT_TRUE(next_goal_available); // Fails when changing other code 
	// Finish the "second" goal, then the 3nd goal should start executing
	finishExecuting();
	ros::spinOnce(); 
	ros::Duration(0.5).sleep(); // TODO: Make this disappear
	ASSERT_TRUE(qserv->isActive()); 
	ASSERT_TRUE(got_goal);
	ASSERT_FALSE(goal_preempted);
	ASSERT_FALSE(next_goal_available);
	ASSERT_EQ(13.0, received_goal->target_pose.pose.position.x);
	finishExecuting(); // Finish the 3nd goal
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
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(got_goal);
	ASSERT_TRUE(qserv->isActive());
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);

	resumeExecuting();
	cli->cancelGoal();
	ros::Duration(1.0).sleep(); 
	ros::spinOnce(); 
	ASSERT_TRUE(goal_preempted);	
	finishExecuting(); // Finish the goal
	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); 
	ASSERT_FALSE(qserv->isActive());

	// Two goals -> Cancel current goal -> Start executing the second
	// First goal
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(0.5).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(qserv->isActive());
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);

	// Cancelling the first goal - PITFALL
	resumeExecuting();
	cli->cancelGoal();
	ros::Duration(1.0).sleep(); 
	ros::spinOnce(); 
	ASSERT_TRUE(goal_preempted);
	// Finish the preempted goal
	finishExecuting(); // Finish 1st goal
	
	// "Second" goal
	goal.target_pose.pose.position.x = 7.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(0.5).sleep(); // TODO: Make this disappear
	ASSERT_TRUE(qserv->isActive());
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(7.0, received_goal->target_pose.pose.position.x); // call FINISH!
	finishExecuting(); // Finish 2nd goal
	
//	- if a cancel request is received for the current goal, set it as preempted (DONE)
//	- if there another goal, start executing it
//	- if no goal, stop (DONE)
}


// TODO: How can I cancel the second goal??? CANCEL REQUEST CANCELS THE LAST GOAL SENT
TEST_F(GoalQueueSuite, goalCancelling) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;

	cli->sendGoal(goal);
	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); // TODO: Make this disappear 
	ASSERT_TRUE(got_goal);
	ASSERT_TRUE(qserv->isActive());
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);

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
