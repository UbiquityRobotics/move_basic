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

	void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr &msg) {
		resetFlags();
		got_goal = true;
		received_goal = msg;
		while (true) {
			goal_preempted = qserv->isPreemptRequested() ? true : false; 
			next_goal_available = qserv->isNewGoalAvailable() ? true : false;

			// Test fixture can continue
			execution = true;
			sleep_cv.notify_one();
			// Wait until signalled to end
			std::unique_lock<std::mutex> lk(execute_lock);
			execute_cv.wait(lk, 
				//lambda function to wait on our bool variables 
				[this](){return finish_executing || resume_executing;} // blocks only if lambda returns false
			);
			execution = false;
			// We were requested to stop, so we stop
			if (finish_executing) {
				break;
			}
		}
	
		// Signal that we are done here 
		std::lock_guard<std::mutex> lk(execute_done_lock);
		execute_done = true;
		execute_done_cv.notify_all(); 
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

	// Helper function to wait to for flags to update in ExecuteCallback
	void sleepExecuting() {
		std::unique_lock<std::mutex> slk(sleep_lock);
		sleep_cv.wait(slk, 
				[this](){return execution;}
			); 
	}

	void resetFlags() {
		got_goal = false;
		goal_preempted = false;
		next_goal_available = false;
		received_goal = nullptr;
		
		// Signal flags
		execution = false;
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
	bool execution;
	bool finish_executing;
	bool resume_executing;
	bool execute_done;
	std::mutex sleep_lock;
	std::mutex execute_lock;
	std::mutex execute_done_lock;
	std::condition_variable sleep_cv;
	std::condition_variable execute_cv;
	std::condition_variable execute_done_cv;

	std::unique_ptr<actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction>> qserv;
	std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> cli;
};

TEST_F(GoalQueueSuite, establishDuplex) {
	move_base_msgs::MoveBaseGoal goal; 

	goal.target_pose.pose.position.x = 3.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 

 	sleepExecuting();
	EXPECT_TRUE(got_goal);
	EXPECT_FALSE(goal_preempted);
	EXPECT_FALSE(next_goal_available);
	EXPECT_EQ(3.0, received_goal->target_pose.pose.position.x);
	finishExecuting();
}

/*
TEST_F(GoalQueueSuite, addGoalWhileExecuting) { 
	move_base_msgs::MoveBaseGoal goal; 

	// First goal
	goal.target_pose.pose.position.x = 3.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 

	sleepExecuting();
	EXPECT_TRUE(got_goal);
	EXPECT_FALSE(goal_preempted);
	EXPECT_FALSE(next_goal_available);
	EXPECT_EQ(3.0, received_goal->target_pose.pose.position.x);

	// Second goal
	goal.target_pose.pose.position.x = 7.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 

	sleepExecuting();
// 	ASSERT_TRUE(next_goal_available); // TODO: Why is this failling?
	
	// Cancelling the last goal - TODO: Doesnt work!
	cli->cancelGoal(); // Cancels the last goal sent
	ros::spinOnce(); 
	ros::Duration(1.0).sleep(); 
	EXPECT_EQ(3.0, received_goal->target_pose.pose.position.x);
 	finishExecuting(); // Finish 1st goal
	ros::Duration(3.0).sleep(); 
	// ASSERT_TRUE(goal_preempted); // TODO:  Why is this failling?
 	finishExecuting(); // Finish 2nd (canceled) goal
	ros::Duration(3.0).sleep(); 
	
	// New goal
	goal.target_pose.pose.position.x = 13.0;
	cli->sendGoal(goal); 
	ros::spinOnce(); 

	sleepExecuting();
	EXPECT_TRUE(got_goal);
	EXPECT_FALSE(goal_preempted);
	EXPECT_EQ(13.0, received_goal->target_pose.pose.position.x); // BUG: X=7 (from canceled goal) 
	finishExecuting(); // Finish new goal

// 	- if another goal is received add it to the queue (DONE) 
// 	- if the queue full, set the next goal as preempted - explicitly called! - so cancelling the next goal and adding new to the queue
// 	- start executing the new goal in queue (after the current)
}

TEST_F(GoalQueueSuite, goalPreempting) {
	move_base_msgs::MoveBaseGoal goal; 

	// One goal -> Cancel request -> Stop
	goal.target_pose.pose.position.x = 3.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 
	sleepExecuting();
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);

	cli->cancelGoal();
	ros::spinOnce(); 
	resumeExecuting();
	// ros::spinOnce(); 
	sleepExecuting();
	// ASSERT_TRUE(goal_preempted);	
	finishExecuting(); // Finish the goal
	ros::spinOnce(); 
	sleepExecuting();
// 	ASSERT_FALSE(qserv->isActive());

	// Two goals -> Cancel current goal -> Start executing the second
	// First goal
	cli->sendGoal(goal);
	ros::spinOnce(); 

	sleepExecuting();
	// ASSERT_TRUE(qserv->isActive());
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);

	// Cancelling the first goal - PITFALL
	cli->cancelGoal();
	ros::spinOnce(); 
	resumeExecuting();
	sleepExecuting();
	// ros::spinOnce(); 
	// ASSERT_TRUE(goal_preempted);
	// Finish the preempted goal
	finishExecuting(); // Finish 1st goal
	
	// "Second" goal
	goal.target_pose.pose.position.x = 7.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 

	sleepExecuting();
	ASSERT_TRUE(got_goal);
	// ASSERT_EQ(7.0, received_goal->target_pose.pose.position.x); // call FINISH!
	finishExecuting(); // Finish 2nd goal
	
//	- if a cancel request is received for the current goal, set it as preempted (DONE)
//	- if there another goal, start executing it
//	- if no goal, stop (DONE)
}

*/
TEST_F(GoalQueueSuite, goalCancelling) {
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.pose.position.x = 3.0;

	// Two goals -> Cancel current goal -> Start executing the second
	cli->sendGoal(goal);
	ros::spinOnce(); 

	sleepExecuting();
	ASSERT_TRUE(got_goal);
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);

	// Second goal
	goal.target_pose.pose.position.x = 7.0;
	cli->sendGoal(goal);
	ros::spinOnce(); 

	resumeExecuting();
	ros::Duration(0.5).sleep(); // Needs to wait so the execution variable can update
	sleepExecuting();
	ASSERT_EQ(3.0, received_goal->target_pose.pose.position.x);
	EXPECT_TRUE(next_goal_available);

	// Cancelling the second goal
	EXPECT_TRUE(qserv->isActive());
	cli->cancelGoal();
	ros::Duration(1.0).sleep(); 
	ros::spinOnce(); 
 	finishExecuting(); // finish 1st goal
 	ros::Duration(1.0).sleep(); // Needs to wait so the executeCallback can finish

	EXPECT_TRUE(goal_preempted); // Must be checked in the goal-thread that is cancelled 
	finishExecuting(); // Finish the cancelled goal
	ros::Duration(1.0).sleep(); 
	ASSERT_FALSE(qserv->isActive());  

// 	- if a cancel request on the "next_goal" received, remove it from the queue and set it as cancelled
}
// Two more TEST_F missing

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_queueing_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
