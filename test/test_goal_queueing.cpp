#include <ros/ros.h>
#include <move_basic/queued_action_server.h>
#include <move_basic/queued_action_server_imp.h>
#include <gtest/gtest.h>


class GoalQueueSuite : public ::testing::Test {
protected:
	virtual void SetUp() {} // constructor
	virtual void TearDown() {} // destructor
}
;
//ASSERT_* or EXPECT_*
//no underscores (_)!
TEST_F(GoalQueueSuite, executionDelay) {}

TEST_F(GoalQueueSuite, queueAdding) {}

TEST_F(GoalQueueSuite, goalPreempting) {}

TEST_F(GoalQueueSuite, goalCanceling) {}

// Two more TEST_F and a PITFALL missing


int main(int argc, char** argv) {
	ros::init(argc, argv, "goal_queueing_test");
	testing::InitGoogleTest(&argc, argv);
	return (RUN_ALL_TESTS() ? "Testing failed." : "Testing successful.");
}
