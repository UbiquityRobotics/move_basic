/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Ubiquity Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Ubiquity Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef QUEUED_ACTION_SERVER_H_
#define QUEUED_ACTION_SERVER_H_

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server.h>
#include <ros/ros.h>

#include <condition_variable>
#include <string>
#include <thread>

namespace actionlib {
template <class ActionSpec>
class QueuedActionServer {
public:
    // Generates typedefs that we'll use to make our lives easier
    ACTION_DEFINITION(ActionSpec);

    // More useful typedefs
    typedef typename ActionServer<ActionSpec>::GoalHandle GoalHandle;
    typedef boost::function<void(const GoalConstPtr &)> ExecuteCallback;

    QueuedActionServer(std::string name, ExecuteCallback execute_callback);
    QueuedActionServer(ros::NodeHandle n, std::string name, ExecuteCallback execute_callback);
    ~QueuedActionServer();

    boost::shared_ptr<const Goal> acceptNewGoal();
    bool isNewGoalAvailable();
    bool isPreemptRequested();
    bool isActive();
    void setSucceeded(const Result &result = Result(), const std::string &text = std::string(""));
    void setAborted(const Result &result = Result(), const std::string &text = std::string(""));
    void setPreempted(const Result &result = Result(), const std::string &text = std::string(""));
    void start();
    void shutdown();

private:
    void goalCallback(GoalHandle preempt);
    void preemptCallback(GoalHandle preempt);

    void executeLoop();

    ros::NodeHandle n_;

    std::shared_ptr<ActionServer<ActionSpec>> as;

    GoalHandle current_goal, next_goal;

    bool new_goal_, preempt_request_, new_goal_preempt_request_;

    std::recursive_mutex lock;

    ExecuteCallback execute_callback;
    std::condition_variable_any execute_condition;
    std::thread *execute_thread;

    std::atomic<bool> need_to_terminate;
};

}  // namespace actionlib

// include the implementation here
#include <move_basic/queued_action_server_imp.h>
#endif
