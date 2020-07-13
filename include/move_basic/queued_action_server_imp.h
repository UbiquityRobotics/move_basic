/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#ifndef QUEUED_ACTION_SERVER_IMP_H_
#define QUEUED_ACTION_SERVER_IMP_H_

#include <ros/ros.h>
#include <chrono>
#include <string>

namespace actionlib {

template <class ActionSpec>
QueuedActionServer<ActionSpec>::QueuedActionServer(std::string name,
                                                   ExecuteCallback execute_callback)
    : new_goal_(false),
      preempt_request_(false),
      new_goal_preempt_request_(false),
      execute_callback(execute_callback),
      execute_thread(NULL),
      need_to_terminate(false) {
    // create the action server
    as = std::make_shared<ActionServer<ActionSpec>>(
        n_, name, boost::bind(&QueuedActionServer::goalCallback, this, _1),
        boost::bind(&QueuedActionServer::preemptCallback, this, _1), false);

    if (execute_callback != NULL) {
        execute_thread = new std::thread(std::bind(&QueuedActionServer::executeLoop, this));
    }
}

template <class ActionSpec>
QueuedActionServer<ActionSpec>::QueuedActionServer(ros::NodeHandle n, std::string name,
                                                   ExecuteCallback execute_callback)
    : n_(n),
      new_goal_(false),
      preempt_request_(false),
      new_goal_preempt_request_(false),
      execute_callback(execute_callback),
      execute_thread(NULL),
      need_to_terminate(false) {
    // create the action server
    as = std::make_shared<ActionServer<ActionSpec>>(
        n, name, boost::bind(&QueuedActionServer::goalCallback, this, _1),
        boost::bind(&QueuedActionServer::preemptCallback, this, _1), false);

    if (execute_callback != NULL) {
        execute_thread = new std::thread(std::bind(&QueuedActionServer::executeLoop, this));
    }
}

template <class ActionSpec>
QueuedActionServer<ActionSpec>::~QueuedActionServer() {
    if (execute_thread) {
        shutdown();
    }
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::shutdown() {
    if (execute_callback) {
        need_to_terminate = true;

        assert(execute_thread);
        if (execute_thread) {
            execute_thread->join();
            delete execute_thread;
            execute_thread = NULL;
        }
    }
}

template <class ActionSpec>
boost::shared_ptr<const typename QueuedActionServer<ActionSpec>::Goal>
QueuedActionServer<ActionSpec>::acceptNewGoal() {
    std::lock_guard<std::recursive_mutex> lk(lock);

    if (!new_goal_ || !next_goal.getGoal()) {
        ROS_ERROR_NAMED("actionlib",
                        "Attempting to accept the next goal when a new goal is not available");
        return boost::shared_ptr<const Goal>();
    }

    ROS_DEBUG_NAMED("actionlib", "Accepting a new goal");

    // accept the next goal
    current_goal = next_goal;
    new_goal_ = false;

    // set preempt to request to equal the preempt state of the new goal
    preempt_request_ = new_goal_preempt_request_;
    new_goal_preempt_request_ = false;

    // set the status of the current goal to be active
    current_goal.setAccepted("This goal has been accepted by the simple action server");

    return current_goal.getGoal();
}

template <class ActionSpec>
bool QueuedActionServer<ActionSpec>::isNewGoalAvailable() {
    return new_goal_;
}

template <class ActionSpec>
bool QueuedActionServer<ActionSpec>::isPreemptRequested() {
    return preempt_request_;
}

template <class ActionSpec>
bool QueuedActionServer<ActionSpec>::isActive() {
    if (!current_goal.getGoal()) {
        return false;
    }
    unsigned int status = current_goal.getGoalStatus().status;
    return status == actionlib_msgs::GoalStatus::ACTIVE ||
           status == actionlib_msgs::GoalStatus::PREEMPTING;
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::setSucceeded(const Result& result, const std::string& text) {
    std::lock_guard<std::recursive_mutex> lk(lock);
    ROS_DEBUG_NAMED("actionlib", "Setting the current goal as succeeded");
    current_goal.setSucceeded(result, text);
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::setAborted(const Result& result, const std::string& text) {
    std::lock_guard<std::recursive_mutex> lk(lock);
    ROS_DEBUG_NAMED("actionlib", "Setting the current goal as aborted");
    current_goal.setAborted(result, text);
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::setPreempted(const Result& result, const std::string& text) {
    std::lock_guard<std::recursive_mutex> lk(lock);
    ROS_DEBUG_NAMED("actionlib", "Setting the current goal as canceled");
    current_goal.setCanceled(result, text);
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::goalCallback(GoalHandle goal) {
    std::lock_guard<std::recursive_mutex> lk(lock);
    ROS_DEBUG_NAMED("actionlib", "A new goal has been recieved by the single goal action server");

    // check that the timestamp is past or equal to that of the current goal and the next goal
    if ((!current_goal.getGoal() || goal.getGoalID().stamp >= current_goal.getGoalID().stamp) &&
        (!next_goal.getGoal() || goal.getGoalID().stamp >= next_goal.getGoalID().stamp)) {
	
        // if next_goal has not been accepted already... its going to get bumped, but we need to let
        // the client know we're preempting
        if (next_goal.getGoal() && (!current_goal.getGoal() || next_goal != current_goal)) {
            next_goal.setCanceled(Result(),
                                  "This goal was canceled because another goal was received"
                                  "bumping this one out of the queue");
        }

        next_goal = goal;
        new_goal_ = true;
        new_goal_preempt_request_ = false;

        // Trigger runLoop to call execute()
        execute_condition.notify_all();
    } else {
        // This is an old goal, we are going to reject it
        goal.setRejected(
            Result(),
            "This goal was rejected because another goal there are newer goals in the queue");
    }
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::preemptCallback(GoalHandle preempt) {
    std::lock_guard<std::recursive_mutex> lk(lock);
    ROS_DEBUG_NAMED("actionlib", "A preempt has been received by the QueuedActionServer");

    // if the preempt is for the current goal, then we'll set the preemptRequest flag and call the
    // user's preempt callback
    if (preempt == current_goal) {
        ROS_DEBUG_NAMED(
            "actionlib",
            "Setting preempt_request bit for the current goal to TRUE");
        preempt_request_ = true;

    } else if (preempt == next_goal) {
        // if the preempt applies to the next goal, we'll set the preempt bit for that
        ROS_DEBUG_NAMED("actionlib", "Setting preempt request bit for the next goal to TRUE");
        new_goal_preempt_request_ = true;
    }
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::executeLoop() {
    ros::Duration loop_duration = ros::Duration().fromSec(.1);

    while (n_.ok()) {
        if (need_to_terminate) {
            break;
        }

        std::unique_lock<std::recursive_mutex> lk(lock);
        execute_condition.wait_for(lk, std::chrono::duration<double>(loop_duration.toSec()),
                                   [this] { return this->new_goal_; });

        if (isActive()) {
            ROS_ERROR_NAMED("actionlib", "Should never reach this code with an active goal");
        } else if (isNewGoalAvailable()) {
            GoalConstPtr goal = acceptNewGoal();

            ROS_FATAL_COND(!execute_callback,
                           "execute_callback must exist. This is a bug in QueuedActionServer");

            {
                // Make sure we're not locked when we call execute, relock in exception safe way
                lk.unlock();
                try {
                    execute_callback(goal);
                } catch (...) {
                    lk.lock();
                    throw;
                }
                lk.lock();
            }

            if (isActive()) {
                ROS_WARN_NAMED("actionlib",
                               "Your executeCallback did not set the goal to a terminal status.\n"
                               "This is a bug in your ActionServer implementation. Fix your code!\n"
                               "For now, the ActionServer will set this goal to aborted");
                setAborted(Result(),
                           "This goal was aborted by the simple action server. The user should "
                           "have set a terminal status on this goal and did not");
            }
        }
    }
}

template <class ActionSpec>
void QueuedActionServer<ActionSpec>::start() {
    as->start();
}

}  // namespace actionlib

#endif
