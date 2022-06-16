/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "goal_queue.hpp"

namespace goal_queue
{
GoalQueue::GoalQueue() : isLoop(ros::param::param("~loop", true)), actionClient("move_base", true)
{
    while (!actionClient.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    path.header.frame_id = "map";

    goalSubscriber = handlePrivate.subscribe("goal", 1, &GoalQueue::onGoal, this);
    resetSubscriber = handlePrivate.subscribe("reset", 1, &GoalQueue::onReset, this);
    pathPublisher = handlePrivate.advertise<nav_msgs::Path>("path", 1, bool("latch"));
    timer = handlePrivate.createTimer(ros::Duration(0.2), [this](const auto&) { trySendingGoal(); });
}

void GoalQueue::onGoal(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    path.poses.push_back(*pose);
    if (!isMoving) {
        iGoalPose = path.poses.size() - 1;
    }

    path.header.stamp = ros::Time::now();
    pathPublisher.publish(path);
}

void GoalQueue::onReset(const geometry_msgs::PointStamped::ConstPtr& point)
{
    iGoalPose = 0;
    path.poses.clear();
    pathPublisher.publish(path);
}

void GoalQueue::onDone(const GoalState& state, const ActionResult& result)
{
    isMoving = false;
    if (path.poses.size() > 1) {
        if (iGoalPose + 1 < path.poses.size()) {
            ++iGoalPose;
        } else if (isLoop) {
            iGoalPose = 0;
        }
        // Goal sending does not work from callback, so timer have to be used.
    }
}

void GoalQueue::trySendingGoal()
{
    if (path.poses.empty()) return;

    if (sentPose != path.poses[iGoalPose]) {
        isMoving = true;
        sentPose = path.poses[iGoalPose];

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = sentPose.pose;

        actionClient.sendGoal(goal,
                              [this](const GoalState& state, const ActionResult& result) { onDone(state, result); });
    }
}
}  // namespace goal_queue