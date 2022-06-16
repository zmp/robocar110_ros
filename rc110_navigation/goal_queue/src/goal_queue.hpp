/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace goal_queue
{
class GoalQueue
{
    using ActionClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
    using GoalState = actionlib::SimpleClientGoalState;
    using ActionResult = move_base_msgs::MoveBaseResultConstPtr;

public:
    GoalQueue();

private:
    void onGoal(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void onReset(const geometry_msgs::PointStamped::ConstPtr& point);
    void onDone(const GoalState& state, const ActionResult& result);
    void trySendingGoal();

private:
    ros::NodeHandle handlePrivate{"~"};
    bool isLoop;
    ros::Subscriber goalSubscriber;
    ros::Subscriber resetSubscriber;
    ros::Publisher pathPublisher;
    ros::Timer timer;
    ActionClient actionClient;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped sentPose;
    int iGoalPose = 0;
    bool isMoving = false;
};

}  // namespace goal_queue
