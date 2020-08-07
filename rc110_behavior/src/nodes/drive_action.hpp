/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>

namespace zmp
{
/**
 * This node changes car speed and steering angle.
 */
class DriveAction : public BT::SyncActionNode
{
public:
    static constexpr char NAME[] = "DriveAction";

public:
    DriveAction(const std::string& name, const BT::NodeConfiguration& config, ros::Publisher& drivePublisher) :
            SyncActionNode(name, config),
            drivePublisher(drivePublisher)
    {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<float>("speed"), BT::InputPort<float>("steering")}; }

protected:
    BT::NodeStatus tick() override;

private:
    ros::Publisher& drivePublisher;
};
}  // namespace zmp