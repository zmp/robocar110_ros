/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <sensor_msgs/PointCloud2.h>

namespace zmp
{
/**
 * This node basing on obstacle position sets the value of the output port "switch" to either: none | near | left | right
 */
class CheckObstacle : public BT::SyncActionNode
{
public:
    static constexpr char NAME[] = "CheckObstacle";

public:
    CheckObstacle(const std::string& name, const BT::NodeConfiguration& config, const sensor_msgs::PointCloud2& cloud) :
            SyncActionNode(name, config),
            cloud(cloud)
    {
    }

    static BT::PortsList providedPorts() { return {BT::OutputPort<std::string>("switch")}; }

    BT::NodeStatus tick() override;

private:
    const sensor_msgs::PointCloud2& cloud;
};
}  // namespace zmp