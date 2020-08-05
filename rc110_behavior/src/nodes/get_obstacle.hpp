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
#include <sensor_msgs/PointCloud2.h>

namespace zmp
{
/**
 * This node basing on obstacle position sets variable named "result" to either of the values: none | near | left | right
 */
class GetObstacle : public BT::DecoratorNode
{
public:
    static constexpr char NAME[] = "GetObstacle";

public:
    GetObstacle(const std::string& name, const BT::NodeConfiguration& config, const sensor_msgs::PointCloud2& cloud) :
            DecoratorNode(name, config), cloud(cloud)
    {
    }

    static BT::PortsList providedPorts() { return {BT::OutputPort<std::string>("switch")}; }

    BT::NodeStatus tick() override;

private:
    const sensor_msgs::PointCloud2& cloud;
};
}  // namespace zmp