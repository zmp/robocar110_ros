/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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