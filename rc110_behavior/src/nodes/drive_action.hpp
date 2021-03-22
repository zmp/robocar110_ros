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