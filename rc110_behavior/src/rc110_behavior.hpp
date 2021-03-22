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

#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace zmp
{
/**
 * Main car controlling class.
 * It's reading sensors and moving drives accordingly.
 */
class Rc110Behavior
{
    struct Parameters {
        std::string treeFile;
    };

public:
    Rc110Behavior(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate);

    /**
     * Behavior tree tick that is triggered in the main loop.
     */
    void update();

private:
    template <class Node, class... Args>
    void registerNodeBuilder(Args... args)
    {
        behaviorTreeFactory.registerBuilder<Node>(  //
                Node::NAME,
                [args...](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<Node>(name, config, args...);
                });
    }

    void onLaser(const sensor_msgs::LaserScan& scan);

private:
    Parameters parameters;
    ros::Subscriber laserSubscriber;
    ros::Publisher drivePublisher;

    BT::BehaviorTreeFactory behaviorTreeFactory;
    BT::Tree behaviorTree;

    sensor_msgs::PointCloud2 cloud;
};
}  // namespace zmp
