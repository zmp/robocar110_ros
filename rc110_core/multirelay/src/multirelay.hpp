/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>


namespace topic_tools
{
/**
 * The node allows to change the current ros namespace to a custom one for a set of topics.
 *
 * @note
 * Subscription happens when input ns != output ns.
 */
class MultyRelay
{
public:
    MultyRelay();

private:
    bool hasSameNs();
    void subscribe();
    void onInputNs(const std_msgs::String& name);
    void onOutputNs(const std_msgs::String& name);
    void onMessage(const std::string& topic, const ros::MessageEvent<topic_tools::ShapeShifter>& event);

private:
    ros::NodeHandle handle;
    ros::Subscriber inputNsSubscriber, outputNsSubscriber;
    std::string inputNs, outputNs;
    std::vector<std::string> topics;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;
    std::vector<ros::Timer> timers;
};
}  // namespace topic_tools