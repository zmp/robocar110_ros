/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "multirelay.hpp"

#include <ros/master.h>

namespace topic_tools
{
MultyRelay::MultyRelay()
{
    topics = ros::param::param("~topics", topics);

    ros::NodeHandle privateHandle("~");
    inputNsSubscriber = privateHandle.subscribe("input_ns", 1, &MultyRelay::onInputNs, this);
    outputNsSubscriber = privateHandle.subscribe("output_ns", 1, &MultyRelay::onOutputNs, this);
}

bool MultyRelay::hasSameNs()
{
    if (inputNs == outputNs) {
        return true;
    }
    auto ns = ros::this_node::getNamespace();
    ns = ns[0] != '/' ? ns : ns.substr(1);

    return (inputNs.empty() && ns == outputNs) || (outputNs.empty() && ns == inputNs);
}

void MultyRelay::subscribe()
{
    if (subscribers.size()) return;

    for (auto topic : topics) {
        using EventType = ros::MessageEvent<topic_tools::ShapeShifter>;
        boost::function<void(const EventType&)> callback = [this, topic](const EventType& event) {
            onMessage(topic, event);
        };
        auto fullTopic = inputNs.empty() ? topic : "/" + inputNs + "/" + topic;
        subscribers.push_back(handle.subscribe<topic_tools::ShapeShifter>(fullTopic, 2, callback));
    }
}

void MultyRelay::onInputNs(const std_msgs::String& name)
{
    if (inputNs != name.data) {
        inputNs = name.data;
        subscribers.clear();

        if (hasSameNs()) {
            publishers.clear();
            timers.clear();
        } else {
            subscribe();
        }
    }
}

void MultyRelay::onOutputNs(const std_msgs::String& name)
{
    if (outputNs != name.data) {
        outputNs = name.data;
        publishers.clear();
        timers.clear();

        if (hasSameNs()) {
            subscribers.clear();
        } else {
            subscribe();
        }
    }
}

void MultyRelay::onMessage(const std::string& topic, const ros::MessageEvent<topic_tools::ShapeShifter>& event)
{
    auto message = event.getConstMessage();
    if (publishers.count(topic)) {
        publishers[topic].publish(message);
        return;
    }

    auto outTopic = outputNs.empty() ? topic : "/" + outputNs + "/" + topic;
    const auto& connectionHeader = event.getConnectionHeaderPtr();
    auto it = connectionHeader->find("latching");
    bool latch = it != connectionHeader->end() && it->second == "1";

    publishers[topic] = message->advertise(handle, outTopic, 1, latch);

    // Make sure that publish happens after subscription on output topic.
    // Timer is the simplest solution, callback on connection needs to check total number of subscribers.
    timers.push_back(handle.createTimer(
            ros::Duration(0.5),
            [=, topic = topic](const ros::TimerEvent&) { publishers[topic].publish(message); },
            bool("oneshot")));
}
}  // namespace topic_tools