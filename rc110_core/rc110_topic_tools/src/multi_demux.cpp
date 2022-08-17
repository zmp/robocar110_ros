/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */

#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>

namespace topic_tools
{
/**
 * Multiple topics namespace demultiplexer.
 *
 * @note
 * Subscription happens when input ns != output ns.
 */
class MultiDemux
{
public:
    MultiDemux();

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

MultiDemux::MultiDemux()
{
    topics = ros::param::param("~topics", topics);

    ros::NodeHandle privateHandle("~");
    inputNsSubscriber = privateHandle.subscribe("input_ns", 1, &MultiDemux::onInputNs, this);
    outputNsSubscriber = privateHandle.subscribe("output_ns", 1, &MultiDemux::onOutputNs, this);
}

bool MultiDemux::hasSameNs()
{
    if (inputNs == outputNs) {
        return true;
    }
    auto ns = ros::this_node::getNamespace();
    ns = ns[0] != '/' ? ns : ns.substr(1);

    return (inputNs.empty() && ns == outputNs) || (outputNs.empty() && ns == inputNs);
}

void MultiDemux::subscribe()
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

void MultiDemux::onInputNs(const std_msgs::String& name)
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

void MultiDemux::onOutputNs(const std_msgs::String& name)
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

void MultiDemux::onMessage(const std::string& topic, const ros::MessageEvent<topic_tools::ShapeShifter>& event)
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

int main(int argc, char* argv[])
try {
    ros::init(argc, argv, "multi_demux");

    topic_tools::MultiDemux node;
    ros::spin();

    return EXIT_SUCCESS;
}  //
catch (std::exception& ex) {
    std::cerr << "Exception in main(): " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    return EXIT_FAILURE;
}