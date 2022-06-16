/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace topic_tools
{
/**
 * Multiple topics namespace demultiplexer.
 *
 * @note
 * Subscription happens when input ns != output ns.
 */
class MultiDemux : public rclcpp::Node
{
public:
    MultiDemux();

private:
    bool hasSameNs();
    void subscribe();
    void onInputNs(const std_msgs::msg::String& name);
    void onOutputNs(const std_msgs::msg::String& name);
    void onMessage(const std::string& topic, const std::shared_ptr<rclcpp::SerializedMessage>& message);

private:
    rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr inputNsSubscriber, outputNsSubscriber;
    std::string inputNs, outputNs;
    std::vector<std::string> topics;
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscribers;
    std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers;
    rclcpp::TimerBase::SharedPtr timer;
};

MultiDemux::MultiDemux() : Node("rc110_multi_demux")
{
    using std::placeholders::_1;
    topics = declare_parameter("topics", topics);

    inputNsSubscriber =
            create_subscription<std_msgs::msg::String>("input_ns", 1, std::bind(&MultiDemux::onInputNs, this, _1));
    outputNsSubscriber =
            create_subscription<std_msgs::msg::String>("output_ns", 1, std::bind(&MultiDemux::onOutputNs, this, _1));

    timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&MultiDemux::subscribe, this));
}

bool MultiDemux::hasSameNs()
{
    if (inputNs == outputNs) {
        return true;
    }
    std::string ns = get_namespace();
    ns = ns[0] != '/' ? ns : ns.substr(1);

    return (inputNs.empty() && ns == outputNs) || (outputNs.empty() && ns == inputNs);
}

void MultiDemux::subscribe()
{
    if (!subscribers.empty() || hasSameNs()) return;

    for (auto& topic : topics) {
        auto fullTopic = inputNs.empty() ? topic : "/" + inputNs + "/" + topic;

        std::vector<rclcpp::TopicEndpointInfo> publisherInfos = get_publishers_info_by_topic(fullTopic);
        if (!publisherInfos.empty()) {
            rclcpp::TopicEndpointInfo info = publisherInfos.front();
            auto topicType = info.topic_type();

            auto outTopic = outputNs.empty() ? topic : "/" + outputNs + "/" + topic;
            publishers[topic] = create_generic_publisher(outTopic, topicType, info.qos_profile());

            subscribers.push_back(
                    create_generic_subscription(fullTopic,
                                                topicType,
                                                info.qos_profile(),
                                                [this, topic](const std::shared_ptr<rclcpp::SerializedMessage>& message) {
                                                    onMessage(topic, message);
                                                }));
        }
    }
}

void MultiDemux::onInputNs(const std_msgs::msg::String& name)
{
    if (inputNs != name.data) {
        inputNs = name.data;
        publishers.clear();
        subscribers.clear();
        subscribe();
    }
}

void MultiDemux::onOutputNs(const std_msgs::msg::String& name)
{
    if (outputNs != name.data) {
        outputNs = name.data;
        publishers.clear();
        subscribers.clear();
        subscribe();
    }
}

void MultiDemux::onMessage(const std::string& topic, const std::shared_ptr<rclcpp::SerializedMessage>& message)
{
    if (publishers.count(topic)) {
        publishers[topic]->publish(*message);
    }
}
}  // namespace topic_tools

int main(int argc, char* argv[])
try {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<topic_tools::MultiDemux>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}  //
catch (std::exception& ex) {
    std::cerr << "Exception in main(): " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    return EXIT_FAILURE;
}