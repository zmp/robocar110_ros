/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */

#include <rc110_topic_tools/srv/mux_select.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace topic_tools
{
/**
 * Topic mux.
 */
class Mux : public rclcpp::Node
{
public:
    explicit Mux(const std::vector<std::string>& args);

private:
    void selectTopic(const std::string& name);
    void subscribe();
    void onSelect(const std::shared_ptr<rc110_topic_tools::srv::MuxSelect::Request>& request,
                  const std::shared_ptr<rc110_topic_tools::srv::MuxSelect::Response>& response);

private:
    std::string outTopic, selectedTopic;
    std::vector<std::string> inTopics;
    rclcpp::GenericSubscription::SharedPtr subscriber;
    rclcpp::GenericPublisher::SharedPtr publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selectedPublisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<rc110_topic_tools::srv::MuxSelect>::SharedPtr mux_select_srv_;
};

Mux::Mux(const std::vector<std::string>& args) : Node("rc110_mux")
{
    outTopic = args[1];
    inTopics = std::vector<std::string>(args.begin() + 2, args.end());
    selectedTopic = inTopics.front();

    selectedPublisher = create_publisher<std_msgs::msg::String>("~/selected", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
    timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&Mux::subscribe, this));
    selectTopic(selectedTopic);

    using namespace std::placeholders;
    mux_select_srv_ =
            create_service<rc110_topic_tools::srv::MuxSelect>("~/select", std::bind(&Mux::onSelect, this, _1, _2));
}

void Mux::selectTopic(const std::string& name)
{
    selectedTopic = name;
    subscriber.reset();
    subscribe();

    std_msgs::msg::String message;
    message.data = selectedTopic;
    selectedPublisher->publish(message);
}

void Mux::subscribe()
{
    if (subscriber) return;

    std::vector<rclcpp::TopicEndpointInfo> publisherInfos = get_publishers_info_by_topic(selectedTopic);
    if (!publisherInfos.empty()) {
        rclcpp::TopicEndpointInfo info = publisherInfos.front();
        auto topicType = info.topic_type();

        publisher = create_generic_publisher(outTopic, topicType, info.qos_profile());
        subscriber = create_generic_subscription(selectedTopic,
                                                 topicType,
                                                 info.qos_profile(),
                                                 [this](const std::shared_ptr<rclcpp::SerializedMessage>& message) {
                                                     publisher->publish(*message);
                                                 });
    }
}

void Mux::onSelect(const std::shared_ptr<rc110_topic_tools::srv::MuxSelect::Request>& request,
                   const std::shared_ptr<rc110_topic_tools::srv::MuxSelect::Response>& response)
{
    auto it = std::find(inTopics.begin(), inTopics.end(), request->topic);
    if (it == inTopics.end()) return;

    response->prev_topic = selectedTopic;
    selectTopic(request->topic);

    response->success = true;
}
}  // namespace topic_tools

int main(int argc, char* argv[])
try {
    auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    if (args.size() < 3) {
        throw std::runtime_error("mux should have at least one output and one input topic specified");
    }
    auto node = std::make_shared<topic_tools::Mux>(args);
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