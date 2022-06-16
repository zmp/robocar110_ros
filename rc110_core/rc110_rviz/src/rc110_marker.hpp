/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace zmp
{
class RC110Marker : public rclcpp::Node
{
public:
    RC110Marker();

private:
    std::string meshFile;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;
};

}  // namespace zmp
