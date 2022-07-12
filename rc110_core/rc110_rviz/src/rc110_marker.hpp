/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ros/ros.h>

namespace zmp
{
class RC110Marker
{
public:
    RC110Marker();

private:
    ros::NodeHandle handle;
    std::string meshFile;
    ros::Publisher publisher;
};

}  // namespace zmp
