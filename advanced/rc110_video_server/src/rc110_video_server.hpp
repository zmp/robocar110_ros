/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

struct _GOptionEntry;
typedef struct _GOptionEntry GOptionEntry;

namespace zmp
{
class Rc110VideoServer
{
    struct Parameters {
        int debugLevel;
        std::string port;
        std::string urlSuffix;
        std::string videoDevice;
        int maxFrameRate;
        int width;
        int height;
    };

public:
    Rc110VideoServer();

private:
    std::string parseOptions();
    bool gstreamerInit();

private:
    ros::NodeHandle handle;
    ros::NodeHandle handlePrivate;

    Parameters parameters;
    char* portPointer;
    std::unique_ptr<GOptionEntry[]> entries;
};
}  // namespace zmp