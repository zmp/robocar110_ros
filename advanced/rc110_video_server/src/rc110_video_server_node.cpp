/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */

#include <gst/gst.h>
#include <ros/ros.h>

#include <cstdlib>
#include <exception>
#include <iostream>

#include "rc110_video_server.hpp"

int main(int argc, char** argv)
try {
    ros::init(argc, argv, "rc110_video_server");

    zmp::Rc110VideoServer server;

    ros::Rate spinRate(30);
    while (ros::ok()) {
        g_main_context_iteration(nullptr, false);  // does not block
        ros::spinOnce();
        spinRate.sleep();
    }

    return EXIT_SUCCESS;
}  //
catch (const std::exception& ex) {
    std::cerr << "Exception in main(): " << ex.what() << std::endl;
    return EXIT_FAILURE;
}  //
catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    return EXIT_FAILURE;
}