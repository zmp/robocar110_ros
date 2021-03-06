/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
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