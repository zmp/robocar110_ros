/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include <ros/ros.h>

#include "rc110_drive_control.hpp"

using namespace zmp;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rc110_drive_control");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    try {
        Rc110DriveControl node(nh, nhPrivate);

        ros::spin();
    } catch (std::exception& ex) {
        std::cerr << "Exception in main(): " << ex.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Unknown exception in main()" << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}