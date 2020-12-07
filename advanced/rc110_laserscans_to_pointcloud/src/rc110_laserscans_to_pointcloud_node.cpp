/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include <ros/ros.h>

#include "rc110_laserscans_to_pointcloud.hpp"

using namespace zmp;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rc110_laserscans_to_pointcloud");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    Rc110LaserScansToPointCloud node(nh, nhPrivate);
    ros::Rate spinRate(10);

    try {
        while (ros::ok()) {
            ros::spinOnce();
            spinRate.sleep();
        }
    } catch (std::exception& ex) {
        std::cerr << "Exception in main(): " << ex.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Unknown exception in main()" << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}