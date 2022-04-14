/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "multirelay.hpp"

int main(int argc, char* argv[])
try {
    ros::init(argc, argv, "multirelay");

    topic_tools::MultyRelay node;
    ros::spin();

    return EXIT_SUCCESS;
}  //
catch (std::exception& ex) {
    ROS_ERROR_STREAM("Exception in main(): " << ex.what());
    return EXIT_FAILURE;
} catch (...) {
    ROS_ERROR_STREAM("Unknown exception in main()");
    return EXIT_FAILURE;
}