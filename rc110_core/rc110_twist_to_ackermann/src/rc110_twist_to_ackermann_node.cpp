/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include <ros/ros.h>

#include "rc110_twist_to_ackermann.hpp"

using namespace zmp;

int main(int argc, char** argv)
try {
    ros::init(argc, argv, "rc110_twist_to_ackermann");

    Rc110TwistToAckermann node;

    ros::spin();

    return EXIT_SUCCESS;
}  //
catch (std::exception& ex) {
    std::cerr << "Exception in main(): " << ex.what() << std::endl;
    return EXIT_FAILURE;
}  //
catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    return EXIT_FAILURE;
}