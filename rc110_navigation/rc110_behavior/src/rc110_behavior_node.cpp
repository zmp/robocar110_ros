/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include <ros/ros.h>

#include "rc110_behavior.hpp"

using namespace zmp;

int main(int argc, char** argv)
try {
    ros::init(argc, argv, "rc110_behavior");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    Rc110Behavior node(nh, nhPrivate);
    ros::Rate spinRate(10);

    while (ros::ok()) {
        ros::spinOnce();
        node.update();
        spinRate.sleep();
    }
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