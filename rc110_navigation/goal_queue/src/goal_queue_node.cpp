/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include <iostream>

#include "goal_queue.hpp"

int main(int argc, char* argv[])
try {
    ros::init(argc, argv, "goal_queue");

    goal_queue::GoalQueue node;
    ros::spin();

    return EXIT_SUCCESS;
}  //
catch (std::exception& ex) {
    std::cerr << "Exception in main(): " << ex.what();
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "Unknown exception in main()";
    return EXIT_FAILURE;
}