/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran, Andrei Pak
 */

#include "rc110_joy_teleop.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
try {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<zmp::Rc110JoyTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}  //
catch (std::exception& ex) {
    std::cerr << "Exception in main(): " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    return EXIT_FAILURE;
}