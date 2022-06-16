/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include <rclcpp/rclcpp.hpp>

#include "rc110_laserscans_to_pointcloud.hpp"

int main(int argc, char** argv)
try {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<zmp::Rc110LaserScansToPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();

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
