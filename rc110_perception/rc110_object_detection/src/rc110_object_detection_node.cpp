/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#include "rc110_object_detection.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<zmp::Rc110ObjectDetection>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& ex) {
        std::cerr << "Exception in main(): " << ex.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Unknown exception in main()" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
