/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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