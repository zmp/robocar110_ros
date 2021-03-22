/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
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
 * Written by btran
 */

#include "rc110_joy_teleop.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rc110_joy_teleop");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        zmp::Rc110JoyTeleop node(nh, pnh);
        ros::spin();
    } catch (std::exception& ex) {
        ROS_ERROR_STREAM("Exception in main(): " << ex.what());
        return EXIT_FAILURE;
    } catch (...) {
        ROS_ERROR_STREAM("Unknown exception in main()");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
