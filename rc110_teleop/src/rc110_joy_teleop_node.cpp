/**
 * @file    rc110_joy_teleop_node.cpp
 *
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
