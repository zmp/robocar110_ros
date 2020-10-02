/**
 * @file    rc110_joy_teleop_node.cpp
 *
 * @author  btran
 *
 */

#include "rc110_joy_teleop.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rc110_joy_teleop");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    double rateHz = 30.0;
    pnh.param<double>("rate", rateHz, rateHz);
    ros::Rate r(rateHz);

    try {
        zmp::Rc110JoyTeleop node(nh, pnh);
        while (ros::ok()) {
            ros::spinOnce();
            node.publish(ros::Duration(1 / rateHz));
            r.sleep();
        }
    } catch (std::exception& ex) {
        ROS_ERROR_STREAM("Exception in main(): " << ex.what());
        return EXIT_FAILURE;
    } catch (...) {
        ROS_ERROR_STREAM("Unknown exception in main()");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
