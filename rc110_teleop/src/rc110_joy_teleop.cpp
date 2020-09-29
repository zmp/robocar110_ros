/**
 * @file    rc110_joy_teleop.cpp
 *
 */

#include "rc110_joy_teleop.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>

zmp::Rc110JoyTeleop::Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh)

{
    pnh.param<std::string>("frame_id", m_param.frameId, m_param.frameId);

    m_joySub = pnh.subscribe<sensor_msgs::Joy>("input_joy", 10, &zmp::Rc110JoyTeleop::joyCallback, this);
    m_drivePub = pnh.advertise<ackermann_msgs::AckermannDriveStamped>("output_cmd", 1);
}

zmp::Rc110JoyTeleop::~Rc110JoyTeleop() {}

void zmp::Rc110JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
    ackermann_msgs::AckermannDriveStamped cmdMsg;
    cmdMsg.header.stamp = ros::Time::now();
    cmdMsg.header.frame_id = m_param.frameId;
    cmdMsg.drive.acceleration = 1;
    cmdMsg.drive.jerk = 1;
    cmdMsg.drive.steering_angle_velocity = 1;
    cmdMsg.drive.speed = joyMsg->axes[2] * m_param.maxSpeed;
    cmdMsg.drive.steering_angle = joyMsg->axes[3] * m_param.maxSteeringAngle;
    m_drivePub.publish(cmdMsg);
}

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
