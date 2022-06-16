/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#ifndef Q_MOC_RUN
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rc110_msgs/msg/baseboard_error.hpp>
#include <rc110_msgs/msg/motor_rate.hpp>
#include <rc110_msgs/msg/offsets.hpp>
#include <rc110_msgs/msg/status.hpp>
#include <rc110_msgs/msg/wheel_speeds.hpp>
#include <rc110_msgs/srv/set_integer.hpp>
#include <rc110_topic_tools/srv/mux_select.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#endif

namespace Ui
{
class PanelWidget;
}
class QAbstractButton;
class QStatusBar;
class QTreeWidgetItem;

namespace zmp
{
/**
 * Widget with RC 1/10 sensors that can be displayed in rviz.
 */
class Rc110Panel : public rviz_common::Panel
{
    Q_OBJECT

    enum TreeItemGroup { DRIVE, BATTERY, TEMPERATURE, IMU, OTHER };
    static constexpr const char* TREE_ITEM_GROUP_NAMES[] = {"Drive", "Battery", "Temperature", "IMU", "Other"};

public:
    explicit Rc110Panel(QWidget* parent = nullptr);
    ~Rc110Panel() override;

    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

protected:
    void onInitialize() override;
    void timerEvent(QTimerEvent*) override;

private:
    void updateRobotNames();
    void trySelectingRobot();
    void setupRobotName(const std::string& name);
    void updateJoystickIcon();
    void setupRosConnections();
    QTreeWidgetItem* getTreeItem(TreeItemGroup group, const char* name) const;

    void onRobotSelectedButton(bool on);
    void onEnableBoard(bool on);
    void onEnableAd(bool on);
    void onSetMotorState(QAbstractButton* button);
    void onSetServoState(QAbstractButton* button);
    void onRobotNameChanged(int id = 0);
    void onEditingFinished();
    void onCalibrate();
    void onFinishCalibration();
    void publishRobotName(const std::string& name);
    void publishDrive();
    void publishOffsets();

    void onAdModeChanged(const std_msgs::msg::String& message);
    void onMotorSpeed(const std_msgs::msg::Float32& message);
    void onSteeringAngle(const std_msgs::msg::Float32& message);
    void showDriveGoalStatus();

    void onError(const rc110_msgs::msg::BaseboardError& message);
    void onRobotStatus(const rc110_msgs::msg::Status& message);
    void onOffsets(const rc110_msgs::msg::Offsets& message);
    void onDriveStatus(const ackermann_msgs::msg::AckermannDriveStamped& driveStatus);
    void onOdometry(const nav_msgs::msg::Odometry& odometry);
    void onServoBattery(const sensor_msgs::msg::BatteryState& batteryState);
    void onMotorBattery(const sensor_msgs::msg::BatteryState& batteryState);
    void onBaseboardTemperature(const sensor_msgs::msg::Temperature& temperature);
    void onServoTemperature(const sensor_msgs::msg::Temperature& temperature);
    void onImu(const sensor_msgs::msg::Imu& imu);
    void onMotorRate(const rc110_msgs::msg::MotorRate& motorRate);
    void onWheelSpeeds(const rc110_msgs::msg::WheelSpeeds& wheelSpeeds);

    template <typename T>
    void publish(const QString& topic, const T& value)
    {
        std::static_pointer_cast<rclcpp::Publisher<T>>(publishers[topic])->publish(value);
    }

private:
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<Ui::PanelWidget> ui;
    rclcpp::SubscriptionBase::SharedPtr rcSubscriber;
    QVector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    QMap<QString, rclcpp::PublisherBase::SharedPtr> publishers;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> teleopStatusService;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> enableBoardService;
    std::shared_ptr<rclcpp::Client<rc110_topic_tools::srv::MuxSelect>> muxDriveService;
    std::shared_ptr<rclcpp::Client<rc110_msgs::srv::SetInteger>> motorStateService;
    std::shared_ptr<rclcpp::Client<rc110_msgs::srv::SetInteger>> servoStateService;
    QHash<TreeItemGroup, QTreeWidgetItem*> treeItems;
    QStatusBar* statusBar;
    QTimer* calibrationTimer;
    QMap<QString, QPair<int, float>> calibrationSums;
    QStringList robotNames;  // names in combo box at the top
    QString rc;              // robot name from parameter (if any)
    QString savedRobotName;  // variable to store name from config until it's loaded to combobox
    QString selectedRobot;   // robot manipulated by rviz tools, can be different from robot in combobox

    std::string ns;  // slashed ROS namespace
    float driveSpeed = 0;
    float steeringAngle = 0;
    rc110_msgs::msg::Offsets offsets;
    int timerCounter = 0;
};

// short version of create_subscription, similar to ROS1
template <typename T, typename Class>
auto subscribe(const std::string& topic, int depth, void (Class::*callback)(const T&), Class* object)
{
    return object->template create_subscription<T>(topic, depth, [object, callback](typename T::ConstSharedPtr in) {
        (object->*callback)(*in);
    });
}

}  // namespace zmp