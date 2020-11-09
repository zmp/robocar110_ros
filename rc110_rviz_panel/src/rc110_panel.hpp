/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#pragma once

#ifndef Q_MOC_RUN
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <rc110_msgs/Rc110Status.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <zmp/RcCommon.hpp>
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
class Rc110Panel : public rviz::Panel
{
    Q_OBJECT

    enum TreeItemGroup { DRIVE, BATTERY, TEMPERATURE, IMU };
    static constexpr const char* TREE_ITEM_GROUP_NAMES[] = {"Drive", "Battery", "Temperature", "IMU"};

public:
    explicit Rc110Panel(QWidget* parent = nullptr);
    ~Rc110Panel() override;

protected:
    void timerEvent(QTimerEvent* event) override;

private:
    QTreeWidgetItem* getTreeItem(TreeItemGroup group, const char* name) const;

    void onEnableBoard(bool on);
    void onEnableAd(bool on);
    void onSetMotorState(QAbstractButton* button);
    void onSetServoState(QAbstractButton* button);
    void onEditingFinished();
    void publishDrive();

    void onAdModeChanged(const std_msgs::String& message);
    void onMotorSpeed(const std_msgs::Float32& message);
    void onSteeringAngle(const std_msgs::Float32& message);
    void showDriveGoalStatus();
    void onGyroOffset(const std_msgs::Float32& message);
    void onMotorCurrentOffset(const std_msgs::Float32& message);
    void onSteeringOffset(const std_msgs::Float32& message);

    void onError(const std_msgs::UInt8& message);
    void onRobotStatus(const rc110_msgs::Rc110Status& message);
    void onDriveStatus(const ackermann_msgs::AckermannDriveStamped& driveStatus);
    void onOdometry(const nav_msgs::Odometry& odometry);
    void onServoBattery(const sensor_msgs::BatteryState& batteryState);
    void onMotorBattery(const sensor_msgs::BatteryState& batteryState);
    void onBaseboardTemperature(const sensor_msgs::Temperature& temperature);
    void onServoTemperature(const sensor_msgs::Temperature& temperature);
    void onImu(const sensor_msgs::Imu& imu);

private:
    std::unique_ptr<Ui::PanelWidget> ui;
    ros::NodeHandle handle;
    QVector<ros::Subscriber> subscribers;
    QMap<QString, ros::Publisher> publishers;
    QHash<TreeItemGroup, QTreeWidgetItem*> treeItems;
    QStatusBar* statusBar;

    float driveSpeed = 0;
    float steeringAngle = 0;
};
}  // namespace zmp