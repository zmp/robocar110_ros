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
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#endif

namespace Ui
{
class PanelWidget;
}
class QTreeWidgetItem;

namespace zmp
{
/**
 * Widget with RC 1/10 sensors that can be displayed in rviz.
 */
class Rc110Panel : public rviz::Panel
{
    Q_OBJECT

    enum TREE_ITEM_GROUP { DRIVE, BATTERY, TEMPERATURE, IMU };
    static constexpr const char* TREE_ITEM_GROUP_NAME[] = {"Drive", "Battery", "Temperature", "IMU"};

public:
    explicit Rc110Panel(QWidget* parent = nullptr);
    ~Rc110Panel() override;

private:
    QTreeWidgetItem* getTreeItem(TREE_ITEM_GROUP group, const char* name) const;
    void onDriveStatus(const ackermann_msgs::AckermannDriveStamped& driveStatus);
    void onMotorBattery(const sensor_msgs::BatteryState& batteryState);
    void onMotorTemperature(const sensor_msgs::Temperature& temperature);
    void onServoTemperature(const sensor_msgs::Temperature& temperature);
    void onImu(const sensor_msgs::Imu& imu);

private:
    std::unique_ptr<Ui::PanelWidget> ui;
    ros::NodeHandle handle;
    QVector<ros::Subscriber> subscribers;
    QHash<TREE_ITEM_GROUP, QTreeWidgetItem*> treeItems;
};
}  // namespace zmp