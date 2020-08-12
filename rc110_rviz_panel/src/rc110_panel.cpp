/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include "rc110_panel.hpp"

#include <boost/math/constants/constants.hpp>

#include "ui_rc110_panel.h"

namespace zmp
{
Rc110Panel::Rc110Panel(QWidget* parent) : Panel(parent), ui(new Ui::PanelWidget)
{
    ui->setupUi(this);

    QList<QTreeWidgetItem*> items;
    for (int i = 0; i < std::size(TREE_ITEM_GROUP_NAME); ++i) {
        items.push_back(new QTreeWidgetItem({TREE_ITEM_GROUP_NAME[i]}));
        treeItems.insert((TREE_ITEM_GROUP)i, items[i]);
    }
    ui->treeWidget->insertTopLevelItems(0, items);

    subscribers.push_back(handle.subscribe("drive_status", 1, &Rc110Panel::onDriveStatus, this));
    subscribers.push_back(handle.subscribe("motor_battery", 1, &Rc110Panel::onMotorBattery, this));
    subscribers.push_back(handle.subscribe("motor_temperature", 1, &Rc110Panel::onMotorTemperature, this));
    subscribers.push_back(handle.subscribe("servo_temperature", 1, &Rc110Panel::onServoTemperature, this));
    subscribers.push_back(handle.subscribe("imu", 1, &Rc110Panel::onImu, this));
}

Rc110Panel::~Rc110Panel() = default;

QTreeWidgetItem* Rc110Panel::getTreeItem(TREE_ITEM_GROUP group, const char* name) const
{
    QTreeWidgetItem* item = nullptr;
    auto list = ui->treeWidget->findItems(name, Qt::MatchExactly | Qt::MatchRecursive);

    // if item exists, check if it is from required group
    for (auto candidate : list) {
        auto groupItem = candidate->parent();
        if (groupItem && treeItems.key(groupItem) == group) {
            item = list.front();
            break;
        }
    }
    // if it does not exist, add it
    if (!item) {
        item = new QTreeWidgetItem({name});
        treeItems[group]->addChild(item);
        treeItems[group]->setExpanded(true);
    }
    return item;
}

void Rc110Panel::onDriveStatus(const ackermann_msgs::AckermannDriveStamped& driveStatus)
{
    using namespace boost::math::float_constants;

    getTreeItem(DRIVE, "speed")->setText(1, QString("%1 m/s").arg(driveStatus.drive.speed));
    getTreeItem(DRIVE, "angle")->setText(1, QString("%1 °").arg(driveStatus.drive.steering_angle * radian));
}

void Rc110Panel::onMotorBattery(const sensor_msgs::BatteryState& batteryState)
{
    getTreeItem(BATTERY, "motor voltage")->setText(1, QString("%1 V").arg(batteryState.voltage));
}

void Rc110Panel::onMotorTemperature(const sensor_msgs::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "motor")->setText(1, QString::fromUtf8("%1 °C").arg(temperature.temperature));
}

void Rc110Panel::onServoTemperature(const sensor_msgs::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "servo")->setText(1, QString::fromUtf8("%1 °C").arg(temperature.temperature));
}

void Rc110Panel::onImu(const sensor_msgs::Imu& imu)
{
    getTreeItem(IMU, "x")->setText(1, QString::fromUtf8("%1 m/s^2").arg(imu.linear_acceleration.x));
    getTreeItem(IMU, "y")->setText(1, QString::fromUtf8("%1 m/s^2").arg(imu.linear_acceleration.y));
    getTreeItem(IMU, "z")->setText(1, QString::fromUtf8("%1 m/s^2").arg(imu.linear_acceleration.z));
    getTreeItem(IMU, "roll")->setText(1, QString::fromUtf8("%1 rad/s").arg(imu.angular_velocity.x));
    getTreeItem(IMU, "pitch")->setText(1, QString::fromUtf8("%1 rad/s").arg(imu.angular_velocity.y));
    getTreeItem(IMU, "yaw")->setText(1, QString::fromUtf8("%1 rad/s").arg(imu.angular_velocity.z));
}

}  // namespace zmp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zmp::Rc110Panel, rviz::Panel)