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

#include <std_srvs/SetBool.h>

#include <QStatusBar>
#include <boost/math/constants/constants.hpp>

#include "ui_rc110_panel.h"

namespace zmp
{
namespace
{
constexpr float RAD_TO_DEG = boost::math::float_constants::radian;
}

Rc110Panel::Rc110Panel(QWidget* parent) : Panel(parent), ui(new Ui::PanelWidget)
{
    ui->setupUi(this);

    QList<QTreeWidgetItem*> items;
    for (int i = 0; i < std::size(TREE_ITEM_GROUP_NAMES); ++i) {
        items.push_back(new QTreeWidgetItem({TREE_ITEM_GROUP_NAMES[i]}));
        treeItems.insert((TreeItemGroup)i, items[i]);
    }
    ui->treeWidget->insertTopLevelItems(0, items);

    statusBar = new QStatusBar(this);
    layout()->addWidget(statusBar);

    ui->splitter->setStretchFactor(0, 1);  // expand tree widget to maximum height

    connect(ui->boardButton, &QPushButton::clicked, this, &Rc110Panel::onEnableBoard);
    connect(ui->motorGroup,
            QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked),
            this,
            &Rc110Panel::onSetMotorState);
    connect(ui->servoGroup,
            QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked),
            this,
            &Rc110Panel::onSetServoState);

    subscribers.push_back(handle.subscribe("drive_status", 1, &Rc110Panel::onDriveStatus, this));
    subscribers.push_back(handle.subscribe("odometry", 1, &Rc110Panel::onOdometry, this));
    subscribers.push_back(handle.subscribe("servo_battery", 1, &Rc110Panel::onServoBattery, this));
    subscribers.push_back(handle.subscribe("motor_battery", 1, &Rc110Panel::onMotorBattery, this));
    subscribers.push_back(handle.subscribe("baseboard_temperature", 1, &Rc110Panel::onBaseboardTemperature, this));
    subscribers.push_back(handle.subscribe("servo_temperature", 1, &Rc110Panel::onServoTemperature, this));
    subscribers.push_back(handle.subscribe("imu", 1, &Rc110Panel::onImu, this));

    // ask initial states
    changeBoardState(EnabledState::ASK);
    onSetMotorState(nullptr);
    onSetServoState(nullptr);
    statusBar->showMessage("");
}

Rc110Panel::~Rc110Panel() = default;

QTreeWidgetItem* Rc110Panel::getTreeItem(TreeItemGroup group, const char* name) const
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

void Rc110Panel::onEnableBoard(bool on)
{
    changeBoardState(on ? EnabledState::ON : EnabledState::OFF);
}

void Rc110Panel::changeBoardState(EnabledState request)
{
    std_srvs::SetBool service;
    service.request.data = uint8_t(request);

    if (!ros::service::call("enable_board", service)) {
        service.response.success = false;
        service.response.message = "disabled";
    }

    ui->boardButton->setChecked(service.response.message == "enabled");
    statusBar->showMessage(service.response.success ? QString("Board was set to %1").arg(service.response.message.data())
                                                    : QString("Failed to set board state"),
                           5000);
}

void Rc110Panel::onSetMotorState(QAbstractButton* button)
{
    MotorState state = (button == ui->motorOffRadio)       ? MotorState::OFF
                       : (button == ui->motorOnRadio)      ? MotorState::ON
                       : (button == ui->motorNeutralRadio) ? MotorState::NEUTRAL
                                                           : MotorState::ASK;
    std_srvs::SetBool service;
    service.request.data = uint8_t(state);

    if (!ros::service::call("motor_state", service)) {
        service.response.success = false;
        service.response.message = "off";
    }

    if (service.response.message == "neutral") {
        ui->motorNeutralRadio->setChecked(true);
    } else if (service.response.message == "on") {
        ui->motorOnRadio->setChecked(true);
    } else {
        ui->motorOffRadio->setChecked(true);
    }

    statusBar->showMessage(service.response.success
                                   ? QString("Drive motor was set to %1").arg(service.response.message.data())
                                   : QString("Failed to set motor state"),
                           5000);
}

void Rc110Panel::onSetServoState(QAbstractButton* button)
{
    MotorState state = (button == ui->servoOffRadio)       ? MotorState::OFF
                       : (button == ui->servoOnRadio)      ? MotorState::ON
                       : (button == ui->servoNeutralRadio) ? MotorState::NEUTRAL
                                                           : MotorState::ASK;
    std_srvs::SetBool service;
    service.request.data = uint8_t(state);

    if (!ros::service::call("servo_state", service)) {
        service.response.success = false;
        service.response.message = "off";
    }

    if (service.response.message == "neutral") {
        ui->servoNeutralRadio->setChecked(true);
    } else if (service.response.message == "on") {
        ui->servoOnRadio->setChecked(true);
    } else {
        ui->servoOffRadio->setChecked(true);
    }

    statusBar->showMessage(service.response.success
                                   ? QString("Servomotor was set to %1").arg(service.response.message.data())
                                   : QString("Failed to set servo state"),
                           5000);
}

void Rc110Panel::onDriveStatus(const ackermann_msgs::AckermannDriveStamped& driveStatus)
{
    getTreeItem(DRIVE, "speed")->setText(1, QString("%1 m/s").arg(driveStatus.drive.speed));
    getTreeItem(DRIVE, "steering angle")->setText(1, QString("%1 °").arg(driveStatus.drive.steering_angle * RAD_TO_DEG));
    getTreeItem(DRIVE, "steering speed")
            ->setText(1, QString("%1 °/s").arg(driveStatus.drive.steering_angle_velocity * RAD_TO_DEG));
}

void Rc110Panel::onOdometry(const nav_msgs::Odometry& odometry)
{
    getTreeItem(DRIVE, "angle speed")->setText(1, QString("%1 °/s").arg(odometry.twist.twist.angular.z * RAD_TO_DEG));
}

void Rc110Panel::onServoBattery(const sensor_msgs::BatteryState& batteryState)
{
    getTreeItem(BATTERY, "servo voltage")->setText(1, QString("%1 V").arg(batteryState.voltage));
    getTreeItem(BATTERY, "servo current")->setText(1, QString("%1 mA").arg(batteryState.current * 1000));
}

void Rc110Panel::onMotorBattery(const sensor_msgs::BatteryState& batteryState)
{
    getTreeItem(BATTERY, "motor voltage")->setText(1, QString("%1 V").arg(batteryState.voltage));
    getTreeItem(BATTERY, "motor current")->setText(1, QString("%1 mA").arg(batteryState.current * 1000));
}

void Rc110Panel::onBaseboardTemperature(const sensor_msgs::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "baseboard")->setText(1, QString::fromUtf8("%1 °C").arg(temperature.temperature));
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
    getTreeItem(IMU, "yaw")->setText(1, QString::fromUtf8("%1 rad/s").arg(imu.angular_velocity.z));
}

}  // namespace zmp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zmp::Rc110Panel, rviz::Panel)