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
#include <topic_tools/MuxSelect.h>

#include <QStatusBar>
#include <boost/math/constants/constants.hpp>

#include "ui_rc110_panel.h"

namespace zmp
{
namespace
{
constexpr float RAD_TO_DEG = boost::math::float_constants::radian;
constexpr float DEG_TO_RAD = boost::math::float_constants::degree;

constexpr int STATUS_MESSAGE_TIME = 5000;  // ms
}  // namespace

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
    connect(ui->adButton, &QPushButton::clicked, this, &Rc110Panel::onEnableAd);
    connect(ui->motorGroup,
            QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked),
            this,
            &Rc110Panel::onSetMotorState);
    connect(ui->servoGroup,
            QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked),
            this,
            &Rc110Panel::onSetServoState);
    connect(ui->driveSpeedEdit, &QLineEdit::editingFinished, this, &Rc110Panel::onEditingFinished);
    connect(ui->steeringEdit, &QLineEdit::editingFinished, this, &Rc110Panel::onEditingFinished);
    connect(ui->gyroOffsetEdit, &QLineEdit::editingFinished, this, &Rc110Panel::onEditingFinished);
    connect(ui->motorCurrentOffsetEdit, &QLineEdit::editingFinished, this, &Rc110Panel::onEditingFinished);
    connect(ui->steeringOffsetEdit, &QLineEdit::editingFinished, this, &Rc110Panel::onEditingFinished);

    subscribers.push_back(handle.subscribe("mux_drive/selected", 1, &Rc110Panel::onAdModeChanged, this));

    subscribers.push_back(handle.subscribe("motor_speed_goal", 1, &Rc110Panel::onMotorSpeed, this));
    subscribers.push_back(handle.subscribe("steering_angle_goal", 1, &Rc110Panel::onSteeringAngle, this));

    subscribers.push_back(handle.subscribe("baseboard_error", 1, &Rc110Panel::onError, this));
    subscribers.push_back(handle.subscribe("robot_status", 1, &Rc110Panel::onRobotStatus, this));
    subscribers.push_back(handle.subscribe("drive_status", 1, &Rc110Panel::onDriveStatus, this));
    subscribers.push_back(handle.subscribe("offsets_status", 1, &Rc110Panel::onOffsets, this));

    subscribers.push_back(handle.subscribe("odometry", 1, &Rc110Panel::onOdometry, this));
    subscribers.push_back(handle.subscribe("servo_battery", 1, &Rc110Panel::onServoBattery, this));
    subscribers.push_back(handle.subscribe("motor_battery", 1, &Rc110Panel::onMotorBattery, this));
    subscribers.push_back(handle.subscribe("baseboard_temperature", 1, &Rc110Panel::onBaseboardTemperature, this));
    subscribers.push_back(handle.subscribe("servo_temperature", 1, &Rc110Panel::onServoTemperature, this));
    subscribers.push_back(handle.subscribe("imu/data_raw", 1, &Rc110Panel::onImu, this));

    publishers["drive_manual"] = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_manual", 1);
    publishers["offsets"] = handle.advertise<rc110_msgs::Offsets>("offsets", 1);

    statusBar->showMessage("");

    startTimer(100);  // ms
}

Rc110Panel::~Rc110Panel() = default;

void Rc110Panel::timerEvent(QTimerEvent* event)
{
    if (driveSpeed != 0) {
        publishDrive();  // prevent motor from stopping by timeout
    }
}

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
    std_srvs::SetBool service;
    service.request.data = uint8_t(on);
    ros::service::call("enable_board", service);

    statusBar->showMessage(service.response.success ? QString("Board was set to %1").arg(on ? "on" : "off")
                                                    : QString("Failed to set board state"),
                           STATUS_MESSAGE_TIME);
}

void Rc110Panel::onEnableAd(bool on)
{
    topic_tools::MuxSelect service;
    service.request.topic = on ? "drive_ad" : "drive_manual";

    if (ros::service::call("mux_drive/select", service)) {
        statusBar->showMessage(on ? "AD enabled" : "Joystick enabled", STATUS_MESSAGE_TIME);
    } else {
        statusBar->showMessage("Failed to switch AD mode", STATUS_MESSAGE_TIME);
    }
}

void Rc110Panel::onSetMotorState(QAbstractButton* button)
{
    MotorState state = (button == ui->motorOnRadio)        ? MotorState::ON
                       : (button == ui->motorNeutralRadio) ? MotorState::NEUTRAL
                                                           : MotorState::OFF;
    std_srvs::SetBool service;
    service.request.data = uint8_t(state);
    ros::service::call("motor_state", service);

    statusBar->showMessage(service.response.success ? QString("Drive motor was set to %1")
                                                              .arg(state == MotorState::ON    ? "on"
                                                                   : state == MotorState::OFF ? "off"
                                                                                              : "neutral")
                                                    : QString("Failed to set motor state"),
                           STATUS_MESSAGE_TIME);
}

void Rc110Panel::onSetServoState(QAbstractButton* button)
{
    MotorState state = (button == ui->servoOnRadio)        ? MotorState::ON
                       : (button == ui->servoNeutralRadio) ? MotorState::NEUTRAL
                                                           : MotorState::OFF;
    std_srvs::SetBool service;
    service.request.data = uint8_t(state);
    ros::service::call("servo_state", service);

    statusBar->showMessage(service.response.success ? QString("Servomotor was set to %1")
                                                              .arg(state == MotorState::ON    ? "on"
                                                                   : state == MotorState::OFF ? "off"
                                                                                              : "neutral")
                                                    : QString("Failed to set servo state"),
                           STATUS_MESSAGE_TIME);
}

void Rc110Panel::onEditingFinished()
{
    if (auto edit = dynamic_cast<QLineEdit*>(sender())) {
        float value = edit->text().toFloat();

        auto name = edit->objectName().toStdString();
        if (name == "driveSpeedEdit") {
            driveSpeed = value;
            publishDrive();
        } else if (name == "steeringEdit") {
            steeringAngle = value;
            publishDrive();
        } else {
            publishOffsets();
        }
    }
}

void Rc110Panel::publishDrive()
{
    ackermann_msgs::AckermannDriveStamped message;
    message.header.stamp = ros::Time::now();
    message.drive.speed = driveSpeed;
    message.drive.steering_angle = steeringAngle * DEG_TO_RAD;
    publishers["drive_manual"].publish(message);
}

void Rc110Panel::publishOffsets()
{
    rc110_msgs::Offsets message;
    message.gyro = ui->gyroOffsetEdit->text().toFloat();
    message.motor_current = ui->motorCurrentOffsetEdit->text().toFloat();
    message.steering = ui->steeringOffsetEdit->text().toFloat();
    publishers["offsets"].publish(message);
}

void Rc110Panel::onAdModeChanged(const std_msgs::String& message)
{
    ui->adButton->setChecked(message.data == "drive_ad");
}

void Rc110Panel::onMotorSpeed(const std_msgs::Float32& message)
{
    ui->driveSpeedEdit->setText(QString::number(message.data));
    showDriveGoalStatus();
}

void Rc110Panel::onSteeringAngle(const std_msgs::Float32& message)
{
    ui->steeringEdit->setText(QString::number(message.data * RAD_TO_DEG));
    showDriveGoalStatus();
}

void Rc110Panel::showDriveGoalStatus()
{
    statusBar->showMessage(QString("Drive was updated. Speed: %1, Angle: %2")
                                   .arg(ui->driveSpeedEdit->text())
                                   .arg(ui->steeringEdit->text()),
                           STATUS_MESSAGE_TIME);
}

void Rc110Panel::onError(const std_msgs::UInt8& message)
{
    BaseboardError error{message.data};
    if (error == BaseboardError::NONE) {
        ui->errorLabel->setPixmap(QPixmap(":/ok.png"));
        ui->errorLabel->setToolTip("Baseboard is ok");
    } else {
        ui->errorLabel->setPixmap(QPixmap(":/error.png"));

        QString tooltip = "Please, restart baseboard: %1";
        if (error == BaseboardError::BOARD_HEAT) {
            ui->errorLabel->setToolTip(tooltip.arg("board has too high temperature"));
        } else if (error == BaseboardError::MOTOR_HEAT) {
            ui->errorLabel->setToolTip(tooltip.arg("motor has too high temperature"));
        } else if (error == BaseboardError::MOTOR_FAILURE) {
            ui->errorLabel->setToolTip(tooltip.arg("encoder feedback and polarity of the motor"));
        } else if (error == BaseboardError::LOW_VOLTAGE) {
            ui->errorLabel->setToolTip(tooltip.arg("voltage dropped less than 6V for around 1s"));
        }
    }
}

void Rc110Panel::onRobotStatus(const rc110_msgs::Status& message)
{
    ui->boardButton->setChecked(message.board_enabled);

    switch (MotorState(message.motor_state)) {
        case MotorState::NEUTRAL:
            ui->motorNeutralRadio->setChecked(true);
            break;
        case MotorState::ON:
            ui->motorOnRadio->setChecked(true);
            break;
        default:
            ui->motorOffRadio->setChecked(true);
    }

    switch (MotorState(message.servo_state)) {
        case MotorState::NEUTRAL:
            ui->servoNeutralRadio->setChecked(true);
            break;
        case MotorState::ON:
            ui->servoOnRadio->setChecked(true);
            break;
        default:
            ui->servoOffRadio->setChecked(true);
    }
}

void Rc110Panel::onDriveStatus(const ackermann_msgs::AckermannDriveStamped& driveStatus)
{
    getTreeItem(DRIVE, "speed")->setText(1, QString("%1 m/s").arg(driveStatus.drive.speed));
    getTreeItem(DRIVE, "steering angle")->setText(1, QString("%1 °").arg(driveStatus.drive.steering_angle * RAD_TO_DEG));
    getTreeItem(DRIVE, "steering speed")
            ->setText(1, QString("%1 °/s").arg(driveStatus.drive.steering_angle_velocity * RAD_TO_DEG));
}

void Rc110Panel::onOffsets(const rc110_msgs::Offsets& message)
{
    ui->gyroOffsetEdit->setText(QString::number(message.gyro));
    ui->motorCurrentOffsetEdit->setText(QString::number(message.motor_current));
    ui->steeringOffsetEdit->setText(QString::number(message.steering));

    statusBar->showMessage(
            QString("Offsets updated: %1, %2, %3").arg(message.gyro).arg(message.motor_current).arg(message.steering),
            STATUS_MESSAGE_TIME);
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

    float voltage = batteryState.voltage;
    if (voltage < 6.f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_0.png"));
    } else if (voltage < 6.5f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_1.png"));
    } else if (voltage < 7.0f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_2.png"));
    } else if (voltage < 7.5f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_3.png"));
    } else {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_4.png"));
    }
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