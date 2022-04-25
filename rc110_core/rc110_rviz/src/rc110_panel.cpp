/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_panel.hpp"

#include <rc110_msgs/SetInteger.h>
#include <std_srvs/SetBool.h>
#include <topic_tools/MuxSelect.h>

#include <QStatusBar>
#include <QTimer>
#include <boost/math/constants/constants.hpp>
#include <param_tools/param_tools.hpp>
#include <regex>

#include "ui_rc110_panel.h"

namespace zmp
{
namespace
{
constexpr float RAD_TO_DEG = boost::math::float_constants::radian;
constexpr float DEG_TO_RAD = boost::math::float_constants::degree;
constexpr float G_TO_MS2 = 9.8f;  // G to m/s2 conversion factor (Tokyo)

constexpr int STATUS_MESSAGE_TIME = 5000;  // ms

bool globalRobotSelected = false;

QString printSensor(float value, const QString& suffix, int precision = 2)
{
    return (value < 0 ? QString("%1 ").arg(value, -10, 'f', precision) : QString(" %1 ").arg(value, -9, 'f', precision)) +
           suffix;
}

QStringList getRobotNames()
{
    ros::V_string nodeNames;
    ros::master::getNodes(nodeNames);

    static const std::regex expression("/(.*)/tf_publisher$");

    QStringList robotNames;
    for (const auto& nodeName : nodeNames) {
        std::smatch match;
        if (std::regex_match(nodeName, match, expression); match.size()) {
            robotNames << match[1].str().c_str();
        }
    }
    return robotNames;
}
}  // namespace

Rc110Panel::Rc110Panel(QWidget* parent) :
        Panel(parent),
        ui(new Ui::PanelWidget),
        calibrationTimer(new QTimer(this)),
        rc(QString::fromStdString(ros::param::param("~rc", std::string())))
{
    ui->setupUi(this);
    calibrationTimer->setSingleShot(true);

    QList<QTreeWidgetItem*> items;
    for (int i = 0; i < std::size(TREE_ITEM_GROUP_NAMES); ++i) {
        items.push_back(new QTreeWidgetItem({TREE_ITEM_GROUP_NAMES[i]}));
        treeItems.insert((TreeItemGroup)i, items[i]);
    }
    ui->treeWidget->insertTopLevelItems(0, items);

    statusBar = new QStatusBar(this);
    layout()->addWidget(statusBar);

    ui->splitter->setStretchFactor(0, 1);  // expand tree widget to maximum height

    connect(ui->robotNameCombo, qOverload<int>(&QComboBox::activated), this, &Rc110Panel::onRobotNameChanged);
    connect(ui->robotSelectedButton, &QPushButton::clicked, this, &Rc110Panel::onRobotSelectedButton);
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
    connect(ui->steeringOffsetEdit, &QLineEdit::editingFinished, this, &Rc110Panel::onEditingFinished);
    connect(ui->calibrateButton, &QPushButton::clicked, this, &Rc110Panel::onCalibrate);
    connect(calibrationTimer, &QTimer::timeout, this, &Rc110Panel::onFinishCalibration);

    statusBar->showMessage("");
}

Rc110Panel::~Rc110Panel() = default;

void Rc110Panel::load(const rviz::Config& config)
{
    Panel::load(config);
    config.mapGetString("robot_name", &savedRobotName);

    // Subscribe to robot name parameter selection, and select it the first time if parameter was set.
    rcSubscriber = param_tools::instance().subscribe("/selected_rc", [this](const XmlRpc::XmlRpcValue& value) {
        setupRobotName(value);
    });

    startTimer(100);  // ms
}

void Rc110Panel::save(rviz::Config config) const
{
    Panel::save(config);
    if (rc.isEmpty()) {
        config.mapSetValue("robot_name", ui->robotNameCombo->currentText());
    }
}

void Rc110Panel::timerEvent(QTimerEvent* event)
{
    if (driveSpeed != 0) {
        publishDrive();  // prevent motor from stopping by timeout
    }

    if (++timerCounter % 10 == 0) {
        updateRobotNames();
    }
}

void Rc110Panel::updateRobotNames()
{
    // Either multiple robots in one window.
    auto newRobotNames = getRobotNames();
    if (robotNames != newRobotNames) {
        robotNames = newRobotNames;  // used anyway to select other robot
        trySelectingRobot();

        if (rc.isEmpty()) {
            auto currentName = ui->robotNameCombo->currentText();
            ui->robotNameCombo->clear();
            ui->robotNameCombo->addItems(robotNames);
            ui->robotNameCombo->setCurrentText(currentName);  // ignored if not in the list

            if (savedRobotName.size()) {
                ui->robotNameCombo->setCurrentText(savedRobotName);
                if (savedRobotName == ui->robotNameCombo->currentText()) {
                    savedRobotName.clear();
                }
            }
            onRobotNameChanged();
            return;
        }
    }
    // Or just one constant robot selected on start.
    if (!rc.isEmpty() && ui->robotNameCombo->count() == 0) {
        ui->robotNameCombo->addItem(rc);
        ui->robotNameCombo->setEnabled(false);
        onRobotNameChanged();
    }
}

void Rc110Panel::trySelectingRobot()
{
    // If robot was not selected, then try doing it from one of the panels.
    if (!globalRobotSelected) {
        globalRobotSelected = true;

        if (selectedRobot.isEmpty() && robotNames.size()) {
            auto firstRc = robotNames.front().toStdString();
            publishRobotName(firstRc);
        }
    }
}

void Rc110Panel::setupRobotName(const std::string& name)
{
    QString qName = name.c_str();
    if (selectedRobot != qName) {
        selectedRobot = qName;
        ui->robotSelectedButton->setChecked(selectedRobot == ui->robotNameCombo->currentText());
    }
}

void Rc110Panel::setupRosConnections()
{
    subscribers.clear();
    subscribers.push_back(handle.subscribe(ns + "/mux_drive/selected", 1, &Rc110Panel::onAdModeChanged, this));

    subscribers.push_back(handle.subscribe(ns + "/motor_speed_goal", 1, &Rc110Panel::onMotorSpeed, this));
    subscribers.push_back(handle.subscribe(ns + "/steering_angle_goal", 1, &Rc110Panel::onSteeringAngle, this));

    subscribers.push_back(handle.subscribe(ns + "/baseboard_error", 1, &Rc110Panel::onError, this));
    subscribers.push_back(handle.subscribe(ns + "/robot_status", 1, &Rc110Panel::onRobotStatus, this));
    subscribers.push_back(handle.subscribe(ns + "/drive_status", 1, &Rc110Panel::onDriveStatus, this));
    subscribers.push_back(handle.subscribe(ns + "/offsets_status", 1, &Rc110Panel::onOffsets, this));

    subscribers.push_back(handle.subscribe(ns + "/odometry", 1, &Rc110Panel::onOdometry, this));
    subscribers.push_back(handle.subscribe(ns + "/servo_battery", 1, &Rc110Panel::onServoBattery, this));
    subscribers.push_back(handle.subscribe(ns + "/motor_battery", 1, &Rc110Panel::onMotorBattery, this));
    subscribers.push_back(handle.subscribe(ns + "/baseboard_temperature", 1, &Rc110Panel::onBaseboardTemperature, this));
    subscribers.push_back(handle.subscribe(ns + "/servo_temperature", 1, &Rc110Panel::onServoTemperature, this));
    subscribers.push_back(handle.subscribe(ns + "/imu/data", 1, &Rc110Panel::onImu, this));
    subscribers.push_back(handle.subscribe(ns + "/motor_rate", 1, &Rc110Panel::onMotorRate, this));
    subscribers.push_back(handle.subscribe(ns + "/wheel_speeds", 1, &Rc110Panel::onWheelSpeeds, this));

    publishers["drive_manual"] = handle.advertise<ackermann_msgs::AckermannDriveStamped>(ns + "/drive_manual", 1);
    publishers["offsets"] = handle.advertise<rc110_msgs::Offsets>(ns + "/offsets", 1);
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

void Rc110Panel::onRobotSelectedButton(bool on)
{
    auto currentRobot = ui->robotNameCombo->currentText();
    if (on) {
        if (selectedRobot != currentRobot) {
            selectedRobot = std::move(currentRobot);
            publishRobotName(selectedRobot.toStdString());
        }
    } else {
        if (selectedRobot == currentRobot) {
            for (const auto& robot : robotNames) {
                if (selectedRobot != robot) {
                    selectedRobot = robot;
                    publishRobotName(selectedRobot.toStdString());
                    break;
                }
            }
        }
    }
}

void Rc110Panel::onEnableBoard(bool on)
{
    std_srvs::SetBool service;
    service.request.data = uint8_t(on);
    ros::service::call(ns + "/enable_board", service);

    statusBar->showMessage(service.response.success ? QString("Board was set to %1").arg(on ? "on" : "off")
                                                    : QString("Failed to set board state"),
                           STATUS_MESSAGE_TIME);
}

void Rc110Panel::onEnableAd(bool on)
{
    topic_tools::MuxSelect service;
    service.request.topic = on ? "drive_ad" : "drive_manual";

    if (ros::service::call(ns + "/mux_drive/select", service)) {
        statusBar->showMessage(on ? "AD enabled" : "Joystick enabled", STATUS_MESSAGE_TIME);
    } else {
        statusBar->showMessage("Failed to switch AD mode", STATUS_MESSAGE_TIME);
    }
}

void Rc110Panel::onSetMotorState(QAbstractButton* button)
{
    int state = (button == ui->motorOnRadio)        ? rc110_msgs::Status::MOTOR_ON
                : (button == ui->motorNeutralRadio) ? rc110_msgs::Status::MOTOR_NEUTRAL
                                                    : rc110_msgs::Status::MOTOR_OFF;
    rc110_msgs::SetInteger service;
    service.request.data = state;
    ros::service::call(ns + "/motor_state", service);

    statusBar->showMessage(service.response.success ? QString("Drive motor was set to %1")
                                                              .arg(state == rc110_msgs::Status::MOTOR_ON    ? "on"
                                                                   : state == rc110_msgs::Status::MOTOR_OFF ? "off"
                                                                                                            : "neutral")
                                                    : QString("Failed to set motor state"),
                           STATUS_MESSAGE_TIME);
}

void Rc110Panel::onSetServoState(QAbstractButton* button)
{
    int state = (button == ui->servoOnRadio)        ? rc110_msgs::Status::MOTOR_ON
                : (button == ui->servoNeutralRadio) ? rc110_msgs::Status::MOTOR_NEUTRAL
                                                    : rc110_msgs::Status::MOTOR_OFF;
    rc110_msgs::SetInteger service;
    service.request.data = state;
    ros::service::call(ns + "/servo_state", service);

    statusBar->showMessage(service.response.success ? QString("Servomotor was set to %1")
                                                              .arg(state == rc110_msgs::Status::MOTOR_ON    ? "on"
                                                                   : state == rc110_msgs::Status::MOTOR_OFF ? "off"
                                                                                                            : "neutral")
                                                    : QString("Failed to set servo state"),
                           STATUS_MESSAGE_TIME);
}

void Rc110Panel::onRobotNameChanged(int)
{
    auto name = ui->robotNameCombo->currentText();
    auto newNs = name.isEmpty() ? "" : "/" + name.toStdString();
    if (ns != newNs) {
        ns = newNs;
        setupRosConnections();
    }
    ui->robotSelectedButton->setChecked(selectedRobot == name);
}

void Rc110Panel::onEditingFinished()
{
    if (auto edit = dynamic_cast<QLineEdit*>(sender())) {
        auto name = edit->objectName().toStdString();

        if (name == "driveSpeedEdit") {
            driveSpeed = edit->text().toFloat();
            publishDrive();
        } else if (name == "steeringEdit") {
            steeringAngle = edit->text().toFloat();
            publishDrive();
        } else if (name == "steeringOffsetEdit") {
            offsets.steering = ui->steeringOffsetEdit->text().toFloat();
            publishOffsets();
        }
    }
}

void Rc110Panel::onCalibrate()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    ui->calibrateButton->setEnabled(false);

    calibrationSums.clear();
    calibrationTimer->start(3000);
}

void Rc110Panel::onFinishCalibration()
{
    const auto& motorCurrent = calibrationSums["motor current"];
    if (motorCurrent.first) {
        offsets.motor_current += motorCurrent.second / motorCurrent.first;
    }
    const auto& accelX = calibrationSums["accel x"];
    if (accelX.first) {
        offsets.accel_x += accelX.second / accelX.first;
    }
    const auto& accelY = calibrationSums["accel y"];
    if (accelY.first) {
        offsets.accel_y += accelY.second / accelY.first;
    }
    const auto& accelZ = calibrationSums["accel z"];
    if (accelZ.first) {
        offsets.accel_z += accelZ.second / accelZ.first - G_TO_MS2;
    }
    const auto& gyroYaw = calibrationSums["gyro yaw"];
    if (gyroYaw.first) {
        offsets.gyro += gyroYaw.second / gyroYaw.first;
    }
    publishOffsets();

    ui->calibrateButton->setEnabled(true);
    QApplication::restoreOverrideCursor();
}

void Rc110Panel::publishRobotName(const std::string& name)
{
    param_tools::instance().publish("/selected_rc", name);
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
    publishers["offsets"].publish(offsets);
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
    statusBar->showMessage(
            QString("Drive was updated. Speed: %1, Angle: %2").arg(ui->driveSpeedEdit->text(), ui->steeringEdit->text()),
            STATUS_MESSAGE_TIME);
}

void Rc110Panel::onError(const rc110_msgs::BaseboardError& message)
{
    int error = message.data;
    if (error == rc110_msgs::BaseboardError::NONE) {
        ui->errorLabel->setPixmap(QPixmap(":/ok.png"));
        ui->errorLabel->setToolTip("Baseboard is ok");
    } else {
        ui->errorLabel->setPixmap(QPixmap(":/error.png"));

        QString tooltip = "Please, restart baseboard: %1";
        if (error == rc110_msgs::BaseboardError::BOARD_HEAT) {
            ui->errorLabel->setToolTip(tooltip.arg("board has too high temperature"));
        } else if (error == rc110_msgs::BaseboardError::MOTOR_HEAT) {
            ui->errorLabel->setToolTip(tooltip.arg("motor has too high temperature"));
        } else if (error == rc110_msgs::BaseboardError::MOTOR_FAILURE) {
            ui->errorLabel->setToolTip(tooltip.arg("encoder feedback and polarity of the motor"));
        } else if (error == rc110_msgs::BaseboardError::LOW_VOLTAGE) {
            ui->errorLabel->setToolTip(tooltip.arg("voltage dropped less than 6V for around 1s"));
        }
    }
}

void Rc110Panel::onRobotStatus(const rc110_msgs::Status& message)
{
    ui->boardButton->setChecked(message.board_enabled);

    switch (message.motor_state) {
        case rc110_msgs::Status::MOTOR_NEUTRAL:
            ui->motorNeutralRadio->setChecked(true);
            break;
        case rc110_msgs::Status::MOTOR_ON:
            ui->motorOnRadio->setChecked(true);
            break;
        default:
            ui->motorOffRadio->setChecked(true);
    }

    switch (message.servo_state) {
        case rc110_msgs::Status::MOTOR_NEUTRAL:
            ui->servoNeutralRadio->setChecked(true);
            break;
        case rc110_msgs::Status::MOTOR_ON:
            ui->servoOnRadio->setChecked(true);
            break;
        default:
            ui->servoOffRadio->setChecked(true);
    }
}

void Rc110Panel::onDriveStatus(const ackermann_msgs::AckermannDriveStamped& driveStatus)
{
    getTreeItem(DRIVE, "speed")->setText(1, printSensor(driveStatus.drive.speed, "m/s"));
    getTreeItem(DRIVE, "steering angle")->setText(1, printSensor(driveStatus.drive.steering_angle * RAD_TO_DEG, "°"));
    getTreeItem(DRIVE, "steering speed")
            ->setText(1, printSensor(driveStatus.drive.steering_angle_velocity * RAD_TO_DEG, "°"));
}

void Rc110Panel::onOffsets(const rc110_msgs::Offsets& message)
{
    offsets = message;
    QString text = "Motor Current:\t %1\n"
                   "Accel X:\t\t %2\n"
                   "Accel Y:\t\t %3\n"
                   "Accel Z:\t\t %4\n"
                   "Gyro Yaw:\t %5\n";
    ui->offsetsLabel->setText(text.arg(printSensor(message.motor_current * 1000, "mA", 4),
                                       printSensor(message.accel_x, "m/s²", 4),
                                       printSensor(message.accel_y, "m/s²", 4),
                                       printSensor(message.accel_z, "m/s²", 4),
                                       printSensor(message.gyro * RAD_TO_DEG, "°/s", 4)));

    ui->steeringOffsetEdit->setText(QString::number(message.steering));

    statusBar->showMessage(QString("Offsets updated."), STATUS_MESSAGE_TIME);
}

void Rc110Panel::onOdometry(const nav_msgs::Odometry& odometry)
{
    getTreeItem(DRIVE, "angular velocity")
            ->setText(1, printSensor(float(odometry.twist.twist.angular.z * RAD_TO_DEG), "°/s"));
}

void Rc110Panel::onServoBattery(const sensor_msgs::BatteryState& batteryState)
{
    getTreeItem(BATTERY, "servo voltage")->setText(1, printSensor(batteryState.voltage, "V"));
    getTreeItem(BATTERY, "servo current")->setText(1, printSensor(batteryState.current * 1000, "mA", 0));
}

void Rc110Panel::onMotorBattery(const sensor_msgs::BatteryState& batteryState)
{
    getTreeItem(BATTERY, "motor voltage")->setText(1, printSensor(batteryState.voltage, "V"));
    getTreeItem(BATTERY, "motor current")->setText(1, printSensor(batteryState.current * 1000, "mA", 0));

    float voltage = batteryState.voltage;
    if (voltage < 6.4f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_0.png"));
    } else if (voltage < 7.0f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_1.png"));
    } else if (voltage < 7.4f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_2.png"));
    } else if (voltage < 7.6f) {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_3.png"));
    } else {
        ui->batteryLabel->setPixmap(QPixmap(":/battery_4.png"));
    }

    if (calibrationTimer->isActive()) {
        auto& value = calibrationSums["motor current"];
        ++value.first;
        value.second += batteryState.current;
    }
}

void Rc110Panel::onBaseboardTemperature(const sensor_msgs::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "baseboard")->setText(1, printSensor(temperature.temperature, "°C", 1));
}

void Rc110Panel::onServoTemperature(const sensor_msgs::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "servo")->setText(1, printSensor(temperature.temperature, "°C", 1));
}

void Rc110Panel::onImu(const sensor_msgs::Imu& imu)
{
    getTreeItem(IMU, "accel x")->setText(1, printSensor(imu.linear_acceleration.x, "m/s²"));
    getTreeItem(IMU, "accel y")->setText(1, printSensor(imu.linear_acceleration.y, "m/s²"));
    getTreeItem(IMU, "accel z")->setText(1, printSensor(imu.linear_acceleration.z, "m/s²"));
    getTreeItem(IMU, "gyro yaw")->setText(1, printSensor(imu.angular_velocity.z * RAD_TO_DEG, "°/s"));

    if (calibrationTimer->isActive()) {
        auto& accelX = calibrationSums["accel x"];
        ++accelX.first;
        accelX.second += imu.linear_acceleration.x;
        auto& accelY = calibrationSums["accel y"];
        ++accelY.first;
        accelY.second += imu.linear_acceleration.y;
        auto& accelZ = calibrationSums["accel z"];
        ++accelZ.first;
        accelZ.second += imu.linear_acceleration.z;
        auto& gyroYaw = calibrationSums["gyro yaw"];
        ++gyroYaw.first;
        gyroYaw.second += imu.angular_velocity.z;
    }
}

void Rc110Panel::onMotorRate(const rc110_msgs::MotorRate& motorRate)
{
    getTreeItem(OTHER, "motor rate")->setText(1, printSensor(motorRate.motor_rate, "cycles/s"));
    getTreeItem(OTHER, "estimated speed")->setText(1, printSensor(motorRate.estimated_speed, "m/s"));
}

void Rc110Panel::onWheelSpeeds(const rc110_msgs::WheelSpeeds& wheelSpeeds)
{
    getTreeItem(OTHER, "wheel speed FL")->setText(1, printSensor(wheelSpeeds.speed_fl, "m/s"));
    getTreeItem(OTHER, "wheel speed FR")->setText(1, printSensor(wheelSpeeds.speed_fr, "m/s"));
    getTreeItem(OTHER, "wheel speed RL")->setText(1, printSensor(wheelSpeeds.speed_rl, "m/s"));
    getTreeItem(OTHER, "wheel speed RR")->setText(1, printSensor(wheelSpeeds.speed_rr, "m/s"));
}
}  // namespace zmp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zmp::Rc110Panel, rviz::Panel)