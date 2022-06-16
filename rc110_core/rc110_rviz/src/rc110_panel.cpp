/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_panel.hpp"

#include <QStatusBar>
#include <QTimer>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <regex>
#include <rviz_common/display_context.hpp>

#include "ui_rc110_panel.h"

using namespace std::chrono_literals;

namespace zmp
{
namespace
{
constexpr float RAD_TO_DEG = boost::math::float_constants::radian;
constexpr float DEG_TO_RAD = boost::math::float_constants::degree;
constexpr float G_TO_MS2 = 9.8f;  // G to m/s2 conversion factor (Tokyo)

constexpr int STATUS_MESSAGE_TIME = 5000;  // ms
constexpr int SERVICE_WAIT_TIME = 500;     // ms
constexpr int TIMER_PERIOD = 100;          // ms

bool globalRobotSelected = false;

QString printSensor(float value, const QString& suffix, int precision = 2)
{
    return (value < 0 ? QString("%1 ").arg(value, -10, 'f', precision) : QString(" %1 ").arg(value, -9, 'f', precision)) +
           suffix;
}

QStringList getRobotNames(rclcpp::Node* node)
{
    std::vector<std::string> nodeNames;
    nodeNames = node->get_node_names();

    static const std::regex expression("/(.*)/drive_control$");

    QStringList robotNames;
    for (const auto& nodeName : nodeNames) {
        std::smatch match;
        if (std::regex_match(nodeName, match, expression); match.size()) {
            robotNames << match[1].str().c_str();
        }
    }
    return robotNames;
}

std::chrono::system_clock::time_point toTimePoint(const rclcpp::Time& time)
{
    return std::chrono::system_clock::time_point(std::chrono::nanoseconds(time.nanoseconds()));
}
}  // namespace

Rc110Panel::Rc110Panel(QWidget* parent) : Panel(parent), ui(new Ui::PanelWidget), calibrationTimer(new QTimer(this))
{
    ui->setupUi(this);
    calibrationTimer->setSingleShot(true);

    QList<QTreeWidgetItem*> items;
    for (size_t i = 0; i < std::size(TREE_ITEM_GROUP_NAMES); ++i) {
        items.push_back(new QTreeWidgetItem({TREE_ITEM_GROUP_NAMES[i]}));
        treeItems.insert((TreeItemGroup)i, items[i]);
    }
    ui->treeWidget->insertTopLevelItems(0, items);

    statusBar = new QStatusBar(this);
    layout()->addWidget(statusBar);

    ui->joyLabel->setVisible(false);
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

Rc110Panel::~Rc110Panel()
{
    node->undeclare_parameter("rc");
}

void Rc110Panel::load(const rviz_common::Config& config)
{
    Panel::load(config);
    config.mapGetString("robot_name", &savedRobotName);
}

void Rc110Panel::save(rviz_common::Config config) const
{
    Panel::save(config);
    if (rc.isEmpty()) {
        config.mapSetValue("robot_name", ui->robotNameCombo->currentText());
    }
}

void Rc110Panel::onInitialize()
{
    // display context available starting from this function
    node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;  // needed for undeclare_parameter
    rc = QString::fromStdString(node->declare_parameter("rc", std::string(), descriptor));

    // Receive and pass current RoboCar selected in RViz.
    rcSubscriber = node->create_subscription<std_msgs::msg::String>(
            "rviz_rc", 1, [this](const std_msgs::msg::String::ConstSharedPtr& name) { setupRobotName(name->data); });
    publishers["rviz_rc"] = node->create_publisher<std_msgs::msg::String>(
            "rviz_rc", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

    startTimer(TIMER_PERIOD);
}

void Rc110Panel::timerEvent(QTimerEvent*)
{
    if (driveSpeed != 0) {
        publishDrive();  // prevent motor from stopping by timeout
    }
    updateJoystickIcon();

    if (++timerCounter % (1000 / TIMER_PERIOD) == 0) {  // 100 ms period
        updateRobotNames();
    }

    auto waitingLimit = toTimePoint(node->now() - rclcpp::Duration::from_seconds(SERVICE_WAIT_TIME / 1000.0));
    auto prunedRequests = std::vector<int64_t>();
    if (teleopStatusService && teleopStatusService->prune_requests_older_than(waitingLimit, &prunedRequests)) {
        statusBar->showMessage("Timeout on service /teleop_status", STATUS_MESSAGE_TIME);
    }
    if (enableBoardService && enableBoardService->prune_requests_older_than(waitingLimit, &prunedRequests)) {
        statusBar->showMessage("Timeout on service /enable_board", STATUS_MESSAGE_TIME);
    }
    if (muxDriveService && muxDriveService->prune_requests_older_than(waitingLimit, &prunedRequests)) {
        statusBar->showMessage("Timeout on service /mux_drive/select", STATUS_MESSAGE_TIME);
    }
    if (motorStateService && motorStateService->prune_requests_older_than(waitingLimit, &prunedRequests)) {
        statusBar->showMessage("Timeout on service /motor_state", STATUS_MESSAGE_TIME);
    }
    if (servoStateService && servoStateService->prune_requests_older_than(waitingLimit, &prunedRequests)) {
        statusBar->showMessage("Timeout on service /servo_state", STATUS_MESSAGE_TIME);
    }
}

void Rc110Panel::updateRobotNames()
{
    // Either multiple robots in one window.
    auto newRobotNames = getRobotNames(node.get());
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

void Rc110Panel::updateJoystickIcon()
{
    if (!teleopStatusService) return;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    teleopStatusService->async_send_request(request,
                                            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
                                                ui->joyLabel->setVisible(response.valid() && response.get()->success);
                                            });
}

void Rc110Panel::setupRosConnections()
{
    using std::placeholders::_1;

    subscribers.clear();
    subscribers.push_back(node->create_subscription<std_msgs::msg::String>(
            ns + "/mux_drive/selected", 1, std::bind(&Rc110Panel::onAdModeChanged, this, _1)));

    subscribers.push_back(node->create_subscription<std_msgs::msg::Float32>(
            ns + "/motor_speed_goal", 1, std::bind(&Rc110Panel::onMotorSpeed, this, _1)));
    subscribers.push_back(node->create_subscription<std_msgs::msg::Float32>(
            ns + "/steering_angle_goal", 1, std::bind(&Rc110Panel::onSteeringAngle, this, _1)));

    subscribers.push_back(node->create_subscription<rc110_msgs::msg::BaseboardError>(
            ns + "/baseboard_error",
            rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal),
            std::bind(&Rc110Panel::onError, this, _1)));
    subscribers.push_back(node->create_subscription<rc110_msgs::msg::Status>(
            ns + "/robot_status", 1, std::bind(&Rc110Panel::onRobotStatus, this, _1)));
    subscribers.push_back(node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            ns + "/drive_status", 1, std::bind(&Rc110Panel::onDriveStatus, this, _1)));
    subscribers.push_back(node->create_subscription<rc110_msgs::msg::Offsets>(
            ns + "/offsets_status", 1, std::bind(&Rc110Panel::onOffsets, this, _1)));

    subscribers.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
            ns + "/odometry", 1, std::bind(&Rc110Panel::onOdometry, this, _1)));
    subscribers.push_back(node->create_subscription<sensor_msgs::msg::BatteryState>(
            ns + "/servo_battery", 1, std::bind(&Rc110Panel::onServoBattery, this, _1)));
    subscribers.push_back(node->create_subscription<sensor_msgs::msg::BatteryState>(
            ns + "/motor_battery", 1, std::bind(&Rc110Panel::onMotorBattery, this, _1)));
    subscribers.push_back(node->create_subscription<sensor_msgs::msg::Temperature>(
            ns + "/baseboard_temperature", 1, std::bind(&Rc110Panel::onBaseboardTemperature, this, _1)));
    subscribers.push_back(node->create_subscription<sensor_msgs::msg::Temperature>(
            ns + "/servo_temperature", 1, std::bind(&Rc110Panel::onServoTemperature, this, _1)));
    subscribers.push_back(node->create_subscription<sensor_msgs::msg::Imu>(ns + "/imu/data_raw",
                                                                           1,
                                                                           std::bind(&Rc110Panel::onImu, this, _1)));
    subscribers.push_back(node->create_subscription<rc110_msgs::msg::MotorRate>(
            ns + "/motor_rate", 1, std::bind(&Rc110Panel::onMotorRate, this, _1)));
    subscribers.push_back(node->create_subscription<rc110_msgs::msg::WheelSpeeds>(
            ns + "/wheel_speeds", 1, std::bind(&Rc110Panel::onWheelSpeeds, this, _1)));

    publishers["drive_manual"] = node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            ns + "/drive_manual", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["offsets"] = node->create_publisher<rc110_msgs::msg::Offsets>(
            ns + "/offsets", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

    teleopStatusService = node->create_client<std_srvs::srv::Trigger>(ns + "/teleop_status");
    enableBoardService = node->create_client<std_srvs::srv::SetBool>(ns + "/enable_board");
    muxDriveService = node->create_client<rc110_topic_tools::srv::MuxSelect>(ns + "/mux_drive/select");
    motorStateService = node->create_client<rc110_msgs::srv::SetInteger>(ns + "/motor_state");
    servoStateService = node->create_client<rc110_msgs::srv::SetInteger>(ns + "/servo_state");
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
    if (!enableBoardService) return;

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = uint8_t(on);  // toggle board
    enableBoardService->async_send_request(
            request, [this, on](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) {
                statusBar->showMessage(response.valid() && response.get()->success
                                               ? QString("Board was set to %1").arg(on ? "on" : "off")
                                               : QString("Failed to set board state"),
                                       STATUS_MESSAGE_TIME);
            });
}

void Rc110Panel::onEnableAd(bool on)
{
    if (!muxDriveService) return;

    auto request = std::make_shared<rc110_topic_tools::srv::MuxSelect::Request>();
    request->topic = on ? "drive_ad" : "drive_manual";
    muxDriveService->async_send_request(
            request, [this, on](rclcpp::Client<rc110_topic_tools::srv::MuxSelect>::SharedFuture response) {
                statusBar->showMessage(response.valid() && response.get()->success
                                               ? QString(on ? "AD enabled" : "Joystick enabled")
                                               : QString("Failed to switch AD mode"),
                                       STATUS_MESSAGE_TIME);
            });
}

void Rc110Panel::onSetMotorState(QAbstractButton* button)
{
    if (!motorStateService) return;

    int state = (button == ui->motorOnRadio)        ? rc110_msgs::msg::Status::MOTOR_ON
                : (button == ui->motorNeutralRadio) ? rc110_msgs::msg::Status::MOTOR_NEUTRAL
                                                    : rc110_msgs::msg::Status::MOTOR_OFF;

    auto request = std::make_shared<rc110_msgs::srv::SetInteger::Request>();
    request->data = state;
    motorStateService->async_send_request(
            request, [this, state](rclcpp::Client<rc110_msgs::srv::SetInteger>::SharedFuture response) {
                statusBar->showMessage(response.valid() && response.get()->success
                                               ? QString("Drive motor was set to %1")
                                                         .arg(state == rc110_msgs::msg::Status::MOTOR_ON    ? "on"
                                                              : state == rc110_msgs::msg::Status::MOTOR_OFF ? "off"
                                                                                                            : "neutral")
                                               : QString("Failed to set motor state"),
                                       STATUS_MESSAGE_TIME);
            });
}

void Rc110Panel::onSetServoState(QAbstractButton* button)
{
    if (!servoStateService) return;

    int state = (button == ui->servoOnRadio)        ? rc110_msgs::msg::Status::MOTOR_ON
                : (button == ui->servoNeutralRadio) ? rc110_msgs::msg::Status::MOTOR_NEUTRAL
                                                    : rc110_msgs::msg::Status::MOTOR_OFF;

    auto request = std::make_shared<rc110_msgs::srv::SetInteger::Request>();
    request->data = state;
    servoStateService->async_send_request(
            request, [this, state](rclcpp::Client<rc110_msgs::srv::SetInteger>::SharedFuture response) {
                statusBar->showMessage(response.valid() && response.get()->success
                                               ? QString("Servomotor was set to %1")
                                                         .arg(state == rc110_msgs::msg::Status::MOTOR_ON    ? "on"
                                                              : state == rc110_msgs::msg::Status::MOTOR_OFF ? "off"
                                                                                                            : "neutral")
                                               : QString("Failed to set servo state"),
                                       STATUS_MESSAGE_TIME);
            });
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
    updateJoystickIcon();
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
    std_msgs::msg::String message;
    message.data = name;
    publish("rviz_rc", message);
}

void Rc110Panel::publishDrive()
{
    ackermann_msgs::msg::AckermannDriveStamped message;
    message.header.stamp = node->now();
    message.drive.speed = driveSpeed;
    message.drive.steering_angle = steeringAngle * DEG_TO_RAD;
    publish("drive_manual", message);
}

void Rc110Panel::publishOffsets()
{
    publish("offsets", offsets);
}

void Rc110Panel::onAdModeChanged(const std_msgs::msg::String& message)
{
    ui->adButton->setChecked(message.data == "drive_ad");
}

void Rc110Panel::onMotorSpeed(const std_msgs::msg::Float32& message)
{
    ui->driveSpeedEdit->setText(QString::number(message.data));
    showDriveGoalStatus();
}

void Rc110Panel::onSteeringAngle(const std_msgs::msg::Float32& message)
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

void Rc110Panel::onError(const rc110_msgs::msg::BaseboardError& message)
{
    int error = message.data;
    if (error == rc110_msgs::msg::BaseboardError::NONE) {
        ui->errorLabel->setPixmap(QPixmap(":/ok.png"));
        ui->errorLabel->setToolTip("Baseboard is ok");
    } else {
        ui->errorLabel->setPixmap(QPixmap(":/error.png"));

        QString tooltip = "Please, restart baseboard: %1";
        if (error == rc110_msgs::msg::BaseboardError::BOARD_HEAT) {
            ui->errorLabel->setToolTip(tooltip.arg("board has too high temperature"));
        } else if (error == rc110_msgs::msg::BaseboardError::MOTOR_HEAT) {
            ui->errorLabel->setToolTip(tooltip.arg("motor has too high temperature"));
        } else if (error == rc110_msgs::msg::BaseboardError::MOTOR_FAILURE) {
            ui->errorLabel->setToolTip(tooltip.arg("encoder feedback and polarity of the motor"));
        } else if (error == rc110_msgs::msg::BaseboardError::LOW_VOLTAGE) {
            ui->errorLabel->setToolTip(tooltip.arg("voltage dropped less than 6V for around 1s"));
        }
    }
}

void Rc110Panel::onRobotStatus(const rc110_msgs::msg::Status& message)
{
    ui->boardButton->setChecked(message.board_enabled);

    switch (message.motor_state) {
        case rc110_msgs::msg::Status::MOTOR_NEUTRAL:
            ui->motorNeutralRadio->setChecked(true);
            break;
        case rc110_msgs::msg::Status::MOTOR_ON:
            ui->motorOnRadio->setChecked(true);
            break;
        default:
            ui->motorOffRadio->setChecked(true);
    }

    switch (message.servo_state) {
        case rc110_msgs::msg::Status::MOTOR_NEUTRAL:
            ui->servoNeutralRadio->setChecked(true);
            break;
        case rc110_msgs::msg::Status::MOTOR_ON:
            ui->servoOnRadio->setChecked(true);
            break;
        default:
            ui->servoOffRadio->setChecked(true);
    }
}

void Rc110Panel::onDriveStatus(const ackermann_msgs::msg::AckermannDriveStamped& driveStatus)
{
    getTreeItem(DRIVE, "speed")->setText(1, printSensor(driveStatus.drive.speed, "m/s"));
    getTreeItem(DRIVE, "steering angle")->setText(1, printSensor(driveStatus.drive.steering_angle * RAD_TO_DEG, "°"));
    getTreeItem(DRIVE, "steering speed")
            ->setText(1, printSensor(driveStatus.drive.steering_angle_velocity * RAD_TO_DEG, "°"));
}

void Rc110Panel::onOffsets(const rc110_msgs::msg::Offsets& message)
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

void Rc110Panel::onOdometry(const nav_msgs::msg::Odometry& odometry)
{
    getTreeItem(DRIVE, "angular velocity")
            ->setText(1, printSensor(float(odometry.twist.twist.angular.z * RAD_TO_DEG), "°/s"));
}

void Rc110Panel::onServoBattery(const sensor_msgs::msg::BatteryState& batteryState)
{
    getTreeItem(BATTERY, "servo voltage")->setText(1, printSensor(batteryState.voltage, "V"));
    getTreeItem(BATTERY, "servo current")->setText(1, printSensor(batteryState.current * 1000, "mA", 0));
}

void Rc110Panel::onMotorBattery(const sensor_msgs::msg::BatteryState& batteryState)
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

void Rc110Panel::onBaseboardTemperature(const sensor_msgs::msg::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "baseboard")->setText(1, printSensor(temperature.temperature, "°C", 1));
}

void Rc110Panel::onServoTemperature(const sensor_msgs::msg::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "servo")->setText(1, printSensor(temperature.temperature, "°C", 1));
}

void Rc110Panel::onImu(const sensor_msgs::msg::Imu& imu)
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

void Rc110Panel::onMotorRate(const rc110_msgs::msg::MotorRate& motorRate)
{
    getTreeItem(OTHER, "motor rate")->setText(1, printSensor(motorRate.motor_rate, "cycles/s"));
    getTreeItem(OTHER, "estimated speed")->setText(1, printSensor(motorRate.estimated_speed, "m/s"));
}

void Rc110Panel::onWheelSpeeds(const rc110_msgs::msg::WheelSpeeds& wheelSpeeds)
{
    getTreeItem(OTHER, "wheel speed FL")->setText(1, printSensor(wheelSpeeds.speed_fl, "m/s"));
    getTreeItem(OTHER, "wheel speed FR")->setText(1, printSensor(wheelSpeeds.speed_fr, "m/s"));
    getTreeItem(OTHER, "wheel speed RL")->setText(1, printSensor(wheelSpeeds.speed_rl, "m/s"));
    getTreeItem(OTHER, "wheel speed RR")->setText(1, printSensor(wheelSpeeds.speed_rr, "m/s"));
}
}  // namespace zmp

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(zmp::Rc110Panel, rviz_common::Panel)