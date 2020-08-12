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
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#endif

namespace Ui
{
class PanelWidget;
}
class QTreeWidgetItem;

namespace zmp
{
class Rc110Panel : public rviz::Panel
{
    Q_OBJECT

    enum TREE_ITEM_GROUP { BATTERY, TEMPERATURE, GYROSCOPE };
    static constexpr const char* TREE_ITEM_GROUP_NAME[] = {"Battery", "Temperature", "Gyroscope"};

public:
    explicit Rc110Panel(QWidget* parent = nullptr);
    ~Rc110Panel() override;

private:
    QTreeWidgetItem* getTreeItem(TREE_ITEM_GROUP group, const char* name) const;
    void onBatteryState(const sensor_msgs::BatteryState& batteryState);
    void onMotorTemperature(const sensor_msgs::Temperature& temperature);

private:
    std::unique_ptr<Ui::PanelWidget> ui;
    ros::NodeHandle handle;
    QVector<ros::Subscriber> subscribers;
    QHash<TREE_ITEM_GROUP, QTreeWidgetItem*> treeItems;
};
}  // namespace zmp