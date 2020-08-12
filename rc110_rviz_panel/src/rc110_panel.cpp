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

    subscribers.push_back(handle.subscribe("battery", 1, &Rc110Panel::onBatteryState, this));
    subscribers.push_back(handle.subscribe("motor_temperature", 1, &Rc110Panel::onMotorTemperature, this));
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

void Rc110Panel::onBatteryState(const sensor_msgs::BatteryState& batteryState)
{
    getTreeItem(BATTERY, "voltage")->setText(1, QString("%1 V").arg(batteryState.voltage));
}

void Rc110Panel::onMotorTemperature(const sensor_msgs::Temperature& temperature)
{
    getTreeItem(TEMPERATURE, "motor")->setText(1, QString::fromUtf8("%1 °C").arg(temperature.temperature));
}

}  // namespace zmp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zmp::Rc110Panel, rviz::Panel)