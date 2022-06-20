/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QTextStream>
#include <QTimer>
#include <QWidget>
#include <regex>

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

void noOutput(QtMsgType, const QMessageLogContext&, const QString&) {}

constexpr int tick = 500;
int timeout;
QStringList lastNames;

int main(int argc, char** argv)
{
    // suppress all output except application result
    if( ros::console::set_logger_level("ros", ros::console::levels::Fatal) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    qInstallMessageHandler(noOutput);

    ros::init(argc, argv, "rc110_selector");
    timeout = ros::param::param("~timeout", 5000);

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    QWidget window;
    window.setWindowFlags(window.windowFlags() | Qt::FramelessWindowHint);

    QLabel image;
    image.setPixmap(QPixmap(":/image.png"));
    QLabel label("Waiting...");
    QComboBox combo;
    combo.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QPushButton button("Ok");

    QVBoxLayout outerLayout;
    QHBoxLayout layout;
    outerLayout.addWidget(&image);
    outerLayout.addLayout(&layout);
    layout.addWidget(&label);
    layout.addWidget(&combo);
    layout.addWidget(&button);

    window.setLayout(&outerLayout);
    window.show();

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&combo, &label] {
        ros::spinOnce();

        QStringList names = getRobotNames();

        // For 2 or more robots, wait for user selection.
        // Otherwise do countdown resulting in "a robot selected" or "no robots".
        if (names.size() >= 2) {
            label.setText("Selection:");
        } else if (timeout > 0) {
            timeout -= tick;
            label.setText(QString("Waiting...(%1)").arg(timeout / 1000));
        } else {
            if (!names.empty()) {
                QTextStream(stdout) << names.front();
            }
            QApplication::quit();
            return;
        }
        // Update robot names each tick.
        if (lastNames != names) {
            lastNames = names;
            combo.clear();
            combo.addItems(names);
        }
    });
    timer.start(tick);

    QObject::connect(&button, &QPushButton::clicked, [&combo] {
        QTextStream(stdout) << combo.currentText();
        QApplication::quit();
    });
    return app.exec();
}
