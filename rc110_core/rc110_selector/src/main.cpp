/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */

#include <QApplication>
#include <QComboBox>
#include <QHostInfo>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QTextStream>
#include <QTimer>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <regex>

QStringList getRobotNames(rclcpp::Node* node)
{
    std::vector<std::string> nodeNames;
    nodeNames = node->get_node_names();

    static const std::regex expression("/(.*)/drive_control$");

    QStringList robotNames;
    for (const auto& nodeName : nodeNames) {
        std::smatch match;
        if (std::regex_match(nodeName, match, expression); !match.empty()) {
            robotNames << match[1].str().c_str();
        }
    }
    return robotNames;
}

void noOutput(QtMsgType, const QMessageLogContext&, const QString&) {}

bool tryUsingHostname(const std::string& prefix)
{
    auto hostname = QHostInfo::localHostName();
    if (0 == hostname.indexOf(prefix.c_str())) {
        QTextStream(stdout) << hostname.replace('-', '_');
        return true;
    }
    return false;
}

constexpr int tick = 500;
int timeout;
QStringList lastNames;

int main(int argc, char** argv)
{
    qInstallMessageHandler(noOutput);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rc110_selector");
    node->get_logger().set_level(rclcpp::Logger::Level::Fatal);

    timeout = (int)node->declare_parameter("timeout", 5000);
    auto prefix = node->declare_parameter("prefix", "rc-");

    if (tryUsingHostname(prefix)) {
        return EXIT_SUCCESS;
    }

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
    QObject::connect(&timer, &QTimer::timeout, [&combo, &label, &node] {
        rclcpp::spin_some(node);

        QStringList names = getRobotNames(node.get());

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
    return QApplication::exec();
}
