#include <QApplication>
#include <QTimer>
#include <QUrl>

#include "../src/rc110_panel.hpp"

using namespace zmp;

const char* APP_NAME = "RC110";

/**
 * A small application to test sensors widget without rviz.
 */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, APP_NAME);

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    Rc110Panel panel;
    if (auto uri = std::getenv("ROS_MASTER_URI")) {
        panel.setWindowTitle(QString("%1: %2").arg(APP_NAME).arg(QUrl(uri).host()));
    } else {
        panel.setWindowTitle(APP_NAME);
    }
    panel.show();

    QTimer timer;
    timer.setInterval(50);
    timer.start();
    QObject::connect(&timer, &QTimer::timeout, [] { ros::spinOnce(); });

    return QApplication::exec();
}
