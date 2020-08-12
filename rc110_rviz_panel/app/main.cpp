#include <QApplication>
#include <QTimer>

#include "../src/rc110_panel.hpp"

using namespace zmp;

const char* APP_NAME = "rc110_panel_app";

int main(int argc, char* argv[])
{
    ros::init(argc, argv, APP_NAME);

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    Rc110Panel panel;
    panel.setWindowTitle(APP_NAME);
    panel.show();

    QTimer timer;
    timer.setInterval(50);
    timer.start();
    QObject::connect(&timer, &QTimer::timeout, [] { ros::spinOnce(); });

    return QApplication::exec();
}
