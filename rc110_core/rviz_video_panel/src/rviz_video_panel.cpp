#include <ros/ros.h>
#include <rviz/panel.h>

#include <QBoxLayout>
#include <QtMultimedia/QMediaPlayer>
#include <QtMultimediaWidgets/QVideoWidget>

namespace zmp
{
void myMessageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    if (type != QtWarningMsg) {
        fprintf(stderr, "%s\n", msg.toLocal8Bit().constData());
    }
}

/**
 * RViz panel to play video streams.
 */
class RVizVideoPanel : public rviz::Panel
{
    QMediaPlayer* player;

public:
    explicit RVizVideoPanel(QWidget* parent = nullptr) : Panel(parent)
    {
        qInstallMessageHandler(myMessageOutput);  // disable QMediaPlayer warnings

        auto videoWidget = new QVideoWidget(this);
        auto layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(videoWidget);

        player = new QMediaPlayer(this);
        player->setVideoOutput(videoWidget);

        connect(player, QOverload<QMediaPlayer::Error>::of(&QMediaPlayer::error), [this](QMediaPlayer::Error error) {
            ROS_ERROR_STREAM("QMediaPlayer::Error: " << player->errorString().toStdString());
        });
    }

protected:
    void onInitialize() override
    {
        QString url = QString::fromStdString(ros::param::param("~video_url", std::string()));
        if (url.isEmpty()) {
            url = getName();
        }
        player->setMedia(QUrl(url));
        player->play();
    }
};
}  // namespace zmp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zmp::RVizVideoPanel, rviz::Panel)