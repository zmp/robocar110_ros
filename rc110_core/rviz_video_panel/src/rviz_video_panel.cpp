#include <QBoxLayout>
#include <QtMultimedia/QMediaPlayer>
#include <QtMultimediaWidgets/QVideoWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

namespace zmp
{
constexpr char DEFAULT_PIPELINE[] = "rtspsrc latency=0 location=%1 "
                                    "! rtph265depay ! h265parse ! queue "
                                    "! nvv4l2decoder ! nvvidconv ! video/x-raw,format=YUY2 "
                                    "! xvimagesink name=qtvideosink sync=false";

void myMessageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    if (type != QtWarningMsg) {
        fprintf(stderr, "%s\n", msg.toLocal8Bit().constData());
    }
}

/**
 * RViz panel to play video streams.
 */
class RVizVideoPanel : public rviz_common::Panel
{
    rclcpp::Node::SharedPtr node;
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
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("QMediaPlayer::Error"), player->errorString().toStdString());
        });
    }

    ~RVizVideoPanel() override { node->undeclare_parameter("video_url"); }

protected:
    void onInitialize() override
    {
        // display context available starting from this function
        node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;  // needed for undeclare_parameter
        QString url = QString::fromStdString(node->declare_parameter("video_url", std::string(), descriptor));
        QString pipeline = QString::fromStdString(node->declare_parameter("pipeline", DEFAULT_PIPELINE, descriptor));
        if (url.isEmpty()) {
            url = getName();
        }

// check if it is jetson basing on architecture, and enable custom pipeline for Qt >= 5.12.2
#if !defined(__ARM_ARCH) || QT_VERSION < QT_VERSION_CHECK(5, 12, 2)
        QString pipelineString = url;
#else
        QString pipelineString = "gst-pipeline: " + pipeline.arg(url);
#endif
        RCLCPP_INFO_STREAM(node->get_logger(), "Pipeline: " << pipelineString.toStdString());
        player->setMedia(QUrl(pipelineString));
        player->play();
    }
};
}  // namespace zmp

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(zmp::RVizVideoPanel, rviz_common::Panel)