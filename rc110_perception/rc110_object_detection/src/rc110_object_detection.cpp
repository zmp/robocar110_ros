/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#include "rc110_object_detection.hpp"

#include <sstream>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace zmp
{
Rc110ObjectDetection::Rc110ObjectDetection() :
        Node("rc110_object_detection"),
        m_overlayFlags(detectNet::OVERLAY_NONE),
        m_inferenceNet(nullptr),
        m_inputConverter(*this),
        m_inputImageSub(create_subscription<sensor_msgs::msg::Image>("/front_camera/image_raw", 1,
                                                                     std::bind(&Rc110ObjectDetection::onImage, this, std::placeholders::_1))),
        m_detectionPub(create_publisher<vision_msgs::msg::Detection2DArray>("detections", 25)),
        m_overlayPub(create_publisher<sensor_msgs::msg::Image>("overlay", 2)),
        m_classesInfoPub(create_publisher<rc110_msgs::msg::StringArray>("classes_info", 1))
{
    m_param.confidenceThresh = declare_parameter("confidence_thresh", m_param.confidenceThresh);
    m_param.modelPath = declare_parameter("model_path", m_param.modelPath);
    m_param.classLabelsPath = declare_parameter("class_labels_path", m_param.classLabelsPath);
    m_param.overlayStr = declare_parameter("overlay_str", m_param.overlayStr);

    m_param.inputBlob = declare_parameter("input_blob", m_param.inputBlob);
    m_param.outputBlob = declare_parameter("output_blob", m_param.outputBlob);
    m_param.outputCount = declare_parameter("output_count", m_param.outputCount);

    m_param.meanPixel = declare_parameter("mean_pixel", m_param.meanPixel);
    m_param.stdPixel = declare_parameter("std_pixel", m_param.stdPixel);
    m_param.useDarknetYolo = declare_parameter("use_darknet_yolo", m_param.useDarknetYolo);

    m_param.numChannels = declare_parameter("num_channels", m_param.numChannels);
    m_param.inputHeight = declare_parameter("input_height", m_param.inputHeight);
    m_param.inputWidth = declare_parameter("input_width", m_param.inputWidth);

    m_overlayFlags = detectNet::OverlayFlagsFromStr(m_param.overlayStr.c_str());

    m_inferenceNet =
            std::move(Rc110CustomDetectNet::Create(m_param.modelPath.c_str(),
                                                   m_param.classLabelsPath.c_str(),
                                                   m_param.confidenceThresh,
                                                   m_param.inputBlob.c_str(),
                                                   Dims3(m_param.numChannels, m_param.inputHeight, m_param.inputWidth),
                                                   m_param.outputBlob.c_str(),
                                                   m_param.outputCount.c_str(),
                                                   m_param.meanPixel,
                                                   m_param.stdPixel,
                                                   m_param.useDarknetYolo));

    if (!m_inferenceNet)
    {
        RCLCPP_ERROR(get_logger(), "failed to load detectNet model");
        rclcpp::shutdown();
    }

    m_classesMsg.data.reserve(m_inferenceNet->GetNumClasses());
    for (std::uint32_t i = 0; i < m_inferenceNet->GetNumClasses(); i++)
    {
        m_classesMsg.data.emplace_back(m_inferenceNet->GetClassDesc(i));
    }

    m_classColors.reserve(m_inferenceNet->GetNumClasses());
    for (std::uint32_t i = 0; i < m_inferenceNet->GetNumClasses(); i++)
    {
        m_classColors.emplace_back(cv::Scalar((m_inferenceNet->GenerateColor(i).x,
                                               m_inferenceNet->GenerateColor(i).y,
                                               m_inferenceNet->GenerateColor(i).z,
                                               m_inferenceNet->GenerateColor(i).w)));
    }
    publishClassesInfo();
}

void Rc110ObjectDetection::onImage(const sensor_msgs::msg::Image& message)
{
    if (!m_inputConverter.toDevice(message)) {
        RCLCPP_INFO(get_logger(),
                    "failed to convert %ux%u %s image",
                    message.width,
                    message.height,
                    message.encoding.c_str());
        return;
    }

    detectNet::Detection* detections = nullptr;
    const int numDetections = m_inferenceNet->CustomDetect(m_inputConverter.deviceOutput(),
                                                           m_inputConverter.imageWidth(),
                                                           m_inputConverter.imageHeight(),
                                                           &detections,
                                                           detectNet::OVERLAY_NONE);
    if (numDetections < 0)
    {
        RCLCPP_ERROR(get_logger(), "failed to run object detection");
        return;
    }

    vision_msgs::msg::Detection2DArray detectionsMsg;
    detectionsMsg.detections.reserve(numDetections);

    for (int i = 0; i < numDetections; ++i)
    {
        detectNet::Detection* curDet = detections + i;
        vision_msgs::msg::Detection2D curDetectionMsg;
        curDetectionMsg.bbox.size_x = curDet->Width();
        curDetectionMsg.bbox.size_y = curDet->Height();
        float cx, cy;
        curDet->Center(&cx, &cy);
        curDetectionMsg.bbox.center.x = cx;
        curDetectionMsg.bbox.center.y = cy;

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = std::to_string(curDet->ClassID);
        hyp.hypothesis.score = curDet->Confidence;
        curDetectionMsg.results.emplace_back(hyp);
        detectionsMsg.detections.emplace_back(curDetectionMsg);
    }

    detectionsMsg.header.stamp = message.header.stamp;
    m_detectionPub->publish(detectionsMsg);

    if (m_overlayPub->get_subscription_count() > 0)
    {
        this->publishOverlay(detectionsMsg, m_classesMsg, m_classColors, message.header.stamp);
    }
}

void Rc110ObjectDetection::publishOverlay(const vision_msgs::msg::Detection2DArray& detectionsMsg,
                                          const rc110_msgs::msg::StringArray& classesMsg,
                                          const std::vector<cv::Scalar>& colors,
                                          const rclcpp::Time& timeStamp)
{
    cv::Mat inputImage(m_inputConverter.imageHeight(),
                       m_inputConverter.imageWidth(),
                       CV_8UC3,
                       m_inputConverter.hostOutput());

    cv::Mat processedImage;
    cv::cvtColor(inputImage, processedImage, cv::COLOR_RGB2BGR);
    cv::Mat overlayImage = this->drawBoundingBox(processedImage, detectionsMsg, classesMsg, colors);

    const std::size_t msgSize = imageFormatSize(Rc110ImageConverter::RCLCPP_OUTPUT_IMAGE_FORMAT,
                                                m_inputConverter.imageWidth(),
                                                m_inputConverter.imageHeight());
    sensor_msgs::msg::Image overlayMsg;
    overlayMsg.data.resize(msgSize);
    memcpy(overlayMsg.data.data(), overlayImage.data, msgSize);
    overlayMsg.width = m_inputConverter.imageWidth();
    overlayMsg.height = m_inputConverter.imageHeight();
    overlayMsg.step =
            (m_inputConverter.imageWidth() * imageFormatDepth(Rc110ImageConverter::RCLCPP_OUTPUT_IMAGE_FORMAT)) / 8;
    overlayMsg.encoding = Rc110ImageConverter::toImageEncoding(Rc110ImageConverter::RCLCPP_OUTPUT_IMAGE_FORMAT);
    overlayMsg.is_bigendian = false;

    overlayMsg.header.stamp = timeStamp;
    m_overlayPub->publish(overlayMsg);
}

cv::Mat Rc110ObjectDetection::drawBoundingBox(const cv::Mat& image,
                                              const vision_msgs::msg::Detection2DArray& detectionsMsg,
                                              const rc110_msgs::msg::StringArray& classesMsg,
                                              const std::vector<cv::Scalar>& colors) const
{
    if (classesMsg.data.size() != colors.size()) {
        throw std::runtime_error("sizes of classes and colors mismatch");
    }

    cv::Mat result = image.clone();

    for (const auto& curDetection : detectionsMsg.detections)
    {
        const std::int64_t classIdx = std::atoll(curDetection.results[0].hypothesis.class_id.c_str());
        const float confidenceScore = curDetection.results[0].hypothesis.score;
        const std::string& curClass = classesMsg.data[classIdx];
        const cv::Scalar& curColor = colors[classIdx];

        std::stringstream ss;
        ss << curClass << " " << std::setprecision(2) << std::fixed << confidenceScore;
        const std::string labelToDraw(std::move(ss.str()));

        float width = curDetection.bbox.size_x, height = curDetection.bbox.size_y;
        float xCenter = curDetection.bbox.center.x, yCenter = curDetection.bbox.center.y;
        int xMin = xCenter - width / 2, yMin = yCenter - height / 2;
        int xMax = xCenter + width / 2, yMax = yCenter + height / 2;

        cv::rectangle(result, cv::Point(xMin, yMin), cv::Point(xMax, yMax), curColor, 1.5);
        int baseLine = 0;
        cv::Size labelSize = cv::getTextSize(labelToDraw, cv::FONT_HERSHEY_SIMPLEX, 0.2, 1, &baseLine);
        int textYMin = std::max(yMin, labelSize.height);
        cv::rectangle(result,
                      cv::Point(xMin, textYMin - std::round(1.5 * labelSize.height)),
                      cv::Point(xMin + std::round(1.5 * labelSize.width), textYMin + baseLine),
                      cv::Scalar(255, 255, 255),
                      -1);
        cv::putText(result,
                    labelToDraw,
                    cv::Point(xMin, textYMin),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.3 /*fontScale*/,
                    cv::Scalar(0, 0, 0),
                    1 /*thickness*/);
    }

    return result;
}

void Rc110ObjectDetection::publishClassesInfo()
{
    m_classesInfoPub->publish(m_classesMsg);
}
}  // namespace zmp
