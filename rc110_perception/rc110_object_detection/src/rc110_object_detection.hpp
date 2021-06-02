/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#pragma once

#include <rc110_msgs/StringArray.h>
#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

#include "rc110_custom_detect_net.hpp"
#include "rc110_image_converter.hpp"

namespace zmp
{
/**
 * Node that uses jetson-inference library for 2d object detection.
 */
class Rc110ObjectDetection
{
public:
    struct Param {
        float confidenceThresh = 0.6;
        std::string modelPath = "";
        std::string classLabelsPath = "";
        std::string overlayStr = "box,labels,conf";

        std::string inputBlob = "Input";
        std::string outputBlob = "NMS";
        std::string outputCount = "NMS_1";

        float meanPixel = 0.0;
        float stdPixel = 1.0;
        bool useDarknetYolo = false;

        int numChannels = 3;
        int inputHeight = 300;
        int inputWidth = 300;
    };

public:
    Rc110ObjectDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    /**
     *  @brief callback function to call detection network inference on new input images
     */
    void onImage(const sensor_msgs::ImageConstPtr& message);

    /**
     *  @brief create and publish overlaid result image message
     */
    void publishOverlay(const vision_msgs::Detection2DArray& detectionsMsg,
                        const rc110_msgs::StringArray& classesMsg,
                        const std::vector<cv::Scalar>& colors,
                        const ros::Time& timeStamp = ros::Time::now());

    cv::Mat drawBoundingBox(const cv::Mat& image,
                            const vision_msgs::Detection2DArray& detectionsMsg,
                            const rc110_msgs::StringArray& classesMsg,
                            const std::vector<cv::Scalar>& colors) const;

    void publishClassesInfo();

private:
    Param m_param;
    std::uint32_t m_overlayFlags;
    rc110_msgs::StringArray m_classesMsg;
    std::vector<cv::Scalar> m_classColors;

    Rc110CustomDetectNet::Ptr m_inferenceNet;
    Rc110ImageConverter m_inputConverter;

    ros::Subscriber m_inputImageSub;

    // publisher for detected bounding boxes
    ros::Publisher m_detectionPub;

    // publisher for images overlaid with detected bounding boxes
    ros::Publisher m_overlayPub;

    // publisher for info of classes
    ros::Publisher m_classesInfoPub;
};
}  // namespace zmp
