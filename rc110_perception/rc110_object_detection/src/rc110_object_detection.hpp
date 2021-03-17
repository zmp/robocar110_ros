/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#pragma once

#include <jetson-inference/detectNet.h>
#include <rc110_msgs/StringArray.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

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
    void publishOverlay(detectNet::Detection* detections,
                        int numDetections,
                        const ros::Time& timeStamp = ros::Time::now());

    void publishClassesInfo();

private:
    Param m_param;
    std::uint32_t m_overlayFlags;
    rc110_msgs::StringArray m_classesMsg;

    std::unique_ptr<detectNet> m_inferenceNet;
    Rc110ImageConverter m_inputConverter;
    Rc110ImageConverter m_overlayConverter;

    ros::Subscriber m_inputImageSub;

    // publisher for detected bounding boxes
    ros::Publisher m_detectionPub;

    // publisher for images overlaid with detected bounding boxes
    ros::Publisher m_overlayPub;

    // publisher for info of classes
    ros::Publisher m_classesInfoPub;
};
}  // namespace zmp
