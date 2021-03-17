/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#include "rc110_object_detection.hpp"

#include <vision_msgs/Detection2DArray.h>

namespace zmp
{
Rc110ObjectDetection::Rc110ObjectDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_overlayFlags(detectNet::OVERLAY_NONE),
        m_inferenceNet(nullptr),
        m_inputConverter(),
        m_overlayConverter(),
        m_inputImageSub(pnh.subscribe("input_image", 1, &Rc110ObjectDetection::onImage, this)),
        m_detectionPub(pnh.advertise<vision_msgs::Detection2DArray>("detections", 25)),
        m_overlayPub(pnh.advertise<sensor_msgs::Image>("overlay", 2)),
        m_classesInfoPub(
                pnh.advertise<rc110_msgs::StringArray>("classes_info", 1, [this](const ros::SingleSubscriberPublisher&) {
                    publishClassesInfo();
                }))
{
    pnh.param<float>("confidence_thresh", m_param.confidenceThresh, m_param.confidenceThresh);
    pnh.param<std::string>("model_path", m_param.modelPath, m_param.modelPath);
    pnh.param<std::string>("class_labels_path", m_param.classLabelsPath, m_param.classLabelsPath);
    pnh.param<std::string>("overlay_str", m_param.overlayStr, m_param.overlayStr);

    pnh.param<std::string>("input_blob", m_param.inputBlob, m_param.inputBlob);
    pnh.param<std::string>("output_blob", m_param.outputBlob, m_param.outputBlob);
    pnh.param<std::string>("output_count", m_param.outputCount, m_param.outputCount);

    pnh.param<int>("num_channels", m_param.numChannels, m_param.numChannels);
    pnh.param<int>("input_height", m_param.inputHeight, m_param.inputHeight);
    pnh.param<int>("input_width", m_param.inputWidth, m_param.inputWidth);

    m_overlayFlags = detectNet::OverlayFlagsFromStr(m_param.overlayStr.c_str());

    m_inferenceNet.reset(detectNet::Create(m_param.modelPath.c_str(),
                                           m_param.classLabelsPath.c_str(),
                                           m_param.confidenceThresh,
                                           m_param.inputBlob.c_str(),
                                           Dims3(m_param.numChannels, m_param.inputHeight, m_param.inputWidth),
                                           m_param.outputBlob.c_str(),
                                           m_param.outputCount.c_str()));

    if (!m_inferenceNet) {
        ROS_ERROR("failed to load detectNet model");
        ros::shutdown();
    }

    m_classesMsg.data.reserve(m_inferenceNet->GetNumClasses());
    for (std::uint32_t i = 0; i < m_inferenceNet->GetNumClasses(); ++i) {
        m_classesMsg.data.emplace_back(m_inferenceNet->GetClassDesc(i));
    }
}

void Rc110ObjectDetection::onImage(const sensor_msgs::ImageConstPtr& message)
{
    if (!m_inputConverter.toDevice(message)) {
        ROS_INFO("failed to convert %ux%u %s image", message->width, message->height, message->encoding.c_str());
        return;
    }

    detectNet::Detection* detections = nullptr;
    const int numDetections = m_inferenceNet->Detect(m_inputConverter.deviceOutput(),
                                                     m_inputConverter.imageWidth(),
                                                     m_inputConverter.imageHeight(),
                                                     &detections,
                                                     detectNet::OVERLAY_NONE);
    if (numDetections < 0) {
        ROS_ERROR("failed to run object detection");
        return;
    }

    vision_msgs::Detection2DArray detectionsMsg;
    detectionsMsg.detections.reserve(numDetections);

    for (int i = 0; i < numDetections; ++i) {
        detectNet::Detection* curDet = detections + i;
        vision_msgs::Detection2D curDetectionMsg;
        curDetectionMsg.bbox.size_x = curDet->Width();
        curDetectionMsg.bbox.size_y = curDet->Height();
        float cx, cy;
        curDet->Center(&cx, &cy);
        curDetectionMsg.bbox.center.x = cx;
        curDetectionMsg.bbox.center.y = cy;

        vision_msgs::ObjectHypothesisWithPose hyp;
        hyp.id = curDet->ClassID;
        hyp.score = curDet->Confidence;
        curDetectionMsg.results.emplace_back(hyp);
        detectionsMsg.detections.emplace_back(curDetectionMsg);
    }

    detectionsMsg.header.stamp = message->header.stamp;
    m_detectionPub.publish(detectionsMsg);

    if (m_overlayPub.getNumSubscribers() > 0) {
        this->publishOverlay(detections, numDetections, message->header.stamp);
    }
}

void Rc110ObjectDetection::publishOverlay(detectNet::Detection* detections, int numDetections, const ros::Time& timeStamp)
{
    if (!m_overlayConverter.allocateMemory(m_inputConverter.imageWidth(),
                                           m_inputConverter.imageHeight(),
                                           Rc110ImageConverter::ROS_OUTPUT_IMAGE_FORMAT) ||
        !m_inferenceNet->Overlay(m_inputConverter.deviceOutput(),
                                 m_overlayConverter.deviceOutput(),
                                 m_inputConverter.imageWidth(),
                                 m_inputConverter.imageHeight(),
                                 Rc110ImageConverter::INTERNAL_IMAGE_FORMAT,
                                 detections,
                                 numDetections,
                                 m_overlayFlags)) {
        return;
    }

    sensor_msgs::ImagePtr overlayMsg = m_overlayConverter.toImageMessage(Rc110ImageConverter::ROS_OUTPUT_IMAGE_FORMAT);

    if (!overlayMsg) {
        return;
    }

    overlayMsg->header.stamp = timeStamp;
    m_overlayPub.publish(overlayMsg);
}

void Rc110ObjectDetection::publishClassesInfo()
{
    m_classesInfoPub.publish(m_classesMsg);
}
}  // namespace zmp
