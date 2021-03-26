/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#include "rc110_image_converter.hpp"

#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <ros/ros.h>

namespace zmp
{
Rc110ImageConverter::Rc110ImageConverter() :
        m_imageWidth(0),
        m_imageHeight(0),
        m_inputImageFormat(ROS_OUTPUT_IMAGE_FORMAT),
        m_hostInput(nullptr),
        m_deviceInput(nullptr),
        m_hostOutput(nullptr),
        m_deviceOutput(nullptr)
{
}

Rc110ImageConverter::~Rc110ImageConverter()
{
    this->freeMemory();
}

imageFormat Rc110ImageConverter::toImageFormat(const std::string& imageEncoding)
{
    if (imageEncoding == sensor_msgs::image_encodings::BGR8) {
        return IMAGE_BGR8;
    } else if (imageEncoding == sensor_msgs::image_encodings::BGRA8) {
        return IMAGE_BGRA8;
    } else if (imageEncoding == sensor_msgs::image_encodings::RGB8) {
        return IMAGE_RGB8;
    } else if (imageEncoding == sensor_msgs::image_encodings::RGBA8) {
        return IMAGE_RGBA8;
    } else if (imageEncoding == sensor_msgs::image_encodings::MONO8) {
        return IMAGE_GRAY8;
    } else if (imageEncoding == sensor_msgs::image_encodings::YUV422) {
        return IMAGE_UYVY;
    } else if (imageEncoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
        return IMAGE_BAYER_RGGB;
    } else if (imageEncoding == sensor_msgs::image_encodings::BAYER_BGGR8) {
        return IMAGE_BAYER_BGGR;
    } else if (imageEncoding == sensor_msgs::image_encodings::BAYER_GBRG8) {
        return IMAGE_BAYER_GBRG;
    } else if (imageEncoding == sensor_msgs::image_encodings::BAYER_GRBG8) {
        return IMAGE_BAYER_GRBG;
    }

    return IMAGE_UNKNOWN;
}

std::string Rc110ImageConverter::toImageEncoding(const imageFormat iFormat)
{
    switch (iFormat) {
        case IMAGE_BGR8:
            return sensor_msgs::image_encodings::BGR8;
        case IMAGE_BGRA8:
            return sensor_msgs::image_encodings::BGRA8;
        case IMAGE_RGB8:
            return sensor_msgs::image_encodings::RGB8;
        case IMAGE_RGBA8:
            return sensor_msgs::image_encodings::RGBA8;
        case IMAGE_GRAY8:
            return sensor_msgs::image_encodings::MONO8;
        case IMAGE_UYVY:
            return sensor_msgs::image_encodings::YUV422;
        case IMAGE_BAYER_RGGB:
            return sensor_msgs::image_encodings::BAYER_RGGB8;
        case IMAGE_BAYER_BGGR:
            return sensor_msgs::image_encodings::BAYER_BGGR8;
        case IMAGE_BAYER_GBRG:
            return sensor_msgs::image_encodings::BAYER_GBRG8;
        case IMAGE_BAYER_GRBG:
            return sensor_msgs::image_encodings::BAYER_GRBG8;
        default:
            return "invalid";
    }
}

bool Rc110ImageConverter::toDevice(const sensor_msgs::ImageConstPtr& inputImageMsg)
{
    ROS_DEBUG("converting %ux%u %s image", inputImageMsg->width, inputImageMsg->height, inputImageMsg->encoding.c_str());
    const imageFormat inputImageFormat = Rc110ImageConverter::toImageFormat(inputImageMsg->encoding);
    if (inputImageFormat == IMAGE_UNKNOWN) {
        ROS_ERROR("image encoding %s is not a compatible format", inputImageMsg->encoding.c_str());
        return false;
    }

    if (!this->allocateMemory(inputImageMsg->width, inputImageMsg->height, inputImageFormat)) {
        return false;
    };
    memcpy(m_hostInput,
           inputImageMsg->data.data(),
           imageFormatSize(inputImageFormat, inputImageMsg->width, inputImageMsg->height));

    // convert inputImageMsg's data to device output pointer of INTERNAL_IMAGE_FORMAT
    if (CUDA_FAILED(cudaConvertColor(m_deviceInput,
                                     inputImageFormat,
                                     m_deviceOutput,
                                     INTERNAL_IMAGE_FORMAT,
                                     inputImageMsg->width,
                                     inputImageMsg->height))) {
        ROS_ERROR("failed to convert %ldx%ld image (from %s to %s) with CUDA",
                  m_imageWidth,
                  m_imageHeight,
                  imageFormatToStr(inputImageFormat),
                  imageFormatToStr(INTERNAL_IMAGE_FORMAT));

        return false;
    }

    return true;
}

sensor_msgs::ImagePtr Rc110ImageConverter::toImageMessage(const imageFormat outputImageFormat) const
{
    if (!m_hostInput || !m_deviceOutput || m_imageWidth == 0 || m_imageHeight == 0) {
        return nullptr;
    }

    if (CUDA_FAILED(cudaConvertColor(
                m_deviceOutput, INTERNAL_IMAGE_FORMAT, m_deviceInput, outputImageFormat, m_imageWidth, m_imageHeight))) {
        ROS_ERROR("failed to convert %ldx%ld image (from %s to %s) with CUDA",
                  m_imageWidth,
                  m_imageHeight,
                  imageFormatToStr(INTERNAL_IMAGE_FORMAT),
                  imageFormatToStr(outputImageFormat));

        return nullptr;
    }

    const std::size_t msgSize = imageFormatSize(outputImageFormat, m_imageWidth, m_imageHeight);
    sensor_msgs::ImagePtr outputMsg(new sensor_msgs::Image());
    outputMsg->data.resize(msgSize);

    memcpy(outputMsg->data.data(), m_hostInput, msgSize);
    outputMsg->width = m_imageWidth;
    outputMsg->height = m_imageHeight;
    outputMsg->step = (m_imageWidth * imageFormatDepth(outputImageFormat)) / 8;
    outputMsg->encoding = Rc110ImageConverter::toImageEncoding(outputImageFormat);
    outputMsg->is_bigendian = false;

    return outputMsg;
}

bool Rc110ImageConverter::allocateMemory(const std::size_t imageWidth,
                                         const std::size_t imageHeight,
                                         const imageFormat iFormat)
{
    if (m_imageWidth != imageWidth || m_imageHeight != imageHeight || m_inputImageFormat != iFormat) {
        const std::size_t inputSize = imageFormatSize(iFormat, imageWidth, imageHeight);
        const std::size_t outputSize = imageFormatSize(INTERNAL_IMAGE_FORMAT, imageWidth, imageHeight);

        this->freeMemory();

        // cudaAllocMapped allocates (page-locked) pinned-memory on host that is accessible by the device.
        // using pinned-memory increases the bandwith for data transfer between host and device.
        // m_hostInput, m_deviceInput both resolve to the same physical memory.
        // m_hostOutput, m_deviceOutput both resolve to the same physical memory.
        if (!cudaAllocMapped((void**)&m_hostInput, (void**)&m_deviceInput, inputSize) ||
            !cudaAllocMapped((void**)&m_hostOutput, (void**)&m_deviceOutput, outputSize)) {
            ROS_ERROR("failed to allocate memory for %ldx%ld image conversion", imageWidth, imageHeight);
            return false;
        }

        ROS_INFO("allocated CUDA memory for %ldx%ld image conversion", imageWidth, imageHeight);

        m_imageWidth = imageWidth;
        m_imageHeight = imageHeight;
        m_inputImageFormat = iFormat;
    }

    return true;
}

void Rc110ImageConverter::freeMemory()
{
    if (m_hostInput) {
        CUDA(cudaFreeHost(m_hostInput));
        m_hostInput = nullptr;
        m_deviceInput = nullptr;
    }

    if (m_hostOutput) {
        CUDA(cudaFreeHost(m_hostOutput));
        m_hostOutput = nullptr;
        m_deviceOutput = nullptr;
    }
}
}  // namespace zmp
