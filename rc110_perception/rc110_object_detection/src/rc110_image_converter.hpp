/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#pragma once

#include <jetson-utils/cudaUtility.h>
#include <jetson-utils/imageFormat.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdlib>
#include <string>

namespace zmp
{
/**
 * Utility class that handles transferring of data between host and device.
 */
class Rc110ImageConverter
{
public:
    // cuda's built-in vector type
    using PixelType = uchar3;

    // image format used for internal CUDA processing
    static constexpr imageFormat INTERNAL_IMAGE_FORMAT = IMAGE_RGB8;

    // image format used for outputting ROS image messages
    static constexpr imageFormat ROS_OUTPUT_IMAGE_FORMAT = IMAGE_BGR8;

public:
    Rc110ImageConverter();

    ~Rc110ImageConverter();

    /**
     *  @brief convert image encoding to imageFormat enum type defined in jetson-utils
     *
     *  @param imageEncoding (ROS) sensor_msgs' image encoding
     *  @return imageFormat enum image format type
     */
    static imageFormat toImageFormat(const std::string& imageEncoding);

    /**
     *  @brief convert imageFormat enum type defined in jetson-utils to image encoding
     *
     *  @param iFormat enum image format type
     *  @return (ROS) sensor_msgs' image encoding
     */
    static std::string toImageEncoding(const imageFormat iFormat);

    std::size_t imageWidth() const { return m_imageWidth; }

    std::size_t imageHeight() const { return m_imageHeight; }

    PixelType* deviceOutput() const { return m_deviceOutput; }

    /**
     *  @brief transfer image message data to device and convert to internal format (IMAGE_RGB8)
     *
     *  @param inputImageMsg input image message
     *  @return true if successful; false otherwise
     */
    bool toDevice(const sensor_msgs::ImageConstPtr& inputImageMsg);

    /**
     *  @brief create image message from internal device data, mainly for overlay debug purpose
     *
     *  @param outputImageFormat output image message's format (enum type defined in jetson-utils)
     *  @return output image message
     */
    sensor_msgs::ImagePtr toImageMessage(const imageFormat outputImageFormat) const;

    /**
     *  @brief allocate pinned-memory resources needed for inference process, or storing overlaid image after inference
     *
     *  basically, raw image data, after being transferred to device, need to be further transformed
     *  so that it can be readily fed into the inference network.
     *
     *  @param imageWidth image width
     *  @param imageHeight image height
     *  @param imageFormat image format (enum type defined in jetson-utils)
     *  @return true if allocated successfully; false otherwise
     */
    bool allocateMemory(const std::size_t imageWidth, const std::size_t imageHeight, const imageFormat iFormat);

private:
    void freeMemory();

private:
    std::size_t m_imageWidth;
    std::size_t m_imageHeight;
    imageFormat m_inputImageFormat;

    // m_hostInput (m_deviceInput) corresponds to the resource needed for raw input image data
    // the other purpose of this resource is to store the overlaid image after inference to create an overlaid debug
    // image message
    void* m_hostInput;
    void* m_deviceInput;

    // m_hostOutput (m_deviceOutput) corresponds to the resource needed for further transformed data
    // that can be readily fed into inference network
    PixelType* m_hostOutput;
    PixelType* m_deviceOutput;
};

}  // namespace zmp
