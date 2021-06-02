/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#pragma once

#include <memory>

#include <jetson-inference/detectNet.h>

namespace zmp
{
class Rc110CustomDetectNet : public detectNet
{
public:
    using Ptr = std::unique_ptr<Rc110CustomDetectNet>;

    static Ptr Create(const char* modelPath,
                      const char* classLabels,
                      const float threshold,
                      const char* input,
                      const Dims3& inputDims,
                      const char* output,
                      const char* numDetections,
                      const float meanPixel,
                      const float stdPixel,
                      const bool useDarknetYolo,
                      const std::uint32_t maxBatchSize = DEFAULT_MAX_BATCH_SIZE,
                      const precisionType precision = TYPE_FASTEST,
                      const deviceType device = DEVICE_GPU,
                      const bool allowGPUFallback = true);

    template <typename T>
    int CustomDetect(T* image,
                     const std::uint32_t width,
                     const std::uint32_t height,
                     Detection** detections,
                     const std::uint32_t overlay = OVERLAY_BOX)
    {
        return this->CustomDetect((void*)image, width, height, imageFormatFromType<T>(), detections, overlay);
    }

private:
    Rc110CustomDetectNet(const float meanPixel, const float stdPixel, const bool useDarknetYolo);

    /**
     *  @brief custom detection function to use darknet yolo model
     *
     *  if m_useDarknetYolo is false, this function will be a wrapper of jetson-inference's detection function
     *
     *  @param detections array of detections that store details of detected objects
     *  @return number of detected objects
     */
    int CustomDetect(void* input,
                     const std::uint32_t width,
                     const std::uint32_t height,
                     const imageFormat format,
                     Detection* detections,
                     const std::uint32_t overlay);

    /**
     *  @brief post process of the inference output
     *
     *  @return number of detected objects
     */
    int PostProcess(const std::uint32_t width, const std::uint32_t height, Detection* detections);

    /**
     *  @brief wrapper of the custom detect function that stores detection details in detectNet's preallocated buffer
     *
     */
    int CustomDetect(void* input,
                     const std::uint32_t width,
                     const std::uint32_t height,
                     const imageFormat format,
                     Detection** detections,
                     const std::uint32_t overlay);

private:
    // standard deviation value for input image's z-score normalization
    // note that this input image has been rescaled to [0,1]
    float m_stdPixel;

    // flag to specify if current weights are from darknet yolo model or not
    bool m_useDarknetYolo;
};
}  // namespace zmp
