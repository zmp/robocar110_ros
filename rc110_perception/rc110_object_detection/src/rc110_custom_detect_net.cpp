/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran
 */

#include "rc110_custom_detect_net.hpp"

#include <jetson-inference/tensorConvert.h>

#include <algorithm>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace
{
// indices of confidence and bounding box values in the inference output vector
constexpr int OUTPUT_CONF_DARKNET_YOLO = 0;
constexpr int OUTPUT_BBOX_DARKNET_YOLO = 1;

#define CHECK_NULL_STR(x) (x) ? (x) : "NULL"
}  // namespace

namespace zmp
{
Rc110CustomDetectNet::Rc110CustomDetectNet(const float meanPixel, const float stdPixel, const bool useDarknetYolo) :
        detectNet(meanPixel),
        m_stdPixel(stdPixel),
        m_useDarknetYolo(useDarknetYolo)
{ }

Rc110CustomDetectNet::Ptr Rc110CustomDetectNet::Create(const char* modelPath,
                                                       const char* classLabels,
                                                       const float threshold,
                                                       const char* input,
                                                       const Dims3& inputDims,
                                                       const char* output,
                                                       const char* numDetections,
                                                       const float meanPixel,
                                                       const float stdPixel,
                                                       const bool useDarknetYolo,
                                                       const std::uint32_t maxBatchSize,
                                                       const precisionType precision,
                                                       const deviceType device,
                                                       const bool allowGPUFallback)
{
    Rc110CustomDetectNet::Ptr net(new Rc110CustomDetectNet(meanPixel, stdPixel, useDarknetYolo));

    // using the same logging system of jetson-inference here
    LogInfo("\nRc110CustomDetectNet -- loading detection network model from:\n");
    LogInfo("                     -- model        %s\n", CHECK_NULL_STR(modelPath));
    LogInfo("                     -- input_blob   '%s'\n", CHECK_NULL_STR(input));
    LogInfo("                     -- output_blob  '%s'\n", CHECK_NULL_STR(output));
    LogInfo("                     -- output_count '%s'\n", CHECK_NULL_STR(numDetections));
    LogInfo("                     -- class_labels %s\n", CHECK_NULL_STR(classLabels));
    LogInfo("                     -- threshold    %f\n", threshold);
    LogInfo("                     -- batch_size   %u\n\n", maxBatchSize);

    std::vector<std::string> outputBlobs;

    if (output)
    {
        outputBlobs.emplace_back(output);
    }

    if (numDetections)
    {
        outputBlobs.emplace_back(numDetections);
    }

    if (!net->LoadNetwork(nullptr, modelPath, nullptr, input, inputDims, outputBlobs, maxBatchSize, precision, device, allowGPUFallback))
    {
        LogError(LOG_TRT "detectNet -- failed to initialize.\n");
        return nullptr;
    }

    if (!net->allocDetections() || !net->loadClassInfo(classLabels))
    {
        return nullptr;
    }

    net->SetThreshold(threshold);
    return net;
    
}

int Rc110CustomDetectNet::CustomDetect(void* input, const std::uint32_t width, const std::uint32_t height,
                                       const imageFormat format, Detection* detections, const std::uint32_t overlay)
{
    if (!m_useDarknetYolo)
    {
        return detectNet::Detect(input, width, height, format, detections, overlay);
    }

    if (!IsModelType(MODEL_ONNX))
    {
        LogError(LOG_TRT "Rc110CustomDetectNet::CustomDetect() only support onnx format for darknet yolo model\n");
        return -1;
    }

    if (!input || width == 0 || height == 0 || !detections)
    {
        LogError(LOG_TRT "Rc110CustomDetectNet::CustomDetect( 0x%p, %u, %u ) -> invalid parameters\n", input, width, height);
        return -1;
    }

    if (!imageFormatIsRGB(format))
    {
        LogError(LOG_TRT "Rc110DetectNet::CustomDetect() -- unsupported image format (%s)\n", imageFormatToStr(format));
        LogError(LOG_TRT "supported formats are: rgb8, rgba8, rgba32f, rgba32f\n");
        return -1;
    }

    PROFILER_BEGIN(PROFILER_PREPROCESS);
    if (CUDA_FAILED(cudaTensorNormMeanRGB(input, format, width, height, mInputs[0].CUDA,
                                          GetInputWidth(), GetInputHeight(), make_float2(0.0f, 1.0f),
                                          make_float3(mMeanPixel, mMeanPixel, mMeanPixel),
                                          make_float3(m_stdPixel, m_stdPixel, m_stdPixel),
                                          GetStream())))
    {
        LogError(LOG_TRT "Rc110CustomDetectNet::CustomDetect() -- cudaTensorNormMeanRGB() failed\n");
        PROFILER_END(PROFILER_PREPROCESS);
        return -1;
    }
    PROFILER_END(PROFILER_PREPROCESS);

    PROFILER_BEGIN(PROFILER_NETWORK);
    if (!this->ProcessNetwork())
    {
        PROFILER_END(PROFILER_NETWORK);
        return -1;
    }
    PROFILER_END(PROFILER_NETWORK);

    PROFILER_BEGIN(PROFILER_POSTPROCESS);
    const int numDetections = this->PostProcess(width, height, detections);
    PROFILER_END(PROFILER_POSTPROCESS);

    CUDA(cudaDeviceSynchronize());

    return numDetections;
}

int Rc110CustomDetectNet::PostProcess(const std::uint32_t width, const std::uint32_t height, Detection* detections)
{
    int numDetections = 0;

    float* conf = mOutputs[OUTPUT_CONF_DARKNET_YOLO].CPU;
    float* bbox = mOutputs[OUTPUT_BBOX_DARKNET_YOLO].CPU;

    const std::uint32_t numBoxes = DIMS_C(mOutputs[OUTPUT_BBOX_DARKNET_YOLO].dims);
    const std::uint32_t numCoord = DIMS_W(mOutputs[OUTPUT_BBOX_DARKNET_YOLO].dims);

    for (std::uint32_t n = 0; n < numBoxes; n++)
    {
        std::uint32_t maxClass = 0;
        float maxScore = std::numeric_limits<float>::lowest();

        for (std::uint32_t m = 0; m < mNumClasses; m++)
        {
            const float score = conf[n * mNumClasses + m];

            if (score < mConfidenceThreshold)
            {
                continue;
            }
            if (score > maxScore)
            {
                maxScore = score;
                maxClass = m;
            }
        }

        if (maxScore < mConfidenceThreshold)
        {
            continue;
        }

        const float* coord = bbox + n * numCoord;

        detections[numDetections].Instance = numDetections;
        detections[numDetections].ClassID = maxClass;
        detections[numDetections].Confidence = maxScore;
        detections[numDetections].Left = coord[0] * width;
        detections[numDetections].Top = coord[1] * height;
        detections[numDetections].Right = coord[2] * width;
        detections[numDetections].Bottom = coord[3] * height;

        numDetections += this->clusterDetections(detections, numDetections);
    }
    this->sortDetections(detections, numDetections);

    for (int n = 0; n < numDetections; n++)
    {
        detections[n].Left = std::clamp<float>(detections[n].Left, 0, width - 1);
        detections[n].Right = std::clamp<float>(detections[n].Right, 0, width - 1);
        detections[n].Top = std::clamp<float>(detections[n].Top, 0, height - 1);
        detections[n].Bottom = std::clamp<float>(detections[n].Bottom, 0, height - 1);
    }

    return numDetections;
}

int Rc110CustomDetectNet::CustomDetect(void* input,
                                       const std::uint32_t width,
                                       const std::uint32_t height,
                                       const imageFormat format,
                                       Detection** detections,
                                       const std::uint32_t overlay)
{
    if (!detections)
    {
        return -1;
    }

    *detections = &(mDetectionSets[0]);

    if (++mDetectionSet >= mNumDetectionSets)
    {
        mDetectionSet = 0;
    }
    return this->CustomDetect(input, width, height, format, *detections, overlay);
}
}  // namespace zmp
