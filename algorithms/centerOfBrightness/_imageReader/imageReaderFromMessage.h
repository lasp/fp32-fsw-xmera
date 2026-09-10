#ifndef _IMAGE_READER_FROM_MESSAGE_H_
#define _IMAGE_READER_FROM_MESSAGE_H_

#include "../imageReaderInterface.h"
#include "msgPayloadDef/CameraImageMsgF32Payload.h"
#include <architecture/messaging/messaging.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

class ImageReaderFromMessage : public ImageReaderInterface {
   public:
    ImageReaderFromMessage() = default;
    ~ImageReaderFromMessage() override = default;
    Eigen::Vector2i getFullImageSize(int32_t cameraId) final;
    void getImageAsArray(const Eigen::Vector2i& center,
                         const Eigen::Vector2i& windowSize,
                         std::array<Eigen::Vector2i, kMaxWindowSize>& output) final;
    int64_t getCurrentImageTimeTag(int32_t cameraId, int64_t previousImageTimeTag) final;

    void setBlurSize(int blur);
    int getBlurSize() const;
    void setPixelThreshold(float threshold);
    float getPixelThreshold() const;

    ReadFunctor<CameraImageMsgF32Payload> imageInMsg;

   private:
    cv::Mat readImageFromMessage();

    CameraImageMsgF32Payload imagePayload{};
    uint64_t imageTimeTag{};
    Eigen::Vector2i imageSize{};  //!< [px] Full image dimensions (width, height)
    float pixelThreshold{};       //!< [-] minimum pixel brightness threshold used for detecting bright pixels
    int32_t blurSize{};           //!< [px] Size of the blurring box in pixels
};

#endif
