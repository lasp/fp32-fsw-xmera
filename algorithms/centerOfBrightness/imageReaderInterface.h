#ifndef F32XMERA_IMAGE_READER_INTERFACE_H
#define F32XMERA_IMAGE_READER_INTERFACE_H

#include <Eigen/Core>

constexpr int kMaxWindowSize = 1024 * 1024;

class ImageReaderInterface {
   public:
    virtual ~ImageReaderInterface() = default;
    virtual Eigen::Vector2i getFullImageSize(int32_t cameraId) = 0;
    virtual int64_t getCurrentImageTimeTag(int32_t cameraId, int64_t previousImageTimeTag) = 0;
    virtual void getImageAsArray(const Eigen::Vector2i& center,
                                 const Eigen::Vector2i& windowSize,
                                 std::array<Eigen::Vector2i, kMaxWindowSize>& output) = 0;
};

#endif  // F32XMERA_IMAGE_READER_INTERFACE_H
