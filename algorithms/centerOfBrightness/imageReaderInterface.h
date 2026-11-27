#ifndef F32XMERA_IMAGE_READER_INTERFACE_H
#define F32XMERA_IMAGE_READER_INTERFACE_H

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

#include <cstdint>

/*! @brief Image source that the CenterOfBrightness adapter uses to get a thresholded image,
 *  independent of how or where the underlying image is sourced.
 */
class ImageReaderInterface {
   public:
    virtual ~ImageReaderInterface() = default;

    /*! Get the full pixel dimensions of the image produced by the given camera.
     @return image size as (width, height) in pixels
     @param cameraId Id of the camera of interest
     */
    virtual Eigen::Vector2i getFullImageSize(int32_t cameraId) = 0;

    /*! Get the time tag of the current image if it is newer than previousImageTimeTag.
     @return time tag of the new image, or a value <= previousImageTimeTag if no new image is present
     @param cameraId Id of the camera of interest
     @param previousImageTimeTag Time tag of the most recently processed image
     */
    virtual int64_t getCurrentImageTimeTag(int32_t cameraId, int64_t previousImageTimeTag) = 0;

    /*! Get the full thresholded image. The reader converts the image to grayscale, then does a box blur
     *  and a binary threshold on it.
     @return single-channel 8-bit image (CV_8UC1) where bright pixels are 255 and all others are 0,
             or an empty image if no image is available
     */
    virtual cv::Mat getImage() = 0;
};

#endif  // F32XMERA_IMAGE_READER_INTERFACE_H
