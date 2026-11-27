#ifndef _IMAGE_READER_UTILITIES_H_
#define _IMAGE_READER_UTILITIES_H_

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>

/*! Convert a BGR cv::Mat to grayscale, then do a box blur and a binary threshold on it.
 *  @return single-channel 8-bit image (CV_8UC1) with bright pixels set to 255 and all others set to 0
 */
inline cv::Mat thresholdImage(const cv::Mat& mat, int32_t blurSize, float pixelThreshold) {
    cv::Mat imageGray;
    cv::Mat blured;
    cv::Mat thresholded;

    cv::cvtColor(mat, imageGray, cv::COLOR_BGR2GRAY);
    cv::blur(imageGray, blured, cv::Size(blurSize, blurSize));
    // cv::threshold requires a double threshold value (OpenCV API boundary).
    cv::threshold(blured, thresholded, static_cast<double>(pixelThreshold), 255, cv::THRESH_BINARY);
    return thresholded;
}

#endif
