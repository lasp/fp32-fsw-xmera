#include "centerOfBrightnessAlgorithm.h"

#include <algorithm>

namespace {

/*! Half-open pixel ranges [rowBegin, rowEnd) x [colBegin, colEnd) inside the image */
struct PixelWindow {
    int32_t rowBegin;
    int32_t rowEnd;
    int32_t colBegin;
    int32_t colEnd;
};

/*! Make sure that the image view describes a buffer that the function can read */
bool isValidImage(const ImageView_c& image) {
    return image.pixels != nullptr && image.rows > 0 && image.cols > 0 && image.strideBytes >= image.cols;
}

/*! Clip the region of interest to the image bounds.
 @return pixel window. The window is empty (begin >= end) when the region is fully outside the image
 @param image Image view
 @param roi Region of interest in full-image coordinates
 */
PixelWindow clipToImage(const ImageView_c& image, const RegionOfInterest_c& roi) {
    // The function uses 64-bit arithmetic. Thus int32 inputs at their limits cannot cause an overflow.
    const int64_t colStart = static_cast<int64_t>(roi.centerX) - roi.width / 2;
    const int64_t rowStart = static_cast<int64_t>(roi.centerY) - roi.height / 2;
    const int64_t colStop = colStart + roi.width;
    const int64_t rowStop = rowStart + roi.height;
    return {.rowBegin = static_cast<int32_t>(std::clamp<int64_t>(rowStart, 0, image.rows)),
            .rowEnd = static_cast<int32_t>(std::clamp<int64_t>(rowStop, 0, image.rows)),
            .colBegin = static_cast<int32_t>(std::clamp<int64_t>(colStart, 0, image.cols)),
            .colEnd = static_cast<int32_t>(std::clamp<int64_t>(colStop, 0, image.cols))};
}

/*! Get the address of the first pixel of a row */
const uint8_t* rowPointer(const ImageView_c& image, int32_t row) {
    return image.pixels + static_cast<std::ptrdiff_t>(row) * image.strideBytes;
}

/*! Centroid and count of the bright pixels in a window */
struct BrightPixels {
    Eigen::Vector2f centroid;  //!< [px] [0] = column (x), [1] = row (y). Zero when count is 0.
    int32_t count;             //!< [-] number of non-zero pixels
};

/*! Calculate sum / count as a float. The function does not convert the large sum to float.
 * The quotient is a pixel coordinate, so float holds it with no error. Only the fraction has a rounding error.
 @return mean value
 @param sum Non-negative sum of pixel coordinates
 @param count Number of coordinates in the sum. It must be more than 0.
 */
float meanOf(const int64_t sum, const int32_t count) {
    const int64_t quotient = sum / count;
    const int64_t remainder = sum % count;
    return static_cast<float>(quotient) + static_cast<float>(remainder) / static_cast<float>(count);
}

/*! Calculate the unweighted centroid of the non-zero pixels inside a window of the image.
 * The function adds the coordinates in 64-bit integers. Float sums do not keep integer precision for
 * values more than 2^24.
 @return centroid and count of the bright pixels
 @param image Correct image view (isValidImage is true)
 @param window Window inside the image bounds
 */
BrightPixels computeBrightPixels(const ImageView_c& image, const PixelWindow& window) {
    const auto [rowBegin, rowEnd, colBegin, colEnd] = window;

    int64_t sumX = 0;
    int64_t sumY = 0;
    int32_t count = 0;
    for (int32_t row = rowBegin; row < rowEnd; ++row) {
        const uint8_t* const rowPixels = rowPointer(image, row);
        for (int32_t col = colBegin; col < colEnd; ++col) {
            if (rowPixels[col] == 0U) {
                continue;
            }
            sumX += col;
            sumY += row;
            ++count;
        }
    }
    if (count == 0) {
        return {.centroid = Eigen::Vector2f::Zero(), .count = 0};
    }
    return {.centroid = Eigen::Vector2f(meanOf(sumX, count), meanOf(sumY, count)), .count = count};
}

}  // namespace

CenterOfBrightnessAlgorithm::CenterOfBrightnessAlgorithm(const CenterOfBrightnessConfig& config) : cfg(config) {}

CenterOfBrightnessAlgorithm::~CenterOfBrightnessAlgorithm() = default;

void CenterOfBrightnessAlgorithm::setConfig(const CenterOfBrightnessConfig& config) { this->cfg = config; }

/*! Reset algorithm state: clears brightness history */
void CenterOfBrightnessAlgorithm::reset() { this->brightnessHistory.resize(0); }

/*! Primary entry point: calculates the center of brightness of the non-zero pixels inside a region of interest.
 * The function clips the region to the image bounds. If the image view is incorrect or the region is
 * fully outside the image, the function finds no bright pixels. Then the result has only noPixelTrigger set,
 * and the brightness history does not change.
 @return CenterOfBrightnessResult with coordinates in full-image pixels, [0] = column (x), [1] = row (y)
 @param image Single-channel 8-bit image. Non-zero pixels are bright.
 @param roi Region of interest in full-image pixel coordinates
 */
CenterOfBrightnessResult CenterOfBrightnessAlgorithm::update(const ImageView_c& image, const RegionOfInterest_c& roi) {
    if (!isValidImage(image)) {
        return this->evaluateBrightness(Eigen::Vector2f::Zero(), 0);
    }
    const auto [centroid, pixelsFound] = computeBrightPixels(image, clipToImage(image, roi));
    return this->evaluateBrightness(centroid, pixelsFound);
}

/*! Do the brightness-increase check on a calculated centroid and update the brightness history.
 @return CenterOfBrightnessResult
 @param coordinates Centroid of the bright pixels
 @param pixelsFound Number of bright pixels
 */
CenterOfBrightnessResult CenterOfBrightnessAlgorithm::evaluateBrightness(const Eigen::Vector2f& coordinates,
                                                                         const int32_t pixelsFound) {
    CenterOfBrightnessResult result{.noPixelTrigger = pixelsFound == 0};
    if (pixelsFound > 0) {
        const float brightnessIncrease = this->computeBrightnessIncrease(pixelsFound);
        if (brightnessIncrease >= this->cfg.getRelativeBrightnessIncreaseThreshold()) {
            result.valid = true;
            result.centerOfBrightness = coordinates;
            result.pixelsFound = pixelsFound;
        } else {
            result.notExceedingBrightnessIncreaseTrigger = true;
        }
        result.rollingAverageBrightness = this->brightnessHistory.mean();
    }
    return result;
}

/*! Compute relative brightness increase from the rolling average.
 @return relative brightness increase
 @param pixelsFound Number of bright pixels found this timestep
 */
float CenterOfBrightnessAlgorithm::computeBrightnessIncrease(const int32_t pixelsFound) {
    float averageBrightnessOld = 0.0F;
    if (this->brightnessHistory.rows() > 0) {
        averageBrightnessOld = this->brightnessHistory.mean();
    }
    this->updateBrightnessHistory(static_cast<float>(pixelsFound));
    const float averageBrightnessNew = this->brightnessHistory.mean();
    float brightnessIncrease = 0.0F;
    if (averageBrightnessOld > 0.0F) {
        brightnessIncrease = (averageBrightnessNew - averageBrightnessOld) / averageBrightnessOld;
    }
    return brightnessIncrease;
}

/*! Update brightness history by shifting back previous brightness values and updating most recent one
    @return void
    @param brightness total brightness of current time step
    */
void CenterOfBrightnessAlgorithm::updateBrightnessHistory(const float brightness) {
    // increase vector size if it is not at its full size yet
    if (this->brightnessHistory.rows() < this->cfg.getNumberOfPointsBrightnessAverage()) {
        this->brightnessHistory.conservativeResize(this->brightnessHistory.rows() + 1, 1);
    }
    // shift previous brightness values back (only if number of data points for rolling average is greater than 1)
    if (this->brightnessHistory.rows() > 1) {
        for (auto i = static_cast<int>(this->brightnessHistory.rows()) - 1; i > 0; --i) {
            this->brightnessHistory[i] = this->brightnessHistory[i - 1];
        }
    }
    // update most recent brightness value
    this->brightnessHistory[0] = brightness;
}
