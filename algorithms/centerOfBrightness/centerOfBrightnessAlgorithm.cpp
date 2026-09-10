#include "centerOfBrightnessAlgorithm.h"

CenterOfBrightnessAlgorithm::CenterOfBrightnessAlgorithm(const CenterOfBrightnessConfig& config) : cfg(config) {}

CenterOfBrightnessAlgorithm::~CenterOfBrightnessAlgorithm() = default;

void CenterOfBrightnessAlgorithm::setConfig(const CenterOfBrightnessConfig& config) { this->cfg = config; }

/*! Reset algorithm state: clears brightness history */
void CenterOfBrightnessAlgorithm::reset() { this->brightnessHistory.resize(0); }

/*! Main entry point: reads the windowed image via imageReader and computes the center of brightness.
 @return CenterOfBrightnessResult
 @param roi Region of interest for windowing
 @param imageReader Image reader providing pixel data
 */
CenterOfBrightnessResult CenterOfBrightnessAlgorithm::update(const CobRegionOfInterest& roi,
                                                             ImageReaderInterface& imageReader) {
    CenterOfBrightnessResult result{};
    imageReader.getImageAsArray(roi.center, roi.size, *this->pixelBuffer);
    const auto [coordinates, pixelsFound] = this->computeCenterOfBrightness(*this->pixelBuffer);

    if (pixelsFound > 0) {
        const float brightnessIncrease = this->computeBrightnessIncrease(pixelsFound);
        result.noPixelTrigger = false;
        if (brightnessIncrease >= this->cfg.getRelativeBrightnessIncreaseThreshold()) {
            result.valid = true;
            result.centerOfBrightness = coordinates;
            result.pixelsFound = pixelsFound;
            result.notExceedingBrightnessIncreaseTrigger = false;
        }
        result.rollingAverageBrightness = this->brightnessHistory.mean();
    }
    return result;
}

/*! Compute the unweighted centroid of non-zero pixels. (0,0) is treated as sentinel / invalid.
 @return pair of centroid coordinates and count of valid pixels
 @param pixels Array of pixel coordinates from image reader
 */
std::pair<Eigen::Vector2f, int32_t> CenterOfBrightnessAlgorithm::computeCenterOfBrightness(
    const std::array<Eigen::Vector2i, kMaxWindowSize>& pixels) const {
    Eigen::Vector2f coordinates = Eigen::Vector2f::Zero();
    int32_t count = 0;
    for (const auto& pixel : pixels) {
        if (pixel.isZero()) continue;
        coordinates[0] += pixel[0];
        coordinates[1] += pixel[1];
        ++count;
    }
    if (count > 0) {
        coordinates /= static_cast<float>(count);
    }
    return {coordinates, count};
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
