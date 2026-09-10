#ifndef F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H
#define F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H

#include <stdint.h>
#include <Eigen/Core>
#include <memory>

#include "imageReaderInterface.h"

/**
 * @brief Result struct for the center of brightness algorithm
 */
struct CenterOfBrightnessResult {
    Eigen::Vector2d centerOfBrightness = Eigen::Vector2d::Zero();
    int32_t pixelsFound{};
    double rollingAverageBrightness{};
    bool valid{};
    bool noPixelTrigger{false};
    bool notExceedingBrightnessIncreaseTrigger{false};
};

/**
 * @brief Bespoke input struct for the algorithm — no msg payload dependency
 */
struct CobRegionOfInterest {
    Eigen::Vector2i center = Eigen::Vector2i::Zero();
    Eigen::Vector2i size = Eigen::Vector2i::Zero();
};

/**
 * @brief Center of brightness detection algorithm
 *
 * Processes an image via an ImageReaderInterface to find the unweighted
 * center of brightness of non-zero pixels.
 */
class CenterOfBrightnessAlgorithm final {
   public:
    CenterOfBrightnessAlgorithm();
    ~CenterOfBrightnessAlgorithm();

    CenterOfBrightnessResult update(const CobRegionOfInterest& roi, ImageReaderInterface& imageReader);
    void reset();

    void setRelativeBrightnessIncreaseThreshold(double increaseThreshold);
    double getRelativeBrightnessIncreaseThreshold() const;
    void setNumberOfPointsBrightnessAverage(int32_t rollingAverage);
    int32_t getNumberOfPointsBrightnessAverage() const;

   private:
    CenterOfBrightnessResult findCob(const CobRegionOfInterest& roi, ImageReaderInterface& imageReader);
    std::pair<Eigen::Vector2d, int32_t> computeCenterOfBrightness(
        const std::array<Eigen::Vector2i, kMaxWindowSize>& pixels) const;
    double computeBrightnessIncrease(int32_t pixelsFound);
    void updateBrightnessHistory(double brightness);

    std::unique_ptr<std::array<Eigen::Vector2i, kMaxWindowSize>> pixelBuffer =
        std::make_unique<std::array<Eigen::Vector2i, kMaxWindowSize>>();
    Eigen::VectorXd brightnessHistory{};           //!< [-] brightness history to be used for rolling average
    double relativeBrightnessIncreaseThreshold{};  //!< [-] minimum relative brightness increase (if less, invalidated)
    int32_t numberOfPointsBrightnessAverage{};  //!< [-] number of points to be used for rolling average of brightness
};

#endif  // F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H
