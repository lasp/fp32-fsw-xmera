#ifndef F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H
#define F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H

#include <stdint.h>
#include <Eigen/Core>
#include <memory>

#include "imageReaderInterface.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

/**
 * @brief Result struct for the center of brightness algorithm
 */
struct CenterOfBrightnessResult {
    Eigen::Vector2f centerOfBrightness = Eigen::Vector2f::Zero();
    int32_t pixelsFound{};
    float rollingAverageBrightness{};
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
 * @brief Validated configuration for the center of brightness algorithm
 */
class CenterOfBrightnessConfig final {
   public:
    static CenterOfBrightnessConfig create(float relativeBrightnessIncreaseThreshold,
                                           int32_t numberOfPointsBrightnessAverage) {
        if (!isValidRelativeBrightnessIncreaseThreshold(relativeBrightnessIncreaseThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("centerOfBrightness: relativeBrightnessIncreaseThreshold must be non-negative.");
        }
        if (!isValidNumberOfPointsBrightnessAverage(numberOfPointsBrightnessAverage)) {
            FSW_THROW_INVALID_ARGUMENT("centerOfBrightness: numberOfPointsBrightnessAverage must be positive.");
        }
        return {relativeBrightnessIncreaseThreshold, numberOfPointsBrightnessAverage};
    }

    // +inf is a legitimate value (an intentionally unreachable threshold); only reject NaN and
    // negative values. NaN comparisons are always false, so this rejects NaN too.
    static bool isValidRelativeBrightnessIncreaseThreshold(float relativeBrightnessIncreaseThreshold) {
        return relativeBrightnessIncreaseThreshold >= 0.0F;
    }
    static bool isValidNumberOfPointsBrightnessAverage(int32_t numberOfPointsBrightnessAverage) {
        return numberOfPointsBrightnessAverage > 0;
    }

    float getRelativeBrightnessIncreaseThreshold() const { return relativeBrightnessIncreaseThreshold; }
    int32_t getNumberOfPointsBrightnessAverage() const { return numberOfPointsBrightnessAverage; }

   private:
    CenterOfBrightnessConfig(float relativeBrightnessIncreaseThreshold, int32_t numberOfPointsBrightnessAverage)
        : relativeBrightnessIncreaseThreshold(relativeBrightnessIncreaseThreshold),
          numberOfPointsBrightnessAverage(numberOfPointsBrightnessAverage) {}

    float relativeBrightnessIncreaseThreshold;
    int32_t numberOfPointsBrightnessAverage;
};

/**
 * @brief Center of brightness detection algorithm
 *
 * Processes an image via an ImageReaderInterface to find the unweighted
 * center of brightness of non-zero pixels.
 */
class CenterOfBrightnessAlgorithm final {
   public:
    explicit CenterOfBrightnessAlgorithm(const CenterOfBrightnessConfig& config);
    ~CenterOfBrightnessAlgorithm();

    CenterOfBrightnessResult update(const CobRegionOfInterest& roi, ImageReaderInterface& imageReader);
    void reset();
    void setConfig(const CenterOfBrightnessConfig& config);

   private:
    CenterOfBrightnessResult findCob(const CobRegionOfInterest& roi, ImageReaderInterface& imageReader);
    std::pair<Eigen::Vector2f, int32_t> computeCenterOfBrightness(
        const std::array<Eigen::Vector2i, kMaxWindowSize>& pixels) const;
    float computeBrightnessIncrease(int32_t pixelsFound);
    void updateBrightnessHistory(float brightness);

    CenterOfBrightnessConfig cfg;
    std::unique_ptr<std::array<Eigen::Vector2i, kMaxWindowSize>> pixelBuffer =
        std::make_unique<std::array<Eigen::Vector2i, kMaxWindowSize>>();
    Eigen::VectorXf brightnessHistory{};  //!< [-] brightness history to be used for rolling average
};

#endif  // F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H
