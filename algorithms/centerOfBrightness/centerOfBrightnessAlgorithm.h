#ifndef F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H
#define F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H

#include <stdint.h>
#include <Eigen/Core>
#include <array>
#include <utility>

#include "utilities/fsw/freestandingInvalidArgument.h"

//!< [-] Maximum number of non-zero pixel coordinates the algorithm can consume per update()
constexpr int kMaxWindowSize = 1024 * 1024;

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
 * Computes the unweighted center of brightness from a caller-supplied array of non-zero
 * pixel coordinates. Image acquisition is not this class's concern: the adapter reads the
 * image via an ImageReaderInterface and passes the resulting pixel array to update().
 */
class CenterOfBrightnessAlgorithm final {
   public:
    explicit CenterOfBrightnessAlgorithm(const CenterOfBrightnessConfig& config);
    ~CenterOfBrightnessAlgorithm();

    CenterOfBrightnessResult update(const std::array<Eigen::Vector2i, kMaxWindowSize>& pixels);
    void reset();
    void setConfig(const CenterOfBrightnessConfig& config);

   private:
    std::pair<Eigen::Vector2f, int32_t> computeCenterOfBrightness(
        const std::array<Eigen::Vector2i, kMaxWindowSize>& pixels) const;
    float computeBrightnessIncrease(int32_t pixelsFound);
    void updateBrightnessHistory(float brightness);

    CenterOfBrightnessConfig cfg;
    Eigen::VectorXf brightnessHistory{};  //!< [-] brightness history to be used for rolling average
};

#endif  // F32XMERA_CENTER_OF_BRIGHTNESS_ALGORITHM_H
