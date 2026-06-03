#ifndef F32XMERA_SUN_SEARCH_ALGORITHM_H
#define F32XMERA_SUN_SEARCH_ALGORITHM_H

#include "sunSearchTypes.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/freestandingIsFinite.hpp"

#include <Eigen/Core>
#include <array>
#include <cstdint>

inline constexpr uint32_t kNumRotations = SUN_SEARCH_NUM_ROTATIONS;

enum class RotationAxis { b1Hat_B = 0, b2Hat_B = 1, b3Hat_B = 2 };

/**
 * @brief Properties defining a single rotation maneuver.
 */
struct RotationProperties {
    float rotationDuration{};                         /*!< [s] duration of this rotation */
    float rotationRate{};                             /*!< [rad/s] commanded scalar component body rate (signed) */
    RotationAxis rotationAxis{RotationAxis::b1Hat_B}; /*!< [-] axis about which to rotate */
};

/**
 * @brief Output from the sun search algorithm.
 */
struct SunSearchOutput {
    Eigen::Vector3f omega_RN_B{Eigen::Vector3f::Zero()}; /*!< [rad/s] reference angular velocity in body frame */
    Eigen::Vector3f omega_BR_B{Eigen::Vector3f::Zero()}; /*!< [rad/s] body-rate error in body frame */
};

/**
 * @brief Validated rotation-sequence configuration for the sun-search algorithm.
 *
 * An instance of this class can only exist if every rotation in the sequence
 * has a finite, positive duration, a finite commanded rate, and a valid axis.
 * Construct via SunSearchConfig::create(...).
 */
class SunSearchConfig final {
   public:
    static SunSearchConfig create(const std::array<RotationProperties, kNumRotations>& rotations) {
        for (auto [rotationDuration, rotationRate, rotationAxis] : rotations) {
            if (!isValidRotationDuration(rotationDuration)) {
                FSW_THROW_INVALID_ARGUMENT("sunSearch: rotationDuration must be finite and > 0");
            }
            if (!isValidRotationRate(rotationRate)) {
                FSW_THROW_INVALID_ARGUMENT("sunSearch: rotationRate must be finite");
            }
            if (!isValidRotationAxis(rotationAxis)) {
                FSW_THROW_INVALID_ARGUMENT("sunSearch: rotationAxis must be b1Hat_B, b2Hat_B, or b3Hat_B");
            }
        }
        return SunSearchConfig{rotations};
    }

    static bool isValidRotationDuration(float time) { return fsw::is_finite(time) && time > 0.0F; }
    static bool isValidRotationRate(float rate) { return fsw::is_finite(rate); }
    static bool isValidRotationAxis(RotationAxis axis) {
        return axis == RotationAxis::b1Hat_B || axis == RotationAxis::b2Hat_B || axis == RotationAxis::b3Hat_B;
    }

    const std::array<RotationProperties, kNumRotations>& getRotations() const { return rotations; }

   private:
    explicit SunSearchConfig(const std::array<RotationProperties, kNumRotations>& rotationsIn)
        : rotations(rotationsIn) {}

    std::array<RotationProperties, kNumRotations> rotations;
};

class SunSearchAlgorithm final {
   public:
    explicit SunSearchAlgorithm(const SunSearchConfig& config);

    void setConfig(const SunSearchConfig& config);
    SunSearchOutput update(uint64_t callTime, const Eigen::Vector3f& omega_BN_B);

   private:
    void precomputeEndTimes();

    SunSearchConfig cfg;
    std::array<uint64_t, kNumRotations> rotationEndTimes{};  //!< [ns] cumulative end time of each rotation
    uint64_t sunSearchStartTime{};                           //!< [ns] time at which the rotation sequence begins
    bool firstPass{true};                                    //!< [-] true until the start time has been captured
};

#endif
