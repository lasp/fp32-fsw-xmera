#ifndef F32XMERA_SUN_SEARCH_POINT_ALGORITHM_H
#define F32XMERA_SUN_SEARCH_POINT_ALGORITHM_H

#include "sunSearchPointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <math.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>

inline constexpr uint32_t kNumRotations = SUN_SEARCH_POINT_NUM_ROTATIONS;

enum class RotationAxis { b1Hat_B = 0, b2Hat_B = 1, b3Hat_B = 2 };

/*! @brief Properties defining a single sun-search rotation maneuver. */
struct RotationProperties {
    float rotationDuration{};                         /*!< [s] duration of this rotation */
    float rotationRate{};                             /*!< [rad/s] commanded scalar component body rate (signed) */
    RotationAxis rotationAxis{RotationAxis::b1Hat_B}; /*!< [-] axis about which to rotate */
};

/*! @brief Structure containing the sun search point attitude guidance output. */
struct SunSearchPointOutput {
    Eigen::Vector3f sigma_BR;    //!< attitude error (MRPs) of B relative to R
    Eigen::Vector3f omega_BR_B;  //!< [rad/s] body rate error of B relative to R in B frame
    Eigen::Vector3f omega_RN_B;  //!< [rad/s] reference frame rate of R relative to N in B frame
    bool faultDetected{false};   //!< [-] true once the search fails to acquire the sun (forced to pointing)
};

/**
 * @brief Validated configuration for the sun search point algorithm.
 *
 * Holds the sun-search rotation sequence and the pointing parameters. An instance can only exist if
 * every rotation has a finite, positive duration, a finite commanded rate, and a valid axis, and if
 * sHatBdyCmd has unit norm (within 1e-3). Construct via SunSearchPointConfig::create(...).
 */
class SunSearchPointConfig final {
   public:
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    static SunSearchPointConfig create(const std::array<RotationProperties, kNumRotations>& rotations,
                                       const Eigen::Vector3f& sHatBdyCmd,
                                       float sunAxisSpinRate,
                                       const Eigen::Vector3f& omega_RN_B,
                                       int observationThreshold) {
        for (auto [rotationDuration, rotationRate, rotationAxis] : rotations) {
            if (!isValidRotationDuration(rotationDuration)) {
                FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: rotationDuration must be finite and > 0");
            }
            if (!isValidRotationRate(rotationRate)) {
                FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: rotationRate must be finite");
            }
            if (!isValidRotationAxis(rotationAxis)) {
                FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: rotationAxis must be b1Hat_B, b2Hat_B, or b3Hat_B");
            }
        }
        if (!isValidSHatBdyCmd(sHatBdyCmd)) {
            FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: sHatBdyCmd norm must be within 1e-3 of 1.0");
        }
        return SunSearchPointConfig{
            rotations, sHatBdyCmd.normalized(), sunAxisSpinRate, omega_RN_B, observationThreshold};
    }

    static bool isValidRotationDuration(float time) { return fsw::is_finite(time) && time > 0.0F; }
    static bool isValidRotationRate(float rate) { return fsw::is_finite(rate); }
    static bool isValidRotationAxis(RotationAxis axis) {
        return axis == RotationAxis::b1Hat_B || axis == RotationAxis::b2Hat_B || axis == RotationAxis::b3Hat_B;
    }
    static bool isValidSHatBdyCmd(const Eigen::Vector3f& sHatBdyCmd) {
        constexpr float kUnitNormTol = 1e-3F;
        return fabsf(sHatBdyCmd.norm() - 1.0F) <= kUnitNormTol;
    }

    const std::array<RotationProperties, kNumRotations>& getRotations() const { return rotations; }
    const Eigen::Vector3f& getSHatBdyCmd() const { return sHatBdyCmd; }
    float getSunAxisSpinRate() const { return sunAxisSpinRate; }
    const Eigen::Vector3f& getOmega_RN_B() const { return omega_RN_B; }
    int getObservationThreshold() const { return observationThreshold; }

   private:
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    SunSearchPointConfig(const std::array<RotationProperties, kNumRotations>& rotationsIn,
                         const Eigen::Vector3f& sHatBdyCmdIn,
                         float sunAxisSpinRateIn,
                         const Eigen::Vector3f& omega_RN_BIn,
                         int observationThresholdIn)
        : rotations(rotationsIn),
          sHatBdyCmd(sHatBdyCmdIn),
          sunAxisSpinRate(sunAxisSpinRateIn),
          omega_RN_B(omega_RN_BIn),
          observationThreshold(observationThresholdIn) {}

    std::array<RotationProperties, kNumRotations> rotations;
    Eigen::Vector3f sHatBdyCmd;
    float sunAxisSpinRate;
    Eigen::Vector3f omega_RN_B;
    int observationThreshold;
};

/**
 * @brief Sun search point attitude guidance algorithm.
 *
 * Drives a one-way safe-mode startup: a scripted sun-search rotation sequence followed by
 * closed-loop sun pointing. The first rotation always runs to completion; after that, the
 * algorithm transitions to pointing once the coarse-sun-sensor observation count reaches the
 * configured threshold, or unconditionally once the full rotation sequence has elapsed. The
 * pointing phase is terminal (re-armed only via reset()).
 */
class SunSearchPointAlgorithm final {
   public:
    explicit SunSearchPointAlgorithm(const SunSearchPointConfig& config);
    ~SunSearchPointAlgorithm() = default;

    SunSearchPointOutput update(uint64_t callTime,
                                const Eigen::Vector3f& rHat_SB_B,
                                const Eigen::Vector3f& omega_BN_B,
                                int numCssViewingSun);

    void setConfig(const SunSearchPointConfig& config);
    void reInitialize();

   private:
    enum class Phase { Searching, Pointing };

    SunSearchPointOutput computeSearch(uint64_t callTime, const Eigen::Vector3f& omega_BN_B) const;
    SunSearchPointOutput computePointing(const Eigen::Vector3f& rHat_SB_B, const Eigen::Vector3f& omega_BN_B) const;
    void precomputeEndTimes();

    SunSearchPointConfig cfg;                                //!< validated configuration (rotations + pointing params)
    std::array<uint64_t, kNumRotations> rotationEndTimes{};  //!< [ns] cumulative end time of each rotation
    uint64_t searchStartTime{};                              //!< [ns] time at which the rotation sequence begins
    bool firstPass{true};                                    //!< [-] true until the start time has been captured
    Phase phase{Phase::Searching};                           //!< [-] current guidance phase (Pointing is terminal)
    bool searchFailed{false};  //!< [-] latched true if the sequence elapsed without acquiring the sun
};

#endif
