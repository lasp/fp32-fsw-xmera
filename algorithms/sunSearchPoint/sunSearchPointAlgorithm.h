#ifndef F32XMERA_SUN_SEARCH_POINT_ALGORITHM_H
#define F32XMERA_SUN_SEARCH_POINT_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "sunSearchPointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/timeConversion.hpp"

#include <math.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <limits>

inline constexpr uint32_t kNumSunSearchRotations = SUN_SEARCH_POINT_NUM_ROTATIONS;

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
    bool sunNotFound{false};     //!< [-] true once the search fails to acquire the sun (forced to pointing)
};

/**
 * @brief Validated configuration for the sun search point algorithm.
 *
 * Holds the sun-search rotation sequence and the pointing parameters. An instance can only exist if
 * every rotation has a duration of at least 1 ns, a finite commanded rate, and a valid axis, if
 * sHatBdyCmd has unit norm (within 1e-3), if sunAxisSpinRate and omega_RN_B are finite, if
 * observationThreshold is at most kMaxNumCssSensors, if controlPeriod is at least 1 ns, if the whole
 * rotation sequence plus one control period is in the uint64_t nanosecond range, and if
 * deadbandAngle is finite and non-negative. deadbandAngle is an attitude deadband: while pointing, the
 * algorithm controls to the band edge, not to zero error. The commanded sun angle error is
 * max(error - deadbandAngle, 0), so it does not jump at the band edge.
 * Construct via SunSearchPointConfig::create(...).
 */
class SunSearchPointConfig final {
   public:
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    static SunSearchPointConfig create(const std::array<RotationProperties, kNumSunSearchRotations>& rotations,
                                       const Eigen::Vector3f& sHatBdyCmd,
                                       float sunAxisSpinRate,
                                       const Eigen::Vector3f& omega_RN_B,
                                       uint32_t observationThreshold,
                                       float controlPeriod,
                                       float deadbandAngle) {
        for (auto [rotationDuration, rotationRate, rotationAxis] : rotations) {
            if (!isValidRotationDuration(rotationDuration)) {
                FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: rotationDuration must be finite and >= 1 ns");
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
        if (!isValidSunAxisSpinRate(sunAxisSpinRate)) {
            FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: sunAxisSpinRate must be finite");
        }
        if (!isValidOmega_RN_B(omega_RN_B)) {
            FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: omega_RN_B must be finite");
        }
        if (!isValidObservationThreshold(observationThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: observationThreshold must be <= kMaxNumCssSensors");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: controlPeriod must be finite and >= 1 ns");
        }
        if (!isValidSequenceTimeline(rotations, controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT(
                "sunSearchPoint: total rotation duration plus one controlPeriod must fit in uint64_t ns");
        }
        if (!isValidDeadband(deadbandAngle)) {
            FSW_THROW_INVALID_ARGUMENT("sunSearchPoint: deadbandAngle must be finite and >= 0");
        }
        return SunSearchPointConfig{rotations,
                                    sHatBdyCmd.stableNormalized(),
                                    sunAxisSpinRate,
                                    omega_RN_B,
                                    observationThreshold,
                                    controlPeriod,
                                    deadbandAngle};
    }

    // A duration must convert to at least 1 ns. The validator rejects NaN, Inf, negative, and
    // sub-nanosecond values, and values that overflow uint64_t.
    static bool isValidRotationDuration(float time) { return fsw::secondsToNanos(time).value_or(0U) > 0U; }
    static bool isValidRotationRate(float rate) { return fsw::is_finite(rate); }
    static bool isValidRotationAxis(RotationAxis axis) {
        return axis == RotationAxis::b1Hat_B || axis == RotationAxis::b2Hat_B || axis == RotationAxis::b3Hat_B;
    }
    static bool isValidSHatBdyCmd(const Eigen::Vector3f& sHatBdyCmd) {
        constexpr float kUnitNormTol = 1e-3F;
        return fabsf(sHatBdyCmd.stableNorm() - 1.0F) <= kUnitNormTol;
    }
    static bool isValidSunAxisSpinRate(float sunAxisSpinRate) { return fsw::is_finite(sunAxisSpinRate); }
    static bool isValidOmega_RN_B(const Eigen::Vector3f& omega_RN_B) { return omega_RN_B.allFinite(); }
    static bool isValidDeadband(float deadband) { return fsw::is_finite(deadband) && deadband >= 0.0F; }

    // Each update() adds the period to the search timeline. A period that rounds to 0 ns never stops the
    // search.
    static bool isValidControlPeriod(float controlPeriod) {
        return fsw::secondsToNanos(controlPeriod).value_or(0U) > 0U;
    }
    // The timeline can stop one control period after the cumulative end time of the last rotation.
    // This sum must not wrap the uint64_t nanosecond counter.
    static bool isValidSequenceTimeline(const std::array<RotationProperties, kNumSunSearchRotations>& rotations,
                                        float controlPeriod) {
        std::uint64_t totalNs = fsw::secondsToNanos(controlPeriod).value_or(0U);
        for (const RotationProperties& rotation : rotations) {
            const std::uint64_t durationNs = fsw::secondsToNanos(rotation.rotationDuration).value_or(0U);
            // Check totalNs + durationNs <= max without calculating the sum. In uint64_t, a sum that is more than
            // max wraps to a small value, so "sum > max" can never be true. max - totalNs does not wrap, because
            // totalNs <= max. It is the space that is available above totalNs.
            if (durationNs > std::numeric_limits<std::uint64_t>::max() - totalNs) {
                return false;
            }
            totalNs += durationNs;
        }
        return true;
    }
    // Zero is meaningful: it transitions to pointing as soon as the first rotation completes. A
    // threshold above the sensor count is not, because it can never be met, so the sequence always
    // elapses and the fault latches on every run whether or not the sun was there.
    static bool isValidObservationThreshold(uint32_t observationThreshold) {
        return observationThreshold <= kMaxNumCssSensors;
    }

    const std::array<RotationProperties, kNumSunSearchRotations>& getRotations() const { return rotations; }
    const Eigen::Vector3f& getSHatBdyCmd() const { return sHatBdyCmd; }
    float getSunAxisSpinRate() const { return sunAxisSpinRate; }
    const Eigen::Vector3f& getOmega_RN_B() const { return omega_RN_B; }
    uint32_t getObservationThreshold() const { return observationThreshold; }
    float getControlPeriod() const { return controlPeriod; }
    float getDeadbandAngle() const { return deadbandAngle; }

   private:
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    SunSearchPointConfig(const std::array<RotationProperties, kNumSunSearchRotations>& rotationsIn,
                         const Eigen::Vector3f& sHatBdyCmdIn,
                         float sunAxisSpinRateIn,
                         const Eigen::Vector3f& omega_RN_BIn,
                         uint32_t observationThresholdIn,
                         float controlPeriodIn,
                         float deadbandAngleIn)
        : rotations(rotationsIn),
          sHatBdyCmd(sHatBdyCmdIn),
          sunAxisSpinRate(sunAxisSpinRateIn),
          omega_RN_B(omega_RN_BIn),
          observationThreshold(observationThresholdIn),
          controlPeriod(controlPeriodIn),
          deadbandAngle(deadbandAngleIn) {}

    std::array<RotationProperties, kNumSunSearchRotations> rotations;
    Eigen::Vector3f sHatBdyCmd;
    float sunAxisSpinRate;
    Eigen::Vector3f omega_RN_B;
    uint32_t observationThreshold;
    float controlPeriod;
    float deadbandAngle;
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

    SunSearchPointOutput update(const Eigen::Vector3f& rHat_SB_B,
                                const Eigen::Vector3f& omega_BN_B,
                                uint32_t numCssViewingSun);

    void setConfig(const SunSearchPointConfig& config);
    void reInitialize();

   private:
    enum class Phase { Searching, Pointing };

    SunSearchPointOutput computeSearch(const Eigen::Vector3f& omega_BN_B) const;
    SunSearchPointOutput computePointing(const Eigen::Vector3f& rHat_SB_B, const Eigen::Vector3f& omega_BN_B) const;
    void precomputeEndTimes();

    SunSearchPointConfig cfg;  //!< validated configuration (rotations + pointing params)
    std::array<uint64_t, kNumSunSearchRotations> rotationEndTimes{};  //!< [ns] cumulative end time of each rotation
    uint64_t controlPeriodNs{};                                       //!< [ns] elapsed time added per update() call
    uint64_t elapsedTimeNs{};       //!< [ns] time elapsed in the search sequence so far
    Phase phase{Phase::Searching};  //!< [-] current guidance phase (Pointing is terminal)
    bool searchFailed{false};       //!< [-] latched true if the sequence elapsed without acquiring the sun
};

#endif
