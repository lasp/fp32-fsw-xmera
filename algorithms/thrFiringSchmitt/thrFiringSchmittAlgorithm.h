#ifndef F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H
#define F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include <stdint.h>
#include <array>
#include <cstdint>

enum class ThrustPulsingRegime : std::uint8_t { ON_PULSING = 0U, OFF_PULSING = 1U };

/*! @brief Thruster force command input */
struct ThrusterForceCmd {
    std::array<float, kMaxThrusterCount> thrForce{};  //!< [N] array of thruster force values
};

/*! @brief Thruster on-time command output */
struct ThrusterOnTimeCmd {
    std::array<float, kMaxThrusterCount> onTimeRequest{};  //!< [s] array of on-time requests
};

/*! @brief Validated thruster array: per-thruster maximum thrust. */
struct ThrFiringSchmittThrusterArray {
    uint32_t numThrusters{};                           //!< [-] number of thrusters on the vehicle
    std::array<float, kMaxThrusterCount> maxThrust{};  //!< [N] per-thruster maximum thrust
};

/*! @brief Firing control parameters governing the Schmitt-trigger pulse logic. */
struct ThrFiringSchmittControlParameters {
    float levelOn{};                      //!< [-] ON duty cycle fraction threshold, in (0, 1]
    float levelOff{};                     //!< [-] OFF duty cycle fraction threshold, in [0, 1)
    float thrMinFireTime{};               //!< [s] minimum commandable thruster fire time
    float controlPeriod{};                //!< [s] control period over which the force command applies
    float onTimeSaturationFactor{1.0F};   //!< [-] control-period multiplier applied when on-time saturates
    ThrustPulsingRegime pulsingRegime{};  //!< [-] on-pulsing or off-pulsing
};

/*!
 * @brief Validated configuration for the thruster firing Schmitt algorithm.
 *
 * Bundles the thruster array (per-thruster maximum thrust) with the Schmitt-trigger firing control parameters.
 * An instance can only exist if: the thruster count does not exceed the compile-time maximum and every maximum
 * thrust is finite and non-negative; the ON/OFF duty cycle thresholds are finite with 0 < levelOn <= 1,
 * 0 <= levelOff < 1, and levelOn >= levelOff; the minimum fire time is finite and positive; the control period is
 * finite and positive; the on-time saturation factor is finite and at least one; and the pulsing regime is a
 * defined enumerator. Construct via ThrFiringSchmittConfig::create(...).
 */
class ThrFiringSchmittConfig final {
   public:
    static ThrFiringSchmittConfig create(const ThrFiringSchmittThrusterArray& thrusterArray,
                                         const ThrFiringSchmittControlParameters& controlParameters) {
        if (!isValidThrusterArray(thrusterArray)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrFiringSchmitt: numThrusters must not exceed the compile-time maximum and each thruster's "
                "maxThrust must be finite and >= 0.");
        }
        if (!isValidLevels(controlParameters.levelOn, controlParameters.levelOff)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrFiringSchmitt: levels must be finite with 0 < levelOn <= 1, 0 <= levelOff < 1, and "
                "levelOn >= levelOff.");
        }
        if (!isValidThrMinFireTime(controlParameters.thrMinFireTime)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringSchmitt: thrMinFireTime must be finite and > 0.");
        }
        if (!isValidControlPeriod(controlParameters.controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringSchmitt: controlPeriod must be finite and > 0.");
        }
        if (!isValidOnTimeSaturationFactor(controlParameters.onTimeSaturationFactor)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringSchmitt: onTimeSaturationFactor must be finite and >= 1.");
        }
        if (!isValidPulsingRegime(controlParameters.pulsingRegime)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringSchmitt: pulsingRegime must be ON_PULSING or OFF_PULSING.");
        }
        return {thrusterArray, controlParameters};
    }

    static bool isValidThrusterArray(const ThrFiringSchmittThrusterArray& thrusterArray) {
        if (thrusterArray.numThrusters > kMaxThrusterCount) {
            return false;
        }
        for (uint32_t i = 0U; i < thrusterArray.numThrusters; ++i) {
            if (!fsw::is_finite(thrusterArray.maxThrust.at(i)) || thrusterArray.maxThrust.at(i) < 0.0F) {
                return false;
            }
        }
        return true;
    }
    static bool isValidLevels(float levelOn, float levelOff) {
        return fsw::is_finite(levelOn) && fsw::is_finite(levelOff) && levelOn > 0.0F && levelOn <= 1.0F &&
               levelOff >= 0.0F && levelOff < 1.0F && levelOn >= levelOff;
    }
    static bool isValidThrMinFireTime(float thrMinFireTime) {
        return fsw::is_finite(thrMinFireTime) && thrMinFireTime > 0.0F;
    }
    static bool isValidControlPeriod(float controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0F;
    }
    static bool isValidOnTimeSaturationFactor(float onTimeSaturationFactor) {
        return fsw::is_finite(onTimeSaturationFactor) && onTimeSaturationFactor >= 1.0F;
    }
    static bool isValidPulsingRegime(ThrustPulsingRegime pulsingRegime) {
        return pulsingRegime == ThrustPulsingRegime::ON_PULSING || pulsingRegime == ThrustPulsingRegime::OFF_PULSING;
    }

    const ThrFiringSchmittThrusterArray& getThrusterArray() const { return thrusterArray; }
    const ThrFiringSchmittControlParameters& getControlParameters() const { return controlParameters; }

   private:
    ThrFiringSchmittConfig(const ThrFiringSchmittThrusterArray& thrusterArray,
                           const ThrFiringSchmittControlParameters& controlParameters)
        : thrusterArray(thrusterArray), controlParameters(controlParameters) {}

    ThrFiringSchmittThrusterArray thrusterArray;
    ThrFiringSchmittControlParameters controlParameters;
};

enum class ThrusterState { OFF = 0, ON = 1 };

class ThrFiringSchmittAlgorithm final {
   public:
    explicit ThrFiringSchmittAlgorithm(const ThrFiringSchmittConfig& config);
    void setConfig(const ThrFiringSchmittConfig& config);
    void reInitialize();
    ThrusterOnTimeCmd update(ThrusterForceCmd thrusterForceCmd);

   private:
    ThrFiringSchmittConfig cfg;
    std::array<ThrusterState, kMaxThrusterCount>
        prevThrustState{};  //!< [-] ON/OFF state of thrusters from previous call
};

#endif
