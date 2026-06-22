#ifndef THR_FIRING_REMAINDER_ALGORITHM
#define THR_FIRING_REMAINDER_ALGORITHM

#include "msgPayloadDef/definitions.h"
#include "thrFiringRemainderTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <stdint.h>
#include <array>
#include <cstdint>

enum class ThrustPulsingRegime : std::uint8_t { ON_PULSING = 0U, OFF_PULSING = 1U };

/*! @brief Single thruster configuration */
struct ThrusterConfig {
    std::array<float, 3> rThrust_B{};     //!< [m] Location of the thruster in the spacecraft
    std::array<float, 3> tHatThrust_B{};  //!< [-] Unit vector of the thrust direction
    float maxThrust{};                    //!< [N] Max thrust
};

/*! @brief Thruster array configuration */
struct ThrusterArrayConfig {
    std::uint32_t numThrusters{};                               //!< [-] number of thrusters
    std::array<ThrusterConfig, kMaxThrusterCount> thrusters{};  //!< [-] array of thruster configuration information
};

/*! @brief Thruster force command input */
struct ThrusterForceCmd {
    std::array<float, kMaxThrusterCount> thrForce{};  //!< [N] array of thruster force values
};

/*! @brief Thruster on-time command output */
struct ThrusterOnTimeCmd {
    std::array<float, kMaxThrusterCount> onTimeRequest{};  //!< [s] array of on-time requests
};

/*! @brief Validated thruster array: per-thruster maximum thrust. */
struct ThrFiringRemainderThrusterArray {
    uint32_t numThrusters{};                           //!< [-] number of thrusters on the vehicle
    std::array<float, kMaxThrusterCount> maxThrust{};  //!< [N] per-thruster maximum thrust
};

/*! @brief Firing control parameters governing the remainder-tracking pulse logic. */
struct ThrFiringControlParameters {
    float thrMinFireTime{};               //!< [s] minimum commandable thruster fire time
    float controlPeriod{};                //!< [s] control period over which the force command applies
    float onTimeSaturationFactor{1.0F};   //!< [-] control-period multiplier applied when on-time saturates
    ThrustPulsingRegime pulsingRegime{};  //!< [-] on-pulsing or off-pulsing
};

/*!
 * @brief Validated configuration for the thruster firing remainder algorithm.
 *
 * Bundles the thruster array (per-thruster maximum thrust) with the firing control parameters. An instance can
 * only exist if: the thruster count does not exceed the compile-time maximum and every maximum thrust is finite
 * and non-negative; the minimum fire time is finite and non-negative; the control period is finite and positive;
 * the on-time saturation factor is finite and at least one; and the pulsing regime is a defined enumerator.
 * Construct via ThrFiringRemainderConfig::create(...).
 */
class ThrFiringRemainderConfig final {
   public:
    static ThrFiringRemainderConfig create(const ThrFiringRemainderThrusterArray& thrusterArray,
                                           const ThrFiringControlParameters& controlParameters) {
        if (!isValidThrusterArray(thrusterArray)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrFiringRemainder: numThrusters must not exceed the compile-time maximum and each thruster's "
                "maxThrust must be finite and >= 0.");
        }
        if (!isValidThrMinFireTime(controlParameters.thrMinFireTime)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringRemainder: thrMinFireTime must be finite and >= 0.");
        }
        if (!isValidControlPeriod(controlParameters.controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringRemainder: controlPeriod must be finite and > 0.");
        }
        if (!isValidOnTimeSaturationFactor(controlParameters.onTimeSaturationFactor)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringRemainder: onTimeSaturationFactor must be finite and >= 1.");
        }
        if (!isValidPulsingRegime(controlParameters.pulsingRegime)) {
            FSW_THROW_INVALID_ARGUMENT("thrFiringRemainder: pulsingRegime must be ON_PULSING or OFF_PULSING.");
        }
        return {thrusterArray, controlParameters};
    }

    static bool isValidThrusterArray(const ThrFiringRemainderThrusterArray& thrusterArray) {
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
    static bool isValidThrMinFireTime(float thrMinFireTime) {
        return fsw::is_finite(thrMinFireTime) && thrMinFireTime >= 0.0F;
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

    const ThrFiringRemainderThrusterArray& getThrusterArray() const { return thrusterArray; }
    const ThrFiringControlParameters& getControlParameters() const { return controlParameters; }

   private:
    ThrFiringRemainderConfig(const ThrFiringRemainderThrusterArray& thrusterArray,
                             const ThrFiringControlParameters& controlParameters)
        : thrusterArray(thrusterArray), controlParameters(controlParameters) {}

    ThrFiringRemainderThrusterArray thrusterArray;
    ThrFiringControlParameters controlParameters;
};

class ThrFiringRemainderAlgorithm {
   public:
    void reset();
    ThrusterOnTimeCmd update(ThrusterForceCmd thrusterForceCmd);

    void setThrusters(const ThrusterArrayConfig& thrusterConfig);

    void setThrMinFireTime(float minFireTime);
    float getThrMinFireTime() const;

    void setThrustPulsingRegime(ThrustPulsingRegime pulsingRegime);
    ThrustPulsingRegime getThrustPulsingRegime() const;

    void setControlPeriod(float period);
    float getControlPeriod() const;

    void setOnTimeSaturationFactor(float factor);
    float getOnTimeSaturationFactor() const;

   private:
    std::array<float, kMaxThrusterCount>
        pulseRemainder{};                              //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    float thrMinFireTime{};                            //!< [s] Minimum fire time
    uint32_t numThrusters{};                           //!< [-] The number of thrusters available on vehicle
    std::array<float, kMaxThrusterCount> maxThrust{};  //!< [N] Max thrust
    ThrustPulsingRegime thrustPulsingRegime{};         //!< [-] Indicates on-pulsing or off-pulsing
    float controlPeriod{};                             //!< [s] Default control period used for first call
    float onTimeSaturationFactor{1.0};  //!< [-] Factor to multiply the control period by when ontime is saturated
};

#endif
