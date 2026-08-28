#ifndef F32XMERA_THR_DESAT_DUTY_CYCLE_ALGORITHM_H
#define F32XMERA_THR_DESAT_DUTY_CYCLE_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <stdint.h>
#include <array>

/*!
 * @brief Validated configuration for the thruster desaturation duty-cycle gate.
 *
 * An instance can only exist if the cycle fires for at least one control period and the full cycle length
 * remains representable in a uint32_t. Construct via ThrDesatDutyCycleConfig::create(...).
 */
class ThrDesatDutyCycleConfig final {
   public:
    static ThrDesatDutyCycleConfig create(uint32_t firingPeriods, uint32_t settlingPeriods) {
        if (!isValidFiringPeriods(firingPeriods)) {
            FSW_THROW_INVALID_ARGUMENT("thrDesatDutyCycle: firingPeriods must be >= 1.");
        }
        if (!isValidSettlingPeriods(settlingPeriods, firingPeriods)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrDesatDutyCycle: firingPeriods + settlingPeriods must not exceed the uint32_t maximum.");
        }
        return {firingPeriods, settlingPeriods};
    }

    /*! A cycle with no firing period would hold the thrusters off forever, so at least one is required. */
    static bool isValidFiringPeriods(uint32_t firingPeriods) { return firingPeriods >= 1U; }

    /*! Any hold-off length is admissible, including none, provided the full cycle length does not wrap around:
     a wrapped length would come out shorter than the firing window and corrupt the cadence. */
    static bool isValidSettlingPeriods(uint32_t settlingPeriods, uint32_t firingPeriods) {
        return settlingPeriods <= UINT32_MAX - firingPeriods;
    }

    /*! @return [-] control periods, at the start of each cycle, for which the force command is passed through. */
    uint32_t getFiringPeriods() const { return this->firingPeriods; }

    /*! @return [-] control periods for which the gate commands zero force, letting the RWs re-settle. */
    uint32_t getSettlingPeriods() const { return this->settlingPeriods; }

    /*! @return [-] length of one full duty cycle, whose first getFiringPeriods() slots are the firing window. */
    uint32_t getCycleLength() const { return this->firingPeriods + this->settlingPeriods; }

   private:
    // Both counts are uint32_t control periods, so they read as swappable. create() is the only caller and
    // validates each by name before forwarding them in declaration order.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    ThrDesatDutyCycleConfig(uint32_t firingPeriods, uint32_t settlingPeriods)
        : firingPeriods(firingPeriods), settlingPeriods(settlingPeriods) {}

    uint32_t firingPeriods;    //!< [-] control periods spent passing the force command through
    uint32_t settlingPeriods;  //!< [-] control periods spent commanding zero force
};

/*!
 * @brief Gates a thruster force command on and off in a fixed duty cycle.
 *
 * The gate passes the commanded force through unchanged for the first firingPeriods control periods of every
 * cycle and commands zero force for the remaining settlingPeriods, giving the reaction wheels quiet windows in
 * which to re-stabilize the attitude between desaturation pulses. The cadence is free-running: the counter
 * advances on every update regardless of what is commanded, so the firing windows sit at a fixed phase.
 *
 * The counter is the algorithm's only runtime state; reInitialize() restarts the cycle at its firing window.
 */
class ThrDesatDutyCycleAlgorithm final {
   public:
    explicit ThrDesatDutyCycleAlgorithm(const ThrDesatDutyCycleConfig& config);

    //! Install the validated configuration; does not touch runtime state.
    void setConfig(const ThrDesatDutyCycleConfig& config);

    //! Restart the duty cycle at the beginning of its firing window.
    void reInitialize();

    //! [N] The per-thruster commanded force during a firing period, zero during a settling period.
    std::array<float, kMaxThrusterCount> update(const std::array<float, kMaxThrusterCount>& thrusterForceCmd);

   private:
    ThrDesatDutyCycleConfig cfg;  //!< [-] validated configuration (duty-cycle cadence)
    uint32_t phaseCounter{};      //!< [-] control periods elapsed since the start of the current duty cycle
};

#endif
