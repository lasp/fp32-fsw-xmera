#ifndef F32XMERA_TORQUE_DUTY_CYCLE_ALGORITHM_H
#define F32XMERA_TORQUE_DUTY_CYCLE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"

#include <stdint.h>
#include <Eigen/Core>

/*!
 * @brief Validated configuration for the torque duty-cycle gate.
 *
 * An instance can only exist if the cycle is on for at least one control period and the full cycle length
 * remains representable in a uint32_t. Construct via TorqueDutyCycleConfig::create(...).
 */
class TorqueDutyCycleConfig final {
   public:
    static TorqueDutyCycleConfig create(uint32_t onPeriods, uint32_t offPeriods) {
        if (!isValidOnPeriods(onPeriods)) {
            FSW_THROW_INVALID_ARGUMENT("torqueDutyCycle: onPeriods must be >= 1.");
        }
        if (!isValidOffPeriods(offPeriods, onPeriods)) {
            FSW_THROW_INVALID_ARGUMENT("torqueDutyCycle: onPeriods + offPeriods must not exceed the uint32_t maximum.");
        }
        return {onPeriods, offPeriods};
    }

    /*! A cycle with no on period would hold the torque at zero forever, so at least one is required. */
    static bool isValidOnPeriods(uint32_t onPeriods) { return onPeriods >= 1U; }

    /*! Any hold-off length is admissible, including none, provided the full cycle length does not wrap around:
     a wrapped length would come out shorter than the on window and corrupt the cadence. The sum is taken
     in a wider type, so the check itself cannot wrap. */
    static bool isValidOffPeriods(uint32_t offPeriods, uint32_t onPeriods) {
        return static_cast<uint64_t>(onPeriods) + static_cast<uint64_t>(offPeriods) <= UINT32_MAX;
    }

    /*! @return [-] control periods, at the start of each cycle, for which the torque command is passed through. */
    uint32_t getOnPeriods() const { return this->onPeriods; }

    /*! @return [-] control periods for which the gate commands zero torque. */
    uint32_t getOffPeriods() const { return this->offPeriods; }

   private:
    // Both counts are uint32_t control periods, so they read as swappable. create() is the only caller and
    // validates each by name before forwarding them in declaration order.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    TorqueDutyCycleConfig(uint32_t onPeriods, uint32_t offPeriods) : onPeriods(onPeriods), offPeriods(offPeriods) {}

    uint32_t onPeriods;   //!< [-] control periods spent passing the torque command through
    uint32_t offPeriods;  //!< [-] control periods spent commanding zero torque
};

/*!
 * @brief Gates a torque command on and off in a fixed duty cycle.
 *
 * The gate passes the commanded body torque through unchanged for the first onPeriods control periods of
 * every cycle and commands zero torque for the remaining offPeriods. The cadence is free-running: the counter
 * advances on every update regardless of what is commanded, so the on windows sit at a fixed phase.
 *
 * The position in the cycle is the algorithm's only runtime state; reInitialize() restarts the cycle at its
 * on window.
 */
class TorqueDutyCycleAlgorithm final {
   public:
    explicit TorqueDutyCycleAlgorithm(const TorqueDutyCycleConfig& config);

    //! Install the validated configuration and derive the cycle length; does not touch runtime state.
    void setConfig(const TorqueDutyCycleConfig& config);

    //! Restart the duty cycle at the beginning of its on window.
    void reInitialize();

    //! [Nm] The commanded body torque during an on period, zero during an off period.
    Eigen::Vector3f update(const Eigen::Vector3f& cmdTorque_B);

   private:
    TorqueDutyCycleConfig cfg;           //!< [-] validated configuration (duty-cycle cadence)
    uint32_t cycleLength{};              //!< [-] control periods in one full duty cycle
    uint32_t previousPositionInCycle{};  //!< [-] position in the duty cycle that the previous update gated
};

#endif
