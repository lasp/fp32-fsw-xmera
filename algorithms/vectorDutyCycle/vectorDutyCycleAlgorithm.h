#ifndef F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_H
#define F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"

#include <stdint.h>
#include <Eigen/Core>

/*!
 * @brief Validated configuration for the vector duty-cycle gate.
 *
 * An instance can only exist if the cycle is on for at least one control period and the full cycle length
 * remains representable in a uint32_t. Construct via VectorDutyCycleConfig::create(...).
 */
class VectorDutyCycleConfig final {
   public:
    static VectorDutyCycleConfig create(uint32_t onPeriods, uint32_t offPeriods) {
        if (!isValidOnPeriods(onPeriods)) {
            FSW_THROW_INVALID_ARGUMENT("vectorDutyCycle: onPeriods must be >= 1.");
        }
        if (!isValidOffPeriods(offPeriods, onPeriods)) {
            FSW_THROW_INVALID_ARGUMENT("vectorDutyCycle: onPeriods + offPeriods must not exceed the uint32_t maximum.");
        }
        return {onPeriods, offPeriods};
    }

    /*! A cycle with no on period would hold the vector at zero forever, so at least one is required. */
    static bool isValidOnPeriods(uint32_t onPeriods) { return onPeriods >= 1U; }

    /*! Any hold-off length is admissible, including none, provided the full cycle length does not wrap around:
     a wrapped length would come out shorter than the on window and corrupt the cadence. The sum is taken
     in a wider type, so the check itself cannot wrap. */
    static bool isValidOffPeriods(uint32_t offPeriods, uint32_t onPeriods) {
        return static_cast<uint64_t>(onPeriods) + static_cast<uint64_t>(offPeriods) <= UINT32_MAX;
    }

    /*! @return [-] control periods, at the start of each cycle, for which the input vector is passed through. */
    uint32_t getOnPeriods() const { return this->onPeriods; }

    /*! @return [-] control periods for which the gate outputs a zero vector. */
    uint32_t getOffPeriods() const { return this->offPeriods; }

   private:
    // Both counts are uint32_t control periods, so they read as swappable. create() is the only caller and
    // validates each by name before forwarding them in declaration order.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    VectorDutyCycleConfig(uint32_t onPeriods, uint32_t offPeriods) : onPeriods(onPeriods), offPeriods(offPeriods) {}

    uint32_t onPeriods;   //!< [-] control periods spent passing the input vector through
    uint32_t offPeriods;  //!< [-] control periods spent outputting a zero vector
};

/*!
 * @brief Gates a vector on and off in a fixed duty cycle.
 *
 * The gate passes the input vector through unchanged for the first onPeriods control periods of every cycle
 * and outputs a zero vector for the remaining offPeriods. The gate makes no assumption on what the vector
 * describes. The cadence is free-running: the counter advances on every update regardless of the input, so the
 * on windows sit at a fixed phase.
 *
 * The position in the cycle is the algorithm's only runtime state; reInitialize() restarts the cycle at its
 * on window.
 */
class VectorDutyCycleAlgorithm final {
   public:
    explicit VectorDutyCycleAlgorithm(const VectorDutyCycleConfig& config);

    //! Install the validated configuration and derive the cycle length; does not touch runtime state.
    void setConfig(const VectorDutyCycleConfig& config);

    //! Restart the duty cycle at the beginning of its on window.
    void reInitialize();

    //! The input vector during an on period, zero during an off period.
    Eigen::Vector3f update(const Eigen::Vector3f& inputVector);

   private:
    VectorDutyCycleConfig cfg;           //!< [-] validated configuration (duty-cycle cadence)
    uint32_t cycleLength{};              //!< [-] control periods in one full duty cycle
    uint32_t previousPositionInCycle{};  //!< [-] position in the duty cycle that the previous update gated
};

#endif
