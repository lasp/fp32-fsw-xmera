#ifndef F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_H
#define F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"

#include <stdint.h>
#include <Eigen/Core>

/*!
 * @brief Validated configuration for the vector duty-cycle gate.
 *
 * The full cycle length of an instance is at least one and not more than UINT32_MAX. Zero on periods are permitted
 * with one or more off periods, and keep the output vector at zero. Use VectorDutyCycleConfig::create(...) to make an
 * instance.
 */
class VectorDutyCycleConfig final {
   public:
    static VectorDutyCycleConfig create(uint32_t onPeriods, uint32_t offPeriods) {
        if (!isValidOffPeriods(offPeriods, onPeriods)) {
            FSW_THROW_INVALID_ARGUMENT(
                "vectorDutyCycle: onPeriods + offPeriods must be at least 1 and must not exceed the uint32_t maximum.");
        }
        return {onPeriods, offPeriods};
    }

    /*! All off-period values are permitted, zero included, if the full cycle length is at least one and not more
     than UINT32_MAX. Thus, zero on periods need one or more off periods. A cycle length of zero has no position, and
     an overflow of the cycle length gives a cycle that is shorter than the on window. The check adds the two values
     in a uint64_t, so the check itself cannot overflow. */
    static bool isValidOffPeriods(uint32_t offPeriods, uint32_t onPeriods) {
        const uint64_t cycleLength = static_cast<uint64_t>(onPeriods) + static_cast<uint64_t>(offPeriods);
        return cycleLength >= 1U && cycleLength <= UINT32_MAX;
    }

    /*! @return [-] number of control periods at the start of each cycle in which the output vector is equal to the
     input vector */
    uint32_t getOnPeriods() const { return this->onPeriods; }

    /*! @return [-] number of control periods in which the gate sets the output vector to zero */
    uint32_t getOffPeriods() const { return this->offPeriods; }

   private:
    // Both counts are uint32_t control periods, so a caller can easily interchange them. create() is the only
    // caller. It validates each value by name and then gives the values in declaration order.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    VectorDutyCycleConfig(uint32_t onPeriods, uint32_t offPeriods) : onPeriods(onPeriods), offPeriods(offPeriods) {}

    uint32_t onPeriods;   //!< [-] control periods in which the output vector is equal to the input vector
    uint32_t offPeriods;  //!< [-] control periods in which the output vector is zero
};

/*!
 * @brief Applies a fixed duty cycle to a vector.
 *
 * In the first onPeriods control periods of each cycle, the output vector is equal to the input vector. In the
 * remaining offPeriods control periods, the output vector is zero. The gate makes no assumption about the physical
 * quantity of the vector. The cadence is continuous. The position in the cycle moves forward at each update,
 * independent of the input vector. Thus, the on windows stay at a fixed phase.
 *
 * The position in the cycle is the only runtime state of the algorithm. reInitialize() starts the cycle again at
 * its on window.
 */
class VectorDutyCycleAlgorithm final {
   public:
    explicit VectorDutyCycleAlgorithm(const VectorDutyCycleConfig& config);

    //! Stores the validated configuration and calculates the cycle length. The runtime state does not change.
    void setConfig(const VectorDutyCycleConfig& config);

    //! Starts the duty cycle again at the start of its on window.
    void reInitialize();

    //! The input vector in an on period, zero in an off period.
    Eigen::Vector3f update(const Eigen::Vector3f& inputVector);

   private:
    VectorDutyCycleConfig cfg;           //!< [-] validated configuration (duty-cycle cadence)
    uint32_t cycleLength{};              //!< [-] control periods in one full duty cycle
    uint32_t previousPositionInCycle{};  //!< [-] position in the duty cycle at the previous update
};

#endif
