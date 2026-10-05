#include "vectorDutyCycleAlgorithm.h"

/*! @brief Construct the gate with a validated configuration. The cycle starts at its on window. */
VectorDutyCycleAlgorithm::VectorDutyCycleAlgorithm(const VectorDutyCycleConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the stored configuration at runtime. The cadence counter is preserved.
 @param config The validated configuration to install
 */
void VectorDutyCycleAlgorithm::setConfig(const VectorDutyCycleConfig& config) {
    this->cfg = config;
    /*! - the sum is at least one and cannot wrap, since the configuration requires at least one on period
     and a full cycle length that fits in a uint32_t */
    this->cycleLength = config.getOnPeriods() + config.getOffPeriods();
}

/*! Restart the duty cycle, so the next update falls at the start of a on window.
 @return void
 */
void VectorDutyCycleAlgorithm::reInitialize() {
    /*! - sit on the final position of the cycle, so the next update advances onto position zero, where the
     on window starts */
    this->previousPositionInCycle = this->cycleLength - 1U;
}

/*! This method gates the input vector on and off in a fixed duty cycle. The vector is passed through unchanged
 during the on window and replaced by zero during the off window. The cadence is free-running, so the
 counter advances regardless of the input.
 @return the input vector while on, zero while off
 @param inputVector The vector to gate
 */
Eigen::Vector3f VectorDutyCycleAlgorithm::update(const Eigen::Vector3f& inputVector) {
    /*! - advance one position, wrapping at the end of the cycle; the wrap also puts the position back in range
     when setConfig() has shortened the cycle below the position already reached */
    const uint32_t positionInCycle = (this->previousPositionInCycle + 1U) % this->cycleLength;

    /*! - the on window occupies the leading positions of the cycle; the rest outputs a zero vector */
    Eigen::Vector3f outputVector = Eigen::Vector3f::Zero();
    if (positionInCycle < this->cfg.getOnPeriods()) {
        outputVector = inputVector;
    }

    this->previousPositionInCycle = positionInCycle;

    return outputVector;
}
