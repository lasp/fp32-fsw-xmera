#include "torqueDutyCycleAlgorithm.h"

/*! @brief Construct the gate with a validated configuration. The cycle starts at its on window. */
TorqueDutyCycleAlgorithm::TorqueDutyCycleAlgorithm(const TorqueDutyCycleConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the stored configuration at runtime. The cadence counter is preserved.
 @param config The validated configuration to install
 */
void TorqueDutyCycleAlgorithm::setConfig(const TorqueDutyCycleConfig& config) {
    this->cfg = config;
    /*! - the sum is at least one and cannot wrap, since the configuration requires at least one on period
     and a full cycle length that fits in a uint32_t */
    this->cycleLength = config.getOnPeriods() + config.getOffPeriods();
}

/*! Restart the duty cycle, so the next update falls at the start of an on window.
 @return void
 */
void TorqueDutyCycleAlgorithm::reInitialize() {
    /*! - sit on the final position of the cycle, so the next update advances onto position zero, where the
     on window starts */
    this->previousPositionInCycle = this->cycleLength - 1U;
}

/*! This method gates the commanded torque on and off in a fixed duty cycle. The torque is passed through
 unchanged during the on window and replaced by zero during the off window. The cadence is free-running,
 so the counter advances whether or not any torque is commanded.
 @return [Nm] the commanded body torque while on, zero while off
 @param cmdTorque_B [Nm] The commanded body torque
 */
Eigen::Vector3f TorqueDutyCycleAlgorithm::update(const Eigen::Vector3f& cmdTorque_B) {
    /*! - advance one position, wrapping at the end of the cycle; the wrap also puts the position back in range
     when setConfig() has shortened the cycle below the position already reached */
    const uint32_t positionInCycle = (this->previousPositionInCycle + 1U) % this->cycleLength;

    /*! - the on window occupies the leading positions of the cycle; the rest commands zero torque */
    Eigen::Vector3f torqueOut_B = Eigen::Vector3f::Zero();
    if (positionInCycle < this->cfg.getOnPeriods()) {
        torqueOut_B = cmdTorque_B;
    }

    this->previousPositionInCycle = positionInCycle;

    return torqueOut_B;
}
