#include "torqueDutyCycle.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

/*! This method performs a complete reset of the module. It validates that the required input message is linked
 and builds the algorithm, whose constructor installs the configuration and restarts the duty cycle.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TorqueDutyCycle::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->cmdTorqueInMsg.isLinked()) {
        throw std::invalid_argument("torqueDutyCycle.cmdTorqueInMsg wasn't connected.");
    }

    /*! - create the algorithm, whose constructor installs the configuration and restarts the duty cycle
     (throws on an invalid config) */
    this->algorithm = std::make_unique<TorqueDutyCycleAlgorithm>(this->toConfig());
}

/*! Build a validated algorithm configuration from the current module properties. The whole configuration is
 held in module properties, so no input message is read here.
 @return TorqueDutyCycleConfig validated configuration
 */
TorqueDutyCycleConfig TorqueDutyCycle::toConfig() const {
    return TorqueDutyCycleConfig::create(this->firingPeriods, this->settlingPeriods);
}

/*! Re-validate the current module properties and push them onto the live algorithm without restarting the
 cadence. Rebuilds the validated config from the public members and installs it via setConfig().
 @return void
 */
void TorqueDutyCycle::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TorqueDutyCycle reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Restart the duty cycle at the beginning of its firing window; a simple pass-through to the algorithm's
 reInitialize().
 @return void
 */
void TorqueDutyCycle::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TorqueDutyCycle reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! The commanded torque is gated on and off in a fixed duty cycle.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TorqueDutyCycle::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TorqueDutyCycle reset() has not been called.");
    }

    /*! - read in the torque command message and map to the freestanding type */
    const CmdTorqueBodyMsgF32Payload cmdTorqueIn = this->cmdTorqueInMsg();
    const Eigen::Vector3f cmdTorque_B = cArrayToEigenVector3<float>(cmdTorqueIn.torqueRequestBody);

    /*! - call algorithm update */
    const Eigen::Vector3f gatedTorque_B = this->algorithm->update(cmdTorque_B);

    /*! - map the freestanding type back to the message payload and write */
    CmdTorqueBodyMsgF32Payload cmdTorqueOut{};
    eigenVectorToCArray(gatedTorque_B, cmdTorqueOut.torqueRequestBody);

    this->cmdTorqueOutMsg.write(cmdTorqueOut, this->moduleID, callTime);
}
