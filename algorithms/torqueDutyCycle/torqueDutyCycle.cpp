#include "torqueDutyCycle.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

/*! This method does a full reset of the module. It makes sure that the necessary input message is connected. It
 then makes the algorithm. The constructor of the algorithm stores the configuration and starts the duty cycle.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TorqueDutyCycle::reset(const uint64_t callTime) {
    // make sure that the necessary input message is connected
    if (!this->cmdTorqueInMsg.isLinked()) {
        throw std::invalid_argument("torqueDutyCycle.cmdTorqueInMsg wasn't connected.");
    }

    /*! - make the algorithm. Its constructor stores the configuration and starts the duty cycle. An invalid
     configuration causes an exception. */
    this->algorithm = std::make_unique<TorqueDutyCycleAlgorithm>(this->toConfig());
}

/*! Makes a validated algorithm configuration from the current module properties. All of the configuration is in
 the module properties. Thus, this function does not read an input message.
 @return TorqueDutyCycleConfig validated configuration
 */
TorqueDutyCycleConfig TorqueDutyCycle::toConfig() const {
    return TorqueDutyCycleConfig::create(this->onPeriods, this->offPeriods);
}

/*! Validates the current module properties again and gives them to the algorithm. The position in the cycle does
 not change. This function makes a validated configuration from the public members and stores it with setConfig().
 @return void
 */
void TorqueDutyCycle::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TorqueDutyCycle reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Starts the duty cycle again at the start of its on window. This function calls the reInitialize() of the
 algorithm.
 @return void
 */
void TorqueDutyCycle::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TorqueDutyCycle reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method applies a fixed duty cycle to the commanded torque.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TorqueDutyCycle::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TorqueDutyCycle reset() has not been called.");
    }

    /*! - read the torque command message and convert it to the freestanding type */
    const CmdTorqueBodyMsgF32Payload cmdTorqueIn = this->cmdTorqueInMsg();
    const Eigen::Vector3f cmdTorque_B = cArrayToEigenVector3<float>(cmdTorqueIn.torqueRequestBody);

    /*! - call the algorithm update */
    const Eigen::Vector3f gatedTorque_B = this->algorithm->update(cmdTorque_B);

    /*! - convert the freestanding type to the message payload and write the message */
    CmdTorqueBodyMsgF32Payload cmdTorqueOut{};
    eigenVectorToCArray(gatedTorque_B, cmdTorqueOut.torqueRequestBody);

    this->cmdTorqueOutMsg.write(cmdTorqueOut, this->moduleID, callTime);
}
