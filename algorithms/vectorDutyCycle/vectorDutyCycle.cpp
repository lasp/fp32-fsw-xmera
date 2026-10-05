#include "vectorDutyCycle.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

/*! This method does a full reset of the module. It makes sure that the input message of the selected vector type
 is connected. It then makes the algorithm and keeps the value of vectorType until the next reset. The constructor
 of the algorithm stores the configuration and starts the duty cycle.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::reset(const uint64_t callTime) {
    // make sure that the input message of the selected vector type is connected
    switch (this->vectorType) {
        case VectorType::Force:
            if (!this->cmdForceInMsg.isLinked()) {
                throw std::invalid_argument("vectorDutyCycle.cmdForceInMsg wasn't connected.");
            }
            break;
        case VectorType::Torque:
            if (!this->cmdTorqueInMsg.isLinked()) {
                throw std::invalid_argument("vectorDutyCycle.cmdTorqueInMsg wasn't connected.");
            }
            break;
        default:
            throw std::invalid_argument("vectorDutyCycle.vectorType is not a valid VectorType.");
    }

    /*! - make the algorithm. Its constructor stores the configuration and starts the duty cycle. An invalid
     configuration causes an exception. */
    this->algorithm = std::make_unique<VectorDutyCycleAlgorithm>(this->toConfig());
    this->activeVectorType = this->vectorType;
}

/*! Makes a validated algorithm configuration from the current module properties. All of the configuration is in
 the module properties. Thus, this function does not read an input message.
 @return VectorDutyCycleConfig validated configuration
 */
VectorDutyCycleConfig VectorDutyCycle::toConfig() const {
    return VectorDutyCycleConfig::create(this->onPeriods, this->offPeriods);
}

/*! Validates the current module properties again and gives them to the algorithm. The position in the cycle does
 not change. This function makes a validated configuration from the public members and stores it with setConfig().
 @return void
 */
void VectorDutyCycle::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("VectorDutyCycle reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Starts the duty cycle again at the start of its on window. This function calls the reInitialize() of the
 algorithm.
 @return void
 */
void VectorDutyCycle::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("VectorDutyCycle reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method applies a fixed duty cycle to the command of the vector type that reset() selected.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("VectorDutyCycle reset() has not been called.");
    }

    if (this->activeVectorType == VectorType::Force) {
        this->updateForce(callTime);
    } else {
        this->updateTorque(callTime);
    }
}

/*! Applies the duty cycle to the commanded force and writes the force output message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::updateForce(const uint64_t callTime) {
    const CmdForceBodyMsgF32Payload cmdForceIn = this->cmdForceInMsg();
    const Eigen::Vector3f cmdForce_B = cArrayToEigenVector3<float>(cmdForceIn.forceRequestBody);

    const Eigen::Vector3f gatedForce_B = this->algorithm->update(cmdForce_B);

    CmdForceBodyMsgF32Payload cmdForceOut{};
    eigenVectorToCArray(gatedForce_B, cmdForceOut.forceRequestBody);

    this->cmdForceOutMsg.write(cmdForceOut, this->moduleID, callTime);
}

/*! Applies the duty cycle to the commanded torque and writes the torque output message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void VectorDutyCycle::updateTorque(const uint64_t callTime) {
    const CmdTorqueBodyMsgF32Payload cmdTorqueIn = this->cmdTorqueInMsg();
    const Eigen::Vector3f cmdTorque_B = cArrayToEigenVector3<float>(cmdTorqueIn.torqueRequestBody);

    const Eigen::Vector3f gatedTorque_B = this->algorithm->update(cmdTorque_B);

    CmdTorqueBodyMsgF32Payload cmdTorqueOut{};
    eigenVectorToCArray(gatedTorque_B, cmdTorqueOut.torqueRequestBody);

    this->cmdTorqueOutMsg.write(cmdTorqueOut, this->moduleID, callTime);
}
