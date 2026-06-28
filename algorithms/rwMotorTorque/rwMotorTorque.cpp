/*
 Mapping required attitude control torque Lr to RW motor torques

 */

#include "rwMotorTorque.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

// The algorithm's C-boundary RW count must match the system-wide RW_EFF_CNT, otherwise the
// payload GsMatrix_B / motorTorque arrays would not map onto the algorithm's fixed-size types.
static_assert(kMaxNumRw == RW_EFF_CNT, "RW_MOTOR_TORQUE_MAX_NUM_RW must match RW_EFF_CNT");

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.rwParamsInMsg wasn't connected.");
    }
    if (!this->vehControlInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.vehControlInMsg wasn't connected.");
    }

    /*! - Read static RW config data message and convert it to the algorithm's own types. Availability
     defaults to all wheels AVAILABLE; if the optional message is linked, copy its flags into the config. */
    const RWArrayConfigMsgF32Payload rwParams = this->rwParamsInMsg();
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = static_cast<uint32_t>(rwParams.numRW);
    rwConfiguration.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwParams.GsMatrix_B);
    if (this->rwAvailInMsg.isLinked()) {
        const RWAvailabilityMsgPayload wheelsAvailability = this->rwAvailInMsg();
        for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
            rwConfiguration.wheelAvailability[i] = wheelsAvailability.wheelAvailability[i];
        }
    }

    /*! - Build the validated configuration and (re)create the algorithm (computes the mapping and
     projection; throws on an invalid config). */
    const auto config = RwMotorTorqueConfig::create(this->controlAxes_B, rwConfiguration, this->omegaGain);
    this->algorithm = std::make_unique<RwMotorTorqueAlgorithm>(config);
}

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("RwMotorTorque reset() has not been called.");
    }

    /*! - Read the commanded control torque and, if linked, the optional second torque message */
    auto [torqueRequestBody] = this->vehControlInMsg();
    Eigen::Vector3f Lr_B = cArrayToEigenVector(torqueRequestBody);
    if (this->vehControlIn2Msg.isLinked()) {
        auto [torqueRequestBody2] = this->vehControlIn2Msg();
        Lr_B += cArrayToEigenVector(torqueRequestBody2);
    }

    /*! - Read the optional RW speeds for the null-space term; unlinked speeds default to zero. */
    RwMotorTorqueSpeeds speeds{};
    if (this->rwSpeedsInMsg.isLinked()) {
        const RWSpeedMsgF32Payload rwSpeeds = this->rwSpeedsInMsg();
        speeds.rwSpeeds = cArrayToEigenVector(rwSpeeds.wheelSpeeds);
    }
    if (this->rwDesiredSpeedsInMsg.isLinked()) {
        const RWSpeedMsgF32Payload rwDesiredSpeeds = this->rwDesiredSpeedsInMsg();
        speeds.rwDesiredSpeeds = cArrayToEigenVector(rwDesiredSpeeds.wheelSpeeds);
    }

    const Eigen::Vector<float, kMaxNumRw> motorTorque = this->algorithm->update(Lr_B, speeds);

    RwMotorTorqueMsgF32Payload rwMotorTorques{};
    eigenVectorToCArray(motorTorque, rwMotorTorques.motorTorque);
    this->rwMotorTorqueOutMsg.write(&rwMotorTorques, this->moduleID, callTime);
}
