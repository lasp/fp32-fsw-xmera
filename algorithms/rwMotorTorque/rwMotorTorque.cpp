/*
 Mapping required attitude control torque Lr to RW motor torques

 */

#include "rwMotorTorque.h"
#include <architecture/utilities/eigenSupport.h>

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.rwParamsInMsg wasn't connected.");
    }
    if (!this->vehControlInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.vehControlInMsg wasn't connected.");
    }

    /*! - Read static RW config data message and convert it to the algorithm's own types */
    const RWArrayConfigMsgF32Payload rwParams = this->rwParamsInMsg();
    RwMotorTorqueArrayConfig rwConfig{};
    rwConfig.numRW = static_cast<uint32_t>(rwParams.numRW);
    rwConfig.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(rwParams.GsMatrix_B);

    /*! - Check if RW availability message is available. If so, copy the payload availability flags. */
    const bool rwAvailIsLinked = this->rwAvailInMsg.isLinked();
    RwMotorTorqueAvailability availability{};
    if (rwAvailIsLinked) {
        const RWAvailabilityMsgPayload wheelsAvailability = this->rwAvailInMsg();
        for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
            availability.wheelAvailability[i] = wheelsAvailability.wheelAvailability[i];
        }
    }

    this->algorithm.configure(rwConfig, availability, rwAvailIsLinked);
}

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::updateState(uint64_t callTime) {
    /*! - Read the commanded control torque and, if linked, the optional second torque message */
    auto [torqueRequestBody] = this->vehControlInMsg();
    Eigen::Vector3f Lr_B = cArrayToEigenVector(torqueRequestBody);
    if (this->vehControlIn2Msg.isLinked()) {
        auto [torqueRequestBody2] = this->vehControlIn2Msg();
        Lr_B += cArrayToEigenVector(torqueRequestBody2);
    }

    const Eigen::Vector<float, kMaxNumRw> motorTorque = this->algorithm.update(Lr_B);

    RwMotorTorqueMsgF32Payload rwMotorTorques{};
    eigenVectorToCArray(motorTorque, rwMotorTorques.motorTorque);
    this->rwMotorTorqueOutMsg.write(&rwMotorTorques, this->moduleID, callTime);
}

/*! Setter method for the control axes mapping matrix CB, where each row includes the transpose of a control axis.
 The matrix needs to be 3x3, so if only 2 axes are controlled, the third row should be all zeros.
 @return void
 @param controlMappingMatrix Control axes mapping matrix, each row holding the transpose of a control axis
*/
void RwMotorTorque::setControlAxes(const Eigen::Matrix3f& controlMappingMatrix) {
    this->algorithm.setControlAxes(controlMappingMatrix);
}

/*! Getter method for the control axes mapping matrix CB.
 @return const Eigen::Matrix3f
*/
Eigen::Matrix3f RwMotorTorque::getControlAxes() const { return this->algorithm.getControlAxes(); }
