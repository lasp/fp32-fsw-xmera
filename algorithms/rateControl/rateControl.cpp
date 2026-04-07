/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "rateControl.h"
#include "architecture/utilities/eigenSupport.h"
#include <stdint.h>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @param callTime [ns] Time the method is called
*/
void RateControl::reset(uint64_t callTime) {
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("rateControl.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("rateControl.vehConfigInMsg wasn't connected.");
    }

    if (this->vehConfigInMsg.isWritten()) {
        auto vehicleConfigPayload = this->vehConfigInMsg();
        const Eigen::Matrix3f spacecraftInertia = cArrayToEigenMatrix3(vehicleConfigPayload.ISCPntB_B);
        this->algorithm.setSpacecraftInertia(spacecraftInertia);
    }
}

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @param callTime [ns] Time the method is called
*/
void RateControl::updateState(uint64_t callTime) {
    CmdTorqueBodyMsgF32Payload torqueCmdOut{};
    if (this->guidInMsg.isWritten()) {
        auto inMsg = this->guidInMsg();
        const Eigen::Vector3f omega_BR_B = cArrayToEigenVector3(inMsg.omega_BR_B);
        const Eigen::Vector3f domega_RN_B = cArrayToEigenVector3(inMsg.domega_RN_B);
        Eigen::Vector3f const out = this->algorithm.update(omega_BR_B, domega_RN_B);
        eigenVectorToCArray(out, torqueCmdOut.torqueRequestBody);
    }

    this->cmdTorqueOutMsg.write(&torqueCmdOut, moduleID, callTime);
}

/*! Setter method for the derivative gain P.
 @param P the derivative gain P
*/
void RateControl::setDerivativeGainP(const float P) { this->algorithm.setDerivativeGainP(P); }

/*! Getter method for the derivative gain P.
 @return the derivative gain P
*/
float RateControl::getDerivativeGainP() const { return this->algorithm.getDerivativeGainP(); }

/*! Setter method for the known external torque.
 @param knownTorquePntB_B the known external torque [N*m] expressed in body frame coordinates
*/
void RateControl::setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B) {
    this->algorithm.setKnownTorquePntB_B(knownTorquePntB_B);
}

/*! Getter method for the known torque.
 @return the known external torque [N*m] expressed in body frame coordinates
*/
const Eigen::Vector3f& RateControl::getKnownTorquePntB_B() const { return this->algorithm.getKnownTorquePntB_B(); }
