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
 @return void
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
        const Eigen::Matrix3f spacecraftInertia =
            Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(this->vehConfigInMsg().ISCPntB_B);
        this->algorithm.setSpacecraftInertia(spacecraftInertia);
    }
}

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime [ns] Time the method is called
*/
void RateControl::updateState(uint64_t callTime) {
    CmdTorqueBodyMsgF32Payload torqueCmdOut{};
    const auto& inMsg = this->guidInMsg();
    InputGuidanceData in{};
    in.omega_BR_B = Eigen::Vector3f(inMsg.omega_BR_B[0], inMsg.omega_BR_B[1], inMsg.omega_BR_B[2]);
    in.omega_RN_B = Eigen::Vector3f(inMsg.omega_RN_B[0], inMsg.omega_RN_B[1], inMsg.omega_RN_B[2]);
    in.domega_RN_B = Eigen::Vector3f(inMsg.domega_RN_B[0], inMsg.domega_RN_B[1], inMsg.domega_RN_B[2]);

    if (this->guidInMsg.isWritten()) {
        Eigen::Vector3f out = this->algorithm.update(in);
        eigenVectorToCArray(out, torqueCmdOut.torqueRequestBody);
    }

    this->cmdTorqueOutMsg.write(&torqueCmdOut, moduleID, callTime);
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void RateControl::setDerivativeGainP(const float P) { this->algorithm.setDerivativeGainP(P); }

/*! Getter method for the derivative gain P.
 @return const float
*/
float RateControl::getDerivativeGainP() const { return this->algorithm.getDerivativeGainP(); }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateControl::setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B) {
    this->algorithm.setKnownTorquePntB_B(knownTorquePntB_B);
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
const Eigen::Vector3f& RateControl::getKnownTorquePntB_B() const { return this->algorithm.getKnownTorquePntB_B(); }
