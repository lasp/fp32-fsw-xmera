/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "rateControlAlgorithm.h"
#include "../utilities/validInertiaCheck.h"
#include "utilities/freestandingInvalidArgument.h"

/*! Computes the required control torque Lr from the attitude rate error and the reference frame angular acceleration.
 @param omega_BR_B the angular velocity of the body frame relative to the reference frame,
 expressed in body frame coordinates [rad/s]
 @param domega_RN_B the time derivative of the reference frame angular velocity, expressed in body
 frame coordinates [rad/s^2]
 @return the required control torque, expressed in body frame coordinates [Nm]
*/
Eigen::Vector3f RateControlAlgorithm::update(const Eigen::Vector3f& omega_BR_B,
                                             const Eigen::Vector3f& domega_RN_B) const {
    const Eigen::Vector3f Lr = -this->P * omega_BR_B + this->ISCPntB_B * domega_RN_B - this->knownTorquePntB_B;  // [Nm]
    return Lr;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @param spacecraftInertia the spacecraft inertia to set
*/
void RateControlAlgorithm::setSpacecraftInertia(const Eigen::Matrix3f& spacecraftInertia) {
    if (inertiaIsValid(spacecraftInertia)) {
        this->ISCPntB_B = spacecraftInertia;
    } else {
        FS_THROW_INVALID_ARGUMENT("Invalid spacecraft inertia");
    }
}

/*! Setter method for the derivative gain P.
 @param derivativeGainP the derivative gain P
*/
void RateControlAlgorithm::setDerivativeGainP(const float derivativeGainP) {
    if (derivativeGainP < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->P = derivativeGainP;
}

/*! Getter method for the derivative gain P.
 @return the derivative gain P
*/
float RateControlAlgorithm::getDerivativeGainP() const { return this->P; }

/*! Setter method for the known external torque.
 @param torquePntB_B the known external torque [N*m] expressed in body frame coordinates
*/
void RateControlAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f& torquePntB_B) {
    this->knownTorquePntB_B = torquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return the known external torque [N*m] expressed in body frame coordinates
*/
const Eigen::Vector3f& RateControlAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
