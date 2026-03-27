/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "rateControlAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include "../utilities/validInertiaCheck.h"

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return Eigen::Vector3d control torque
 @param InputGuidanceData Attitude guidance input
*/
Eigen::Vector3f RateControlAlgorithm::update(const Eigen::Vector3f& omega_BR_B,
                                             const Eigen::Vector3f& omega_RN_B,
                                             const Eigen::Vector3f& domega_RN_B) const {
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;
    const Eigen::Vector3f Lr = -this->P * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                               this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                               this->knownTorquePntB_B;  // [Nm]
    return Lr;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @return void
 @param spacecraftInertia spacecraft inertia
*/
void RateControlAlgorithm::setSpacecraftInertia(const Eigen::Matrix3f& spacecraftInertia) {
    if (inertiaIsValid(spacecraftInertia)) {
        this->ISCPntB_B = spacecraftInertia;
    } else {
        FS_THROW_INVALID_ARGUMENT("Invalid spacecraft inertia");
    }
}

/*! Setter method for the derivative gain P.
 @return void
 @param derivativeGainP [N*m*s] Rate error feedback gain applied
*/
void RateControlAlgorithm::setDerivativeGainP(const float derivativeGainP) {
    if (derivativeGainP < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->P = derivativeGainP;
}

/*! Getter method for the derivative gain P.
 @return const float
*/
float RateControlAlgorithm::getDerivativeGainP() const { return this->P; }

/*! Setter method for the known external torque about point B.
 @return void
 @param torquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateControlAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f& torquePntB_B) {
    this->knownTorquePntB_B = torquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
const Eigen::Vector3f& RateControlAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
