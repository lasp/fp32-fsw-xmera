/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "rateControlAlgorithm.h"

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"

#include <Eigen/Geometry>

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return torqueCmdOut
 @param attGuidIn Attitude guidance input
*/
CmdTorqueBodyMsgF32Payload RateControlAlgorithm::update(AttGuidMsgF32Payload attGuidIn) const {
    CmdTorqueBodyMsgF32Payload torqueCmdOut{};

    // Compute required attitude control torque vector
    const Eigen::Vector3f omega_BR_B = cArrayAsEigenVector(attGuidIn.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayAsEigenVector(attGuidIn.omega_RN_B);
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;
    const Eigen::Vector3f domega_RN_B = cArrayAsEigenVector(attGuidIn.domega_RN_B);
    const Eigen::Vector3f Lr = -this->P * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                               this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                               this->knownTorquePntB_B;  // [Nm]

    eigenVectorToCArray(Lr, torqueCmdOut.torqueRequestBody);

    return torqueCmdOut;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @return void
 @param vehicleConfigIn Vehicle config input
*/
void RateControlAlgorithm::setSpacecraftInertia(VehicleConfigMsgF32Payload vehicleConfigIn) {
    this->ISCPntB_B = cArrayAsEigenMatrix3(vehicleConfigIn.ISCPntB_B);
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void RateControlAlgorithm::setDerivativeGainP(const float P) {
    if (P < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->P = P;
}

/*! Getter method for the derivative gain P.
 @return const float
*/
float RateControlAlgorithm::getDerivativeGainP() const { return this->P; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateControlAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
const Eigen::Vector3f& RateControlAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
