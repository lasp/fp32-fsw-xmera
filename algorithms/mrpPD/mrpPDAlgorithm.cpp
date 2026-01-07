#include "mrpPDAlgorithm.h"
#include "architecture/utilities/eigenSupport.h"
#include <cmath>

/*! Update method for mrpPD control algorithm. This method takes the attitude and rate errors relative to the
 reference frame, as well as the reference frame angular rates and acceleration, and computes the required control
 torque Lr.
 @return void
 @param callTime [ns] Time the method is called
 @param guidInMsg [-] guidance message input
*/
CmdTorqueBodyMsgF32Payload MrpPDAlgorithm::update(uint64_t callTime, AttGuidMsgF32Payload guidInMsg) {
    // Compute hub inertial angular velocity in B-frame components
    Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidInMsg.omega_BR_B);
    Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidInMsg.omega_RN_B);
    Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidInMsg.sigma_BR);
    Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidInMsg.domega_RN_B);

    // Compute required attitude control torque vector
    Eigen::Vector3f Lr = -this->proportionalGain * sigma_BR - this->feedbackGain * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                         this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                         this->knownTorquePntB_B;  // [Nm]

    // Create the output message
    auto torqueCmdMsgF32Payload = CmdTorqueBodyMsgF32Payload();
    eigenVectorToCArray(Lr, torqueCmdMsgF32Payload.torqueRequestBody);

    return torqueCmdMsgF32Payload;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @return void
 @param vehicleConfigIn Vehicle config input
*/
void MrpPDAlgorithm::setSpacecraftInertia(VehicleConfigMsgF32Payload vehicleConfigIn) {
    this->ISCPntB_B = cArrayToEigenMatrix3(vehicleConfigIn.ISCPntB_B);
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void MrpPDAlgorithm::setDerivativeGainP(float P) {
    if (P < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->feedbackGain = P;
}

/*! Getter method for the derivative gain P.
 @return float
*/
float MrpPDAlgorithm::getDerivativeGainP() const { return this->feedbackGain; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorque [N*m] Known external torque expressed in body frame components about point B
*/
void MrpPDAlgorithm::setKnownTorquePntB_B(Eigen::Vector3f& knownTorque) {
    this->knownTorquePntB_B = knownTorque;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f&
*/
const Eigen::Vector3f& MrpPDAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }

/*! Setter method for the proportional gain K.
 @return void
 @param K [rad/s] Proportional gain applied to MRP errors
*/
void MrpPDAlgorithm::setProportionalGainK(float K) {
    if (K < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain K must not be negative");
    }
    this->proportionalGain = K;
}

/*! Getter method for the proportional gain K.
 @return float
*/
float MrpPDAlgorithm::getProportionalGainK() const { return this->proportionalGain; }
