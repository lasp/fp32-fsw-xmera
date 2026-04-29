#include "mrpPDAlgorithm.h"
#include "../utilities/validInertiaCheck.h"
#include "utilities/freestandingInvalidArgument.h"

/*! Update method for mrpPD control algorithm. This method takes the attitude and rate errors relative to the
 reference frame, as well as the reference frame angular rates and acceleration, and computes the required control
 torque Lr.
 @return Eigen::Vector3f
 @param sigma_BR Body to reference MRP
 @param omega_BR_B Body to reference rate
 @param domega_RN_B Body to reference acceleration
*/
Eigen::Vector3f MrpPDAlgorithm::update(const Eigen::Vector3f& sigma_BR,
                                       const Eigen::Vector3f& omega_BR_B,
                                       const Eigen::Vector3f& domega_RN_B) const {
    // Compute required attitude control torque vector
    const Eigen::Vector3f Lr = -this->proportionalGain * sigma_BR - this->feedbackGain * omega_BR_B +
                               this->ISCPntB_B * domega_RN_B - this->knownTorquePntB_B;  // [Nm]

    return Lr;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @return void
 @param inertia Inertia matrix
*/
void MrpPDAlgorithm::setSpacecraftInertia(const Eigen::Matrix3f& inertia) {
    if (!inertiaIsValid(inertia)) {
        FSW_THROW_INVALID_ARGUMENT("Matrix inertia did not pass validity checks");
    }
    this->ISCPntB_B = inertia;
}

/*! This method gets the spacecraft inertia matrix
 @return Eigen::Matrix3f
*/
Eigen::Matrix3f MrpPDAlgorithm::getSpacecraftInertia() const { return this->ISCPntB_B; }

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void MrpPDAlgorithm::setDerivativeGainP(float P) {
    if (P < 0.0) {
        FSW_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
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
void MrpPDAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f& knownTorque) { this->knownTorquePntB_B = knownTorque; }

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
        FSW_THROW_INVALID_ARGUMENT("Feedback gain K must not be negative");
    }
    this->proportionalGain = K;
}

/*! Getter method for the proportional gain K.
 @return float
*/
float MrpPDAlgorithm::getProportionalGainK() const { return this->proportionalGain; }
