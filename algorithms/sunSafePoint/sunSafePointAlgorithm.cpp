#include "sunSafePointAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "../utilities/safeMath.h"
#include <cassert>
#include <cmath>

/*! Reset method for the sunSafePoint guidance algorithm.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSafePointAlgorithm::reset(uint64_t callTime) {
    // Compute an Eigen axis orthogonal to sHatBdyCmd
    Eigen::Vector3f v1 = {1.0f, 0.0f, 0.0f};
    this->eHat180_B = this->sHatBdyCmd.cross(v1);
    if (this->eHat180_B.norm() < 0.1f) {
        v1 = {0.0f, 1.0f, 0.0f};
        this->eHat180_B = this->sHatBdyCmd.cross(v1);
    }
    this->eHat180_B.normalize();
}

/*! Update method for the sunSafePoint guidance algorithm. This method takes the estimated body-observed sun vector
 and computes the current attitude/attitude rate errors to pass on to control.
 @return AttGuidMsgPayload Attitude guidance message
 @param callTime [ns] Time the method is called
 @param imuInMsg IMU navigation message
 @param sunDirectionInMsg  Sun direction navigation message
*/
AttGuidMsgF32Payload SunSafePointAlgorithm::update(uint64_t callTime,
                                                   NavAttMsgF32Payload imuInMsg,
                                                   NavAttMsgF32Payload sunDirectionInMsg) {
    // Read the current sun body vector estimate input message
    this->sunDirectionInBuffer = sunDirectionInMsg;

    // Determine norm of measured Sun-direction vector
    const float sHatNorm = cArrayToEigenVector(this->sunDirectionInBuffer.vehSunPntBdy).norm();

    // Zero the attitude guidance output buffer message
    this->attGuidanceOutBuffer = AttGuidMsgF32Payload();

    // Computing the attitude guidance states sigma_BR and omega_RN_B
    if (this->sunDirectionIsAvailable(sHatNorm)) {
        this->computeAttGuidanceStates(sHatNorm);
    } else {
        Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();
        eigenVectorToCArray(sigma_BR, this->attGuidanceOutBuffer.sigma_BR);
    }

    // Compute the hub angular rate error omega_BR_B
    this->computeHubAngularRateError(imuInMsg);

    // Create the output guidance message
    eigenVectorToCArray(this->omega_RN_B, this->attGuidanceOutBuffer.omega_RN_B);

    return this->attGuidanceOutBuffer;
}

/*! Method for computing the attitude guidance states sigma_BR and omega_RN_B if a valid sun direction vector is
 available.
 @return void
 @param sHatNorm Norm of measured Sun-direction vector
*/
void SunSafePointAlgorithm::computeAttGuidanceStates(float sHatNorm) {
    // Compute the current sun angle error
    float dotProductNormalized =
        this->sHatBdyCmd.dot(cArrayToEigenVector(this->sunDirectionInBuffer.vehSunPntBdy)) / sHatNorm;
    dotProductNormalized = std::abs(dotProductNormalized) > 1.0f ? dotProductNormalized / std::abs(dotProductNormalized)
                                                                 : dotProductNormalized;
    float sunAngleErr = safeAcosf(dotProductNormalized);

    // Compute the heading error relative to the sun direction vector
    // Sun heading and desired body axis are essentially aligned. Set attitude error to zero.
    if (sunAngleErr < this->smallAngle) {
        Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();
        eigenVectorToCArray(sigma_BR, this->attGuidanceOutBuffer.sigma_BR);
    } else {
        Eigen::Vector3f e_hat;  // Eigen Axis
        // The commanded body vector nearly is opposite the sun heading
        if (static_cast<float>(M_PI) - sunAngleErr < this->smallAngle) {
            e_hat = this->eHat180_B;
            // Normal case where sun and commanded body vectors are not aligned
        } else {
            e_hat = cArrayToEigenVector(this->sunDirectionInBuffer.vehSunPntBdy).cross(this->sHatBdyCmd);
        }
        Eigen::Vector3f sunMnvrVec = e_hat / e_hat.norm();
        Eigen::Vector3f sigma_BR = std::tan(sunAngleErr * 0.25f) * sunMnvrVec;
        sigma_BR = mrpSwitch(sigma_BR, 1.0f);
        eigenVectorToCArray(sigma_BR, this->attGuidanceOutBuffer.sigma_BR);
    }

    // Rate tracking error is the body rate to bring spacecraft to rest
    this->omega_RN_B =
        (this->sunAxisSpinRate / sHatNorm) * cArrayToEigenVector(this->sunDirectionInBuffer.vehSunPntBdy);
}

/*! Method for computing the hub angular rate error omega_BR_B.
 @return void
*/
void SunSafePointAlgorithm::computeHubAngularRateError(NavAttMsgF32Payload imuInMsg) {
    const Eigen::Vector3f omega_BN_B = cArrayToEigenVector(imuInMsg.omega_BN_B);  // [rad/s]
    Eigen::Vector3f omega_BR_B = omega_BN_B - this->omega_RN_B;                   // [rad/s]

    eigenVectorToCArray(omega_BR_B, this->attGuidanceOutBuffer.omega_BR_B);
}

/*! Method for determining if a valid sun direction vector is available.
 @return bool
 @param sHatNorm Norm of measured Sun-direction vector
*/
bool SunSafePointAlgorithm::sunDirectionIsAvailable(const float sHatNorm) const { return sHatNorm > this->minUnitMag; }

/*! Getter method for the minimally accepted sun body vector norm.
 @return float
*/
float SunSafePointAlgorithm::getMinUnitMag() const { return this->minUnitMag; }

/*! Getter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return float
*/
float SunSafePointAlgorithm::getSmallAngle() const { return this->smallAngle; }

/*! Getter method for the desired constant spin rate about sun heading vector.
 @return float
*/
float SunSafePointAlgorithm::getSunAxisSpinRate() const { return this->sunAxisSpinRate; }

/*! Getter method for the desired body rate vector if no sun direction is available.
 @return Eigen::Vector3f
*/
Eigen::Vector3f SunSafePointAlgorithm::getOmega_RN_B() const { return this->omega_RN_B; }

/*! Getter method for the desired body vector to point at the sun.
 @return Eigen::Vector3f
*/
Eigen::Vector3f SunSafePointAlgorithm::getSHatBdyCmd() const { return this->sHatBdyCmd; }

/*! Setter method for the minimally accepted sun body vector norm.
 @return void
 @param minUnitMag The minimally acceptable norm of sun body vector (Must be positive)
*/
void SunSafePointAlgorithm::setMinUnitMag(float minUnitMag) {
    assert(minUnitMag > 0.0f);
    this->minUnitMag = std::abs(minUnitMag);
}

/*! Setter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return void
 @param smallAngle [rad] An angle value that specifies what is near 0 or 180 degrees (Must be positive)
*/
void SunSafePointAlgorithm::setSmallAngle(float smallAngle) {
    assert(smallAngle >= 0.0f);
    this->smallAngle = std::abs(smallAngle);
}

/*! Setter method for the desired constant spin rate about sun heading vector.
 @return void
 @param sunAxisSpinRate [rad/s] Desired constant spin rate about sun heading vector
*/
void SunSafePointAlgorithm::setSunAxisSpinRate(const float sunAxisSpinRate) {
    this->sunAxisSpinRate = sunAxisSpinRate;
}

/*! Setter method for the desired body rate vector if no sun direction is available.
 @return void
 @param omega_RN_B [rad/s] Desired body rate vector if no sun direction is available
*/
void SunSafePointAlgorithm::setOmega_RN_B(const Eigen::Vector3f& omega_RN_B) { this->omega_RN_B = omega_RN_B; }

/*! Setter method for the desired body vector to point at the sun.
 @return void
 @param sHatBdyCmd Desired body vector to point at the sun
*/
void SunSafePointAlgorithm::setSHatBdyCmd(Eigen::Vector3f& sHatBdyCmd) {
    assert(sHatBdyCmd.norm() > 1e-8f);
    this->sHatBdyCmd = sHatBdyCmd.normalized();
}
