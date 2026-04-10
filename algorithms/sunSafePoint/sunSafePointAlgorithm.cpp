#include "sunSafePointAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/freestandingInvalidArgument.h"
#include "../utilities/safeMath.h"
#include <math.h>
#include <numbers>

/*! Reset method for the sunSafePoint guidance algorithm.
 @return void
*/
void SunSafePointAlgorithm::reset() {
    // Compute an Eigen axis orthogonal to sHatBdyCmd
    Eigen::Vector3f v1 = {1.0F, 0.0F, 0.0F};
    this->eHat180_B = this->sHatBdyCmd.cross(v1);
    if (this->eHat180_B.norm() < 0.1F) {
        v1 = {0.0F, 1.0F, 0.0F};
        this->eHat180_B = this->sHatBdyCmd.cross(v1);
    }
    this->eHat180_B.normalize();
}

/*! Update method for the sunSafePoint guidance algorithm. This method takes the estimated body-observed sun vector
 and computes the current attitude/attitude rate errors to pass on to control.
 @return SunSafePointOutput Attitude guidance output
 @param vehSunPntBdy Sun direction vector in body frame
 @param omega_BN_B Body angular velocity vector
*/
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
SunSafePointOutput SunSafePointAlgorithm::update(const Eigen::Vector3f& vehSunPntBdy,
                                                  const Eigen::Vector3f& omega_BN_B) const {
    SunSafePointOutput output{};

    // Computing the attitude guidance states sigma_BR and omega_RN_B if valid sun direction is available
    if (vehSunPntBdy.norm() > this->minUnitMag) {
        // Compute the current sun angle error
        float const sunAngleErr = safeAcosf(this->sHatBdyCmd.dot(vehSunPntBdy) / vehSunPntBdy.norm());

        // Compute the heading error relative to the sun direction vector
        Eigen::Vector3f sigma_BR{};
        // Sun heading and desired body axis are essentially aligned. Set attitude error to zero.
        if (sunAngleErr < this->smallAngle) {
            sigma_BR = Eigen::Vector3f::Zero();
        } else {
            Eigen::Vector3f e_hat{};  // Eigen Axis
            // The commanded body vector nearly is opposite the sun heading
            if (static_cast<float>(std::numbers::pi) - sunAngleErr < this->smallAngle) {
                e_hat = this->eHat180_B;
            // Normal case where sun and commanded body vectors are not aligned
            } else {
                e_hat = vehSunPntBdy.cross(this->sHatBdyCmd);
            }
            Eigen::Vector3f const sunMnvrVec = e_hat / e_hat.norm();
            sigma_BR = safeTanf(sunAngleErr * 0.25F) * sunMnvrVec;
            sigma_BR = mrpSwitch(sigma_BR, 1.0F);
        }

        output.sigma_BR = sigma_BR;
        // Rate tracking error is the body rate to bring spacecraft to rest
        output.omega_RN_B = this->sunAxisSpinRate / vehSunPntBdy.norm() * vehSunPntBdy;
    } else {
        output.sigma_BR = Eigen::Vector3f::Zero();
        output.omega_RN_B = this->omega_RN_B;
    }

    // Compute the hub angular rate error omega_BR_B
    output.omega_BR_B = omega_BN_B - output.omega_RN_B;

    return output;
}

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
 @param magnitude The minimally acceptable norm of sun body vector (Must be positive)
*/
void SunSafePointAlgorithm::setMinUnitMag(const float magnitude) {
    if (magnitude <= 0.0F) {
        FSW_THROW_INVALID_ARGUMENT("sunSafePoint: minUnitMag must be positive");
    }
    this->minUnitMag = magnitude;
}

/*! Setter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return void
 @param angle [rad] An angle value that specifies what is near 0 or 180 degrees (Must be positive)
*/
void SunSafePointAlgorithm::setSmallAngle(const float angle) {
    if (angle <= 0.0F) {
        FSW_THROW_INVALID_ARGUMENT("sunSafePoint: smallAngle must be positive");
    }
    this->smallAngle = angle;
}

/*! Setter method for the desired constant spin rate about sun heading vector.
 @return void
 @param rate [rad/s] Desired constant spin rate about sun heading vector
*/
void SunSafePointAlgorithm::setSunAxisSpinRate(const float rate) {
    this->sunAxisSpinRate = rate;
}

/*! Setter method for the desired body rate vector if no sun direction is available.
 @return void
 @param omega [rad/s] Desired body rate vector if no sun direction is available
*/
void SunSafePointAlgorithm::setOmega_RN_B(const Eigen::Vector3f& omega) { this->omega_RN_B = omega; }

/*! Setter method for the desired body vector to point at the sun.
 @return void
 @param sHat Desired body vector to point at the sun
*/
void SunSafePointAlgorithm::setSHatBdyCmd(const Eigen::Vector3f& sHat) {
    if (sHat.norm() <= 1e-8F) {
        FSW_THROW_INVALID_ARGUMENT("sunSafePoint: sHatBdyCmd must be a non-zero vector");
    }
    this->sHatBdyCmd = sHat.normalized();
}
