#include "sunSafePointAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/freestandingInvalidArgument.h"
#include "../utilities/safeMath.h"
#include <Eigen/Geometry>
#include <math.h>
#include <numbers>

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

    const Eigen::Vector3f rHat_SB_B = vehSunPntBdy.stableNormalized();

    // Computing the attitude guidance states sigma_BR and omega_RN_B if valid sun direction is available
    if (rHat_SB_B.stableNorm() > 0.0F) {
        // Compute the current sun angle error
        float const sunAngleErr = safeAcosf(this->sHatBdyCmd.dot(rHat_SB_B));

        // Compute the heading error relative to the sun direction vector
        Eigen::Vector3f sigma_BR{Eigen::Vector3f::Zero()};
        // If Sun heading and desired body axis are essentially aligned, set attitude error to zero.
        if (sunAngleErr >= this->smallAngle) {
            Eigen::Vector3f e_hat{};  // Eigen Axis
            // The commanded body vector nearly is opposite the sun heading
            if (static_cast<float>(std::numbers::pi) - sunAngleErr < this->smallAngle) {
                e_hat = this->sHatBdyCmd.unitOrthogonal();  // find orthogonal unit vector to sHatBdyCmd
            // Normal case where sun and commanded body vectors are not aligned
            } else {
                e_hat = rHat_SB_B.cross(this->sHatBdyCmd);
            }
            Eigen::Vector3f const sunMnvrVec = e_hat / e_hat.norm();
            sigma_BR = safeTanf(sunAngleErr * 0.25F) * sunMnvrVec;
            sigma_BR = mrpSwitch(sigma_BR, 1.0F);
        }

        output.sigma_BR = sigma_BR;
        // Rate tracking error is the body rate to bring spacecraft to rest
        output.omega_RN_B = this->sunAxisSpinRate * rHat_SB_B;
    } else {
        output.sigma_BR = Eigen::Vector3f::Zero();
        output.omega_RN_B = this->omega_RN_B;
    }

    // Compute the hub angular rate error omega_BR_B
    output.omega_BR_B = omega_BN_B - output.omega_RN_B;

    return output;
}

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
    constexpr float normTolerance = 1e-3F;
    if (fabsf(sHat.stableNorm() - 1.0F) > normTolerance) {
        FSW_THROW_INVALID_ARGUMENT("sunSafePoint: sHatBdyCmd norm must be within 1e-3 to 1.0.");
    }
    this->sHatBdyCmd = sHat.normalized();
}
