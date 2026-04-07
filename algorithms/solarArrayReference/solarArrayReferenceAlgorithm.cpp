#include "solarArrayReferenceAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <math.h>
#include <numbers>

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/timeConstants.h"

const float epsilon = 1e-6F;  // module tolerance for zero

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
*/
void SolarArrayReferenceAlgorithm::reset() { this->count = 0; }

/*! This method computes the updated rotation angle reference based on current attitude, reference attitude, and current
 rotation angle
 @return SolarArrayReferenceOutput containing reference theta and thetaDot
 @param sigma_BN body attitude MRP with respect to inertial frame
 @param sigma_RN reference attitude MRP with respect to inertial frame
 @param vehSunPntBdy Sun pointing vector in body frame
 @param theta current panel angular displacement [rad]
 @param callTime The clock time at which the function was called (nanoseconds)
*/
SolarArrayReferenceOutput SolarArrayReferenceAlgorithm::update(const Eigen::Vector3f& sigma_BN,
                                                               const Eigen::Vector3f& sigma_RN,
                                                               const Eigen::Vector3f& vehSunPntBdy,
                                                               float theta,
                                                               uint64_t callTime) {
    SolarArrayReferenceOutput output{};

    /*! read Sun direction in B frame and map it to R frame */
    const Eigen::Vector3f rHat_SB_B = vehSunPntBdy.normalized();
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    const Eigen::Matrix3f dcm_RB = dcm_RN * dcm_BN.transpose();
    Eigen::Vector3f rHat_SB_R = dcm_RB * rHat_SB_B;

    /*! compute solar array frame axes at zero rotation */
    const Eigen::Vector3f a1 = this->a1Hat_B.normalized();
    const Eigen::Vector3f a3 = (a1.cross(this->a2Hat_B)).normalized();
    const Eigen::Vector3f a2 = (a3.cross(a1)).normalized();

    /*! compute solar array reference frame axes at zero rotation */
    const float dotP = a1.dot(rHat_SB_R);
    Eigen::Vector3f a2Hat_R = rHat_SB_R - dotP * a1;
    const float a2Hat_R_norm = a2Hat_R.norm();

    /*! compute current rotation angle thetaC from input msg */
    const float sinThetaC = sinf(theta);
    const float cosThetaC = cosf(theta);
    const float thetaC = atan2f(sinThetaC, cosThetaC);  // clip theta current between 0 and 2*pi

    /*! compute reference angle and store in output */
    constexpr float pi = std::numbers::pi_v<float>;
    if (a2Hat_R_norm < epsilon) {
        // if norm(a2Hat_R) = 0, reference coincides with current angle
        output.theta = theta;
    } else {
        a2Hat_R.normalize();
        const Eigen::Vector3f a1Hat_R = a2.cross(a2Hat_R);
        float thetaR = acosf(fminf(fmaxf(a2.dot(a2Hat_R), -1.0F), 1.0F));
        // if a1 and a1Hat_R are opposite, take the negative of thetaR
        if (a1.dot(a1Hat_R) < 0) {
            thetaR = -thetaR;
        }
        // always make the absolute difference |thetaR-thetaC| smaller that 2*pi
        if (thetaR - thetaC > pi) {
            output.theta = theta + thetaR - thetaC - 2 * pi;
        } else if (thetaR - thetaC < -pi) {
            output.theta = theta + thetaR - thetaC + 2 * pi;
        } else {
            output.theta = theta + thetaR - thetaC;
        }
    }

    /*! implement finite differences to compute thetaDotR */
    if (this->count == 0) {
        output.thetaDot = 0;
    } else {
        const double dt = static_cast<double>(callTime - this->priorT) * kNano2Sec;
        output.thetaDot = static_cast<float>((output.theta - this->priorThetaR) / dt);
    }
    // update stored variables
    this->priorThetaR = output.theta;
    this->priorT = callTime;
    this->count += 1;

    return output;
}

/*! Set the solar array drive axis in body frame coordinates.
 *  The vector must be non-zero; it is normalized before storing.
 *  @param axis [-] solar array drive axis in body frame coordinates
 */
void SolarArrayReferenceAlgorithm::setA1Hat_B(const Eigen::Vector3f& axis) {
    if (axis.norm() == 0) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm.a1Hat_B must be non-zero.");
    }
    this->a1Hat_B = axis.normalized();
}

/*! Get the solar array drive axis in body frame coordinates.
 *  @return Eigen::Vector3f [-] solar array drive axis in body frame coordinates
 */
Eigen::Vector3f SolarArrayReferenceAlgorithm::getA1Hat_B() const { return this->a1Hat_B; }

/*! Set the solar array surface normal at zero rotation.
 *  The vector must be non-zero; it is normalized before storing.
 *  @param normal [-] solar array surface normal at zero rotation
 */
void SolarArrayReferenceAlgorithm::setA2Hat_B(const Eigen::Vector3f& normal) {
    if (normal.norm() == 0) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm.a2Hat_B must be non-zero.");
    }
    this->a2Hat_B = normal.normalized();
}

/*! Get the solar array surface normal at zero rotation.
 *  @return Eigen::Vector3f [-] solar array surface normal at zero rotation
 */
Eigen::Vector3f SolarArrayReferenceAlgorithm::getA2Hat_B() const { return this->a2Hat_B; }
