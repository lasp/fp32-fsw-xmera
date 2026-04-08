#include "solarArrayReferenceAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <math.h>
#include <numbers>

#include "architecture/utilities/rigidBodyKinematics.hpp"

const float epsilon = 1e-6F;  // module tolerance for zero

/*! This method computes the updated rotation angle reference based on current attitude, reference attitude, and current
 rotation angle
 @return float
 @param sigma_BN body attitude MRP with respect to inertial frame
 @param sigma_RN reference attitude MRP with respect to inertial frame
 @param vehSunPntBdy Sun pointing vector in body frame
 @param theta current panel angular displacement [rad]
*/
float SolarArrayReferenceAlgorithm::update(const Eigen::Vector3f& sigma_BN,
                                                               const Eigen::Vector3f& sigma_RN,
                                                               const Eigen::Vector3f& vehSunPntBdy,
                                                               const float theta) const {

    /*! track Sun in reference frame R, i.e. in the body frame that will be obtained at the end of the slew */
    const Eigen::Vector3f rHat_SB_Bc = vehSunPntBdy.normalized();  // Sun direction in current body frame Bc
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    const Eigen::Matrix3f dcm_RB = dcm_RN * dcm_BN.transpose();
    const Eigen::Vector3f rHat_SB_B = dcm_RB * rHat_SB_Bc;  // assume body frame B equals reference frame R (end of slew)

    /*! compute solar array frame axes at zero rotation */
    const Eigen::Vector3f a1 = this->a1Hat_B.normalized();
    const Eigen::Vector3f a3 = (a1.cross(this->a2Hat_B)).normalized();
    const Eigen::Vector3f a2 = (a3.cross(a1)).normalized();

    /*! required solar array surface normal direction to align surface normal with Sun direction as well as possible */
    const float dotP = a1.dot(rHat_SB_B);
    Eigen::Vector3f a2HatRef_B = rHat_SB_B - dotP * a1;
    const float a2HatRef_B_norm = a2HatRef_B.norm();

    /*! compute current rotation angle thetaC from input msg */
    const float sinThetaC = sinf(theta);
    const float cosThetaC = cosf(theta);
    const float thetaC = atan2f(sinThetaC, cosThetaC);  // wrap current theta between -pi and pi

    /*! compute reference angle and store in output */
    float thetaRefOut{};
    constexpr float pi = std::numbers::pi_v<float>;
    if (a2HatRef_B_norm < epsilon) {
        // if norm(a2HatRef_B) = 0, drive axis is aligned with sun direction, so no preferred angle and leave at current
        thetaRefOut = theta;
    } else {
        a2HatRef_B.normalize();
        const Eigen::Vector3f a1HatRef_B = a2.cross(a2HatRef_B);
        float thetaRef = acosf(fminf(fmaxf(a2.dot(a2HatRef_B), -1.0F), 1.0F));
        // if a1 and a1HatRef_B are opposite, take the negative of thetaRef
        if (a1.dot(a1HatRef_B) < 0) {
            thetaRef = -thetaRef;
        }
        // always make the absolute difference |thetaR-thetaC| is smaller than pi
        if (thetaRef - thetaC > pi) {
            thetaRefOut = theta + thetaRef - thetaC - 2 * pi;
        } else if (thetaRef - thetaC < -pi) {
            thetaRefOut = theta + thetaRef - thetaC + 2 * pi;
        } else {
            thetaRefOut = theta + thetaRef - thetaC;
        }
    }

    return thetaRefOut;
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
