#include "solarArrayReferenceAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/safeMath.h"
#include <math.h>
#include <numbers>

#include "architecture/utilities/rigidBodyKinematics.hpp"


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
    const Eigen::Vector3f rHat_SB_Bc = vehSunPntBdy.stableNormalized();  // Sun direction in current body frame Bc
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    const Eigen::Matrix3f dcm_RB = dcm_RN * dcm_BN.transpose();
    // assume body frame B equals reference frame R (end of slew)
    const Eigen::Vector3f rHat_SB_B = (dcm_RB * rHat_SB_Bc).stableNormalized();

    /*! check if sun direction is nearly aligned with drive axis */
    const float sunDriveAngle = safeAcosf(fabsf(rHat_SB_B.dot(this->a1Hat_B)));

    /*! compute reference angle and store in output */
    float thetaRef{};
    if (sunDriveAngle < this->alignmentThreshold || rHat_SB_B.stableNorm() == 0.0F) {
        // sun direction is nearly parallel to drive axis, no preferred rotation angle so set reference to current angle
        thetaRef = safeAtan2f(safeSinf(theta), safeCosf(theta));  // wrap current theta between -pi and pi;
    } else {
        /*! required solar array surface normal direction to align with Sun as well as possible */
        const Eigen::Vector3f a2HatRef_B = (rHat_SB_B - this->a1Hat_B.dot(rHat_SB_B) * this->a1Hat_B).stableNormalized();
        const Eigen::Vector3f a1HatRef_B = this->a2Hat_B.cross(a2HatRef_B);
        thetaRef = safeAcosf(this->a2Hat_B.dot(a2HatRef_B));
        // if this->a1Hat_B and a1HatRef_B are opposite, take the negative of thetaRef
        if (this->a1Hat_B.dot(a1HatRef_B) < 0) {
            thetaRef = -thetaRef;
        }
    }

    return thetaRef;
}

/*! Set the solar array drive axis and surface normal in body frame coordinates.
 *  Both vectors must have norm within 1e-3 to 1.0 and must be orthogonal (|dot| < 1e-5 after normalization).
 *  Both are normalized before storing.
 *  @param driveAxis [-] solar array drive axis in body frame coordinates
 *  @param surfaceNormal [-] solar array surface normal at zero rotation
 */
void SolarArrayReferenceAlgorithm::setSolarArrayAxes_B(const Eigen::Vector3f& driveAxis,
                                                       const Eigen::Vector3f& surfaceNormal) {
    constexpr float normTolerance = 1e-3F;
    constexpr float maxDot = 1e-5F;
    if (fabsf(driveAxis.stableNorm() - 1.0F) > normTolerance) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm: drive axis norm must be within 1e-3 to 1.0.");
    }
    if (fabsf(surfaceNormal.stableNorm() - 1.0F) > normTolerance) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm: surface normal norm must be within 1e-3 to 1.0.");
    }
    const Eigen::Vector3f a1 = driveAxis.stableNormalized();
    const Eigen::Vector3f a2 = surfaceNormal.stableNormalized();
    if (fabsf(a1.dot(a2)) > maxDot) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm: drive axis and surface normal must be orthogonal.");
    }
    this->a1Hat_B = a1;
    // Orthogonalize a2 against a1 to ensure exact orthogonality
    const Eigen::Vector3f a3 = (a1.cross(a2)).stableNormalized();
    this->a2Hat_B = (a3.cross(a1)).stableNormalized();
}

/*! Get the solar array drive axis and surface normal in body frame coordinates.
 *  @return std::array<Eigen::Vector3f, 2> [driveAxis, surfaceNormal]
 */
std::array<Eigen::Vector3f, 2> SolarArrayReferenceAlgorithm::getSolarArrayAxes_B() const {
    return {this->a1Hat_B, this->a2Hat_B};
}

/*! Set the alignment threshold angle between sun direction and drive axis.
 *  @param threshold [rad] angle threshold in [0, pi/2]
 */
void SolarArrayReferenceAlgorithm::setAlignmentThreshold(const float threshold) {
    if (threshold < 0.0F || threshold > std::numbers::pi_v<float> / 2.0F) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm.alignmentThreshold must be in [0, pi/2].");
    }
    this->alignmentThreshold = threshold;
}

/*! Get the alignment threshold angle between sun direction and drive axis.
 *  @return float [rad] alignment threshold
 */
float SolarArrayReferenceAlgorithm::getAlignmentThreshold() const { return this->alignmentThreshold; }
