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
 @param rHatIn_SB_B Sun pointing vector in body frame
 @param theta current panel angular displacement [rad]
*/
float SolarArrayReferenceAlgorithm::update(
    const Eigen::Vector3f& sigma_BN,
    const Eigen::Vector3f& sigma_RN,  // NOLINT(bugprone-easily-swappable-parameters)
    const Eigen::Vector3f& rHatIn_SB_B,
    const float theta) const {
    float thetaRef{};
    switch (this->trackingMode) {
        case TrackingMode::AUTO_TRACK: {
            /*! track Sun in reference frame R, i.e. in the body frame that will be obtained at the end of the slew */
            const Eigen::Vector3f rHat_SB_Bc =
                rHatIn_SB_B.stableNormalized();  // Sun direction in current body frame Bc
            const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
            const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
            const Eigen::Matrix3f dcm_RB = dcm_RN * dcm_BN.transpose();
            // assume body frame B equals reference frame R (end of slew)
            const Eigen::Vector3f rHat_SB_B = (dcm_RB * rHat_SB_Bc).stableNormalized();

            /*! check if sun direction is nearly aligned with drive axis */
            const float sunDriveAngle = safeAcosf(fabsf(rHat_SB_B.dot(this->a1Hat_B)));

            /*! compute reference angle and store in output */
            if (sunDriveAngle < this->alignmentThreshold || rHat_SB_B.stableNorm() == 0.0F) {
                // sun direction is nearly parallel to drive axis, no preferred rotation angle so set reference to
                // current angle
                thetaRef = theta;
            } else {
                /*! required solar array surface normal direction to align with Sun as well as possible */
                thetaRef = safeAtan2f(this->a3Hat_B.dot(rHat_SB_B), this->a2Hat_B.dot(rHat_SB_B)) + this->offsetAngle;
            }
            break;
        }
        case TrackingMode::SPECIFIED_ANGLE: {
            thetaRef = this->specifiedArrayAngle + this->offsetAngle;
            break;
        }
    }

    const float thetaRefOut = safeAtan2f(safeSinf(thetaRef), safeCosf(thetaRef));

    return thetaRefOut;
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
    this->a3Hat_B = (a1.cross(a2)).stableNormalized();
    this->a2Hat_B = (this->a3Hat_B.cross(a1)).stableNormalized();
}

/*! Get the solar array drive axis and surface normal in body frame coordinates.
 *  @return std::array<Eigen::Vector3f, 2> [driveAxis, surfaceNormal]
 */
std::array<Eigen::Vector3f, 2> SolarArrayReferenceAlgorithm::getSolarArrayAxes_B() const {
    return {this->a1Hat_B, this->a2Hat_B};
}

/*! Set the alignment threshold angle between sun direction and drive axis.
 *  The lower bound of 1e-3 rad reflects the fp32 precision floor for the alignment check: with unit-vector
 *  rounding of O(eps) ~ 1e-7, the dot product comes out 1 - O(1e-7), and acos amplifies this to
 *  sqrt(2 * 1e-7) ~ 5e-4 rad. Any threshold smaller than ~1e-3 rad is below the noise floor and would make
 *  the alignment check unreliable.
 *  @param threshold [rad] angle threshold in [1e-3, pi/2]
 */
void SolarArrayReferenceAlgorithm::setAlignmentThreshold(const float threshold) {
    constexpr float minAlignmentThreshold = 1e-3F;
    if (threshold < minAlignmentThreshold || threshold > std::numbers::pi_v<float> / 2.0F) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm.alignmentThreshold must be in [1e-3, pi/2].");
    }
    this->alignmentThreshold = threshold;
}

/*! Get the alignment threshold angle between sun direction and drive axis.
 *  @return float [rad] alignment threshold
 */
float SolarArrayReferenceAlgorithm::getAlignmentThreshold() const { return this->alignmentThreshold; }

/*! Set the tracking mode of the solar array.
 *  @param mode tracking mode (AUTO_TRACK or SPECIFIED_ANGLE)
 */
void SolarArrayReferenceAlgorithm::setTrackingMode(const TrackingMode mode) { this->trackingMode = mode; }

/*! Get the tracking mode of the solar array.
 *  @return TrackingMode current tracking mode
 */
TrackingMode SolarArrayReferenceAlgorithm::getTrackingMode() const { return this->trackingMode; }

/*! Set the specified reference array angle (used when trackingMode is SPECIFIED_ANGLE).
 *  @param angle [rad] specified reference array angle, must be in [-pi, pi]
 */
void SolarArrayReferenceAlgorithm::setSpecifiedArrayAngle(const float angle) {
    constexpr float pi = std::numbers::pi_v<float>;
    if (angle < -pi || angle > pi) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm.specifiedArrayAngle must be in [-pi, pi].");
    }
    this->specifiedArrayAngle = angle;
}

/*! Get the specified reference array angle.
 *  @return float [rad] specified reference array angle (as stored, not wrapped)
 */
float SolarArrayReferenceAlgorithm::getSpecifiedArrayAngle() const { return this->specifiedArrayAngle; }

/*! Set the offset angle added to the computed reference angle before wrapping.
 *  @param angle [rad] offset angle, must be in [-pi, pi]
 */
void SolarArrayReferenceAlgorithm::setOffsetAngle(const float angle) {
    constexpr float pi = std::numbers::pi_v<float>;
    if (angle < -pi || angle > pi) {
        FSW_THROW_INVALID_ARGUMENT("solarArrayReferenceAlgorithm.offsetAngle must be in [-pi, pi].");
    }
    this->offsetAngle = angle;
}

/*! Get the offset angle.
 *  @return float [rad] offset angle (as stored, not wrapped)
 */
float SolarArrayReferenceAlgorithm::getOffsetAngle() const { return this->offsetAngle; }
