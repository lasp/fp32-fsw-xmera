#include "solarArrayReferenceAlgorithm.h"
#include "utilities/fsw/safeMath.h"
#include <math.h>

#include "utilities/fsw/rigidBodyKinematics.hpp"

/*! @brief Construct the algorithm with a validated configuration. */
SolarArrayReferenceAlgorithm::SolarArrayReferenceAlgorithm(const SolarArrayReferenceConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the algorithm's stored configuration at runtime. */
void SolarArrayReferenceAlgorithm::setConfig(const SolarArrayReferenceConfig& config) { this->cfg = config; }

/*! @brief Reset the retained runtime state by zeroing the reference angle from the previous update. */
void SolarArrayReferenceAlgorithm::reInitialize() { this->priorThetaRef = 0.0F; }

/*! This method computes the updated rotation angle reference based on the reference attitude
 @return float
 @param sigma_RN reference attitude MRP with respect to inertial frame
 @param rHat_SB_N Sun direction in inertial frame
*/
float SolarArrayReferenceAlgorithm::update(
    const Eigen::Vector3f& sigma_RN,  // NOLINT(bugprone-easily-swappable-parameters)
    const Eigen::Vector3f& rHat_SB_N) {
    const Eigen::Vector3f& a1Hat_B = this->cfg.getDriveAxisHat_B();
    const Eigen::Vector3f& a2Hat_B = this->cfg.getSurfaceNormalHat_B();
    const Eigen::Vector3f& a3Hat_B = this->cfg.getThirdAxisHat_B();

    float thetaRef{};
    switch (this->cfg.getTrackingMode()) {
        case TrackingMode::AUTO_TRACK: {
            /*! track Sun in reference frame R, i.e. in the body frame that will be obtained at the end of the slew */
            const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
            // assume body frame B equals reference frame R (end of slew)
            const Eigen::Vector3f rHat_SB_B = dcm_RN * rHat_SB_N.stableNormalized();

            /*! check if sun direction is nearly aligned with drive axis */
            const float sunDriveAngle = safeAcosf(fabsf(rHat_SB_B.dot(a1Hat_B)));

            /*! compute reference angle and store in output */
            if (sunDriveAngle < this->cfg.getAlignmentThreshold() || rHat_SB_B.stableNorm() == 0.0F) {
                // sun direction is nearly parallel to drive axis, no preferred rotation angle so hold the reference
                // angle from the previous update
                thetaRef = this->priorThetaRef;
            } else {
                /*! required solar array surface normal direction to align with Sun as well as possible */
                thetaRef = safeAtan2f(a3Hat_B.dot(rHat_SB_B), a2Hat_B.dot(rHat_SB_B)) + this->cfg.getOffsetAngle();
            }
            break;
        }
        case TrackingMode::SPECIFIED_ANGLE: {
            thetaRef = this->cfg.getSpecifiedArrayAngle();
            break;
        }
    }

    const float thetaRefOut = safeAtan2f(safeSinf(thetaRef), safeCosf(thetaRef));

    this->priorThetaRef = thetaRefOut;

    return thetaRefOut;
}
