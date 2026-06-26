#include "triadAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <math.h>
#include <Eigen/Core>
#include <utility>

TriadAlgorithm::TriadAlgorithm(TriadConfig config) : cfg(std::move(config)) {}

void TriadAlgorithm::setConfig(const TriadConfig& config) { this->cfg = config; }

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// bugprone-easily-swappable-parameters: the Vector3f attitude/direction inputs are documented in
// the header and follow the standard (sigma_BN, rHat_SB_B, thrustHat_B) ordering.
Eigen::Vector3f TriadAlgorithm::update(const Eigen::Vector3f& sigma_BN,
                                       const Eigen::Vector3f& rHat_SB_B,
                                       const Eigen::Vector3f& thrustHat_B) const {
    /*! Set the default reference attitude to the current spacecraft attitude. Used if a valid triad cannot be formed */
    Eigen::Vector3f sigma_RN = mrpSwitch(sigma_BN, 1.0F);

    /*! Compute angle between solar array drive axis and thrust direction */
    const Eigen::Vector3f sadaHat_B = this->cfg.getSadaHat_B().normalized();
    const float sadaAxisToThrustAngle = safeAcosf(fabsf(sadaHat_B.dot(thrustHat_B)));

    /*! Configuration is valid only if the solar array drive axis and thrust direction are not nearly parallel and
     * neither thrustHat_B or rHat_SB_B are zero */
    const bool isValidConfiguration = sadaAxisToThrustAngle >= kParallelThresholdRad &&
                                      thrustHat_B.stableNorm() != 0.0F && rHat_SB_B.stableNorm() != 0.0F;

    if (isValidConfiguration) {
        /*! Triad (D frame) basis vectors in hub reference frame */
        const Eigen::Vector3f d2Hat_B = thrustHat_B.normalized();
        const Eigen::Vector3f d3Hat_B = sadaHat_B.cross(d2Hat_B).normalized();
        const Eigen::Vector3f d1Hat_B = d2Hat_B.cross(d3Hat_B).normalized();
        Eigen::Matrix3f dcm_BD;
        dcm_BD.col(0) = d1Hat_B;
        dcm_BD.col(1) = d2Hat_B;
        dcm_BD.col(2) = d3Hat_B;

        /*! Compute angle between sun direction and thrust inertial reference direction */
        const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
        const Eigen::Vector3f rHat_SB_N = (dcm_BN.transpose() * rHat_SB_B).normalized();
        const Eigen::Vector3f thrustRefHat_N = this->cfg.getThrustReqHat_N();
        const float sunToThrustRefAngle = safeAcosf(fabsf(rHat_SB_N.dot(thrustRefHat_N)));

        /*! Triad (D Frame) basis vectors in inertial frame */
        const Eigen::Vector3f d2Hat_N = thrustRefHat_N;  // NOLINT(performance-unnecessary-copy-initialization)
        Eigen::Vector3f d1Hat_N = Eigen::Vector3f::Zero();
        Eigen::Vector3f d3Hat_N = Eigen::Vector3f::Zero();

        /*! If sun direction and thrust inertial reference are nearly parallel, cross the second triad axis instead
         * with the configured inertial z-axis */
        bool isFallbackValid = true;
        if (fabsf(sunToThrustRefAngle) < kParallelThresholdRad) {
            const Eigen::Vector3f zHat_N = (this->cfg.getSignOfZHat_N() * Eigen::Vector3f::UnitZ()).normalized();

            /*! Keep the default current attitude if the fallback inertial z-axis is nearly parallel to the thrust
             * reference direction (undefined) */
            const float zToThrustRefAngle = safeAcosf(fabsf(zHat_N.dot(d2Hat_N)));
            isFallbackValid = zToThrustRefAngle >= kParallelThresholdRad;
            if (isFallbackValid) {
                d3Hat_N = zHat_N.cross(d2Hat_N).normalized();
                d1Hat_N = d2Hat_N.cross(d3Hat_N).normalized();
            }
        } else {
            // Normal triad otherwise
            d1Hat_N = rHat_SB_N.cross(d2Hat_N).normalized();
            d3Hat_N = d1Hat_N.cross(d2Hat_N).normalized();
        }

        if (isFallbackValid) {
            Eigen::Matrix3f dcm_ND;
            dcm_ND.col(0) = d1Hat_N;
            dcm_ND.col(1) = d2Hat_N;
            dcm_ND.col(2) = d3Hat_N;

            const Eigen::Matrix3f dcm_RN = dcm_BD * dcm_ND.transpose();
            sigma_RN = dcmToMrp(dcm_RN);
        }
    }

    return sigma_RN;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
