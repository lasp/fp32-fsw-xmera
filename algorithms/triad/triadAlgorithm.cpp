#include "triadAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <math.h>
#include <Eigen/Core>

TriadAlgorithm::TriadAlgorithm(const TriadConfig& config) : cfg(config) {}

void TriadAlgorithm::setConfig(const TriadConfig& config) { this->cfg = config; }

Eigen::Vector3f TriadAlgorithm::update(const Eigen::Vector3f& sigma_BN,
                                       const Eigen::Vector3f& rHat_SB_B,
                                       const Eigen::Vector3f& hRefHat_B) const {
    /*! Compute angle between solar array drive axis and thrust direction */
    const Eigen::Vector3f a1 = this->cfg.getA1Hat_B().normalized();
    const float sadaAxisToThrustAngle = safeAcosf(fabsf(a1.dot(hRefHat_B)));

    /*! Return current attitude if solar array drive axis and thrust direction are nearly parallel or if either
     * thrustHat_B or rHat_SB_B are zero */
    if (sadaAxisToThrustAngle < kParallelThresholdRad || hRefHat_B.stableNorm() == 0.0F ||
        rHat_SB_B.stableNorm() == 0.0F) {
        return mrpSwitch(sigma_BN, 1.0F);
    }

    /*! Triad (D frame) basis vectors in hub reference frame */
    Eigen::Matrix3f RD;
    const Eigen::Vector3f r2 = hRefHat_B.normalized();
    const Eigen::Vector3f r3 = a1.cross(hRefHat_B).normalized();
    const Eigen::Vector3f r1 = r2.cross(r3).normalized();
    RD.col(0) = r1;
    RD.col(1) = r2;
    RD.col(2) = r3;

    /*! Compute angle between sun direction and thrust inertial reference direction */
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_N = (dcm_BN.transpose() * rHat_SB_B).normalized();
    const Eigen::Vector3f hReqHat_N = this->cfg.getHHat_N();
    const float SPE = safeAcosf(fabsf(rHat_SB_N.dot(hReqHat_N)));

    /*! Triad (D Frame) basis vectors in inertial frame */
    const Eigen::Vector3f n2 = hReqHat_N;
    Eigen::Vector3f n1 = Eigen::Vector3f::Zero();
    Eigen::Vector3f n3 = Eigen::Vector3f::Zero();

    /*! If sun direction and thrust inertial reference are nearly parallel, cross the second triad axis instead with the
     * configured inertial z-axis */
    if (fabsf(SPE) < kParallelThresholdRad) {
        const Eigen::Vector3f zHat_N = (this->cfg.getSignOfZHat_N() * Eigen::Vector3f::UnitZ()).normalized();
        n3 = zHat_N.cross(n2).normalized();
        n1 = n2.cross(n3);
    } else {
        // Normal triad otherwise
        n1 = rHat_SB_N.cross(n2).normalized();
        n3 = n1.cross(n2);
    }
    Eigen::Matrix3f ND;
    ND.col(0) = n1;
    ND.col(1) = n2;
    ND.col(2) = n3;

    const Eigen::Matrix3f RN = RD * ND.transpose();

    return dcmToMrp(RN);
}
