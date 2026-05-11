#include "dvGuidanceAlgorithm.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/timeConstants.h"

DvGuidanceAlgorithm::DvGuidanceAlgorithm(const DvGuidanceConfig& config) : cfg(config) {}

void DvGuidanceAlgorithm::setConfig(const DvGuidanceConfig& config) { this->cfg = config; }

// NOLINTBEGIN(readability-convert-member-functions-to-static, bugprone-easily-swappable-parameters)
// readability-convert-member-functions-to-static: DvGuidanceConfig is intentionally empty for this
// algorithm; the cfg member is held for API consistency with the standard two-phase-init pattern.
// bugprone-easily-swappable-parameters: the Vector3f / float / uint64 inputs are documented in the
// header and follow the burn-command struct ordering.
DvGuidanceOutput DvGuidanceAlgorithm::update(const Eigen::Vector3f& dvInrtlCmd,
                                             const Eigen::Vector3f& dvRotVecUnit,
                                             const float dvRotVecMag,
                                             const uint64_t burnStartTime,
                                             const uint64_t callTime) const {
    // base burn frame Bub: 1st axis along dvHat_N, 2nd axis perpendicular to {dvHat_N, dvRotVecUnit},
    // 3rd axis completes the right-handed triad. The DCM rows are the Bub axes in N coordinates.
    const Eigen::Vector3f dvHat_N = dvInrtlCmd.normalized();
    Eigen::Matrix3f dcm_BubN;
    dcm_BubN.row(0) = dvHat_N;
    dcm_BubN.row(1) = dvRotVecUnit.cross(dvHat_N).normalized();
    dcm_BubN.row(2) = dcm_BubN.row(0).cross(dcm_BubN.row(1)).normalized();

    const float burnTime =
        static_cast<float>(static_cast<int64_t>(callTime) - static_cast<int64_t>(burnStartTime)) * kNano2SecF;

    // current burn frame Bu = base burn frame rotated about its 3rd axis by (dvRotVecMag * burnTime)
    const Eigen::Vector3f prv = {0.0F, 0.0F, dvRotVecMag * burnTime};
    const Eigen::Matrix3f dcm_ButBub = prvToDcm(prv);
    const Eigen::Matrix3f dcm_ButN = dcm_ButBub * dcm_BubN;

    DvGuidanceOutput out;
    out.sigma_RN = dcmToMrp(dcm_ButN);
    // angular velocity is dvRotVecMag along the 3rd Bu axis, expressed in N
    out.omega_RN_N = dvRotVecMag * dcm_ButN.row(2).transpose();
    out.domega_RN_N = Eigen::Vector3f::Zero();
    return out;
}
// NOLINTEND(readability-convert-member-functions-to-static, bugprone-easily-swappable-parameters)
