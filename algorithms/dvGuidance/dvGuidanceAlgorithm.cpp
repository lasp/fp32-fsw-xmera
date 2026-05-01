#include "dvGuidanceAlgorithm.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/timeConstants.h"

DvGuidanceOutput DvGuidanceAlgorithm::update(const Eigen::Vector3d& dvInrtlCmd,
                                             const Eigen::Vector3d& dvRotVecUnit,
                                             const double dvRotVecMag,
                                             const uint64_t burnStartTime,
                                             const uint64_t callTime) const {
    // base burn frame Bub: 1st axis along dvHat_N, 2nd axis perpendicular to {dvHat_N, dvRotVecUnit},
    // 3rd axis completes the right-handed triad. The DCM rows are the Bub axes in N coordinates.
    const Eigen::Vector3d dvHat_N = dvInrtlCmd.normalized();
    Eigen::Matrix3d dcm_BubN;
    dcm_BubN.row(0) = dvHat_N;
    dcm_BubN.row(1) = dvRotVecUnit.cross(dvHat_N).normalized();
    dcm_BubN.row(2) = dcm_BubN.row(0).cross(dcm_BubN.row(1)).normalized();

    const double burnTime =
        static_cast<double>(static_cast<int64_t>(callTime) - static_cast<int64_t>(burnStartTime)) * kNano2Sec;

    // current burn frame Bu = base burn frame rotated about its 3rd axis by (dvRotVecMag * burnTime)
    const Eigen::Vector3d prv = {0.0, 0.0, dvRotVecMag * burnTime};
    const Eigen::Matrix3d dcm_ButBub = prvToDcm(prv);
    const Eigen::Matrix3d dcm_ButN = dcm_ButBub * dcm_BubN;

    DvGuidanceOutput out;
    out.sigma_RN = dcmToMrp(dcm_ButN);
    // angular velocity is dvRotVecMag along the 3rd Bu axis, expressed in N
    out.omega_RN_N = dvRotVecMag * dcm_ButN.row(2).transpose();
    out.domega_RN_N = Eigen::Vector3d::Zero();
    return out;
}
