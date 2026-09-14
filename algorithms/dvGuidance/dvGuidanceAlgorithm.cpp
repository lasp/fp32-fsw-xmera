#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/timeConstants.h"
#include <math.h>
#include <Eigen/Geometry>

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// bugprone-easily-swappable-parameters: the Vector3f / float / uint64 inputs are documented in the
// header and follow the burn-command struct ordering.
DvGuidanceOutput DvGuidanceAlgorithm::update(const Eigen::Vector3f& dvInrtlCmd,
                                             const Eigen::Vector3f& dvRotVecUnit,
                                             const float dvRotVecMag,
                                             const uint64_t burnStartTime,
                                             const uint64_t callTime) {
    // Guard: a near-zero or non-finite delta-V has no defined direction. Hold attitude (identity,
    // zero rates) when the commanded delta-V is non-finite or below the minimum norm threshold.
    const bool isDvInrtlCmdValid = dvInrtlCmd.allFinite() && dvInrtlCmd.squaredNorm() >= kMinNormSq;

    // base burn frame Bub: 1st axis along dvHat_N, 2nd axis perpendicular to {dvHat_N, dvRotVecUnit},
    // 3rd axis completes the right-handed triad. The DCM rows are the Bub axes in N coordinates.
    const Eigen::Vector3f dvHat_N = dvInrtlCmd.stableNormalized();

    // Guard: when dvRotVecUnit is non-finite, (anti)parallel to dvHat_N, or itself near-zero, the
    // cross product is undefined or collapses, so the base frame is ill-defined and FP32-noise-
    // dominated. Hold attitude rather than emit noise/NaN. The negated form also rejects the NaN a
    // zero-axis stableNormalized() produces (NaN >= kMinCrossSq is false).
    const Eigen::Vector3f cross = dvRotVecUnit.stableNormalized().cross(dvHat_N);
    const bool isCrossValid = dvRotVecUnit.allFinite() && cross.squaredNorm() >= kMinCrossSq;

    DvGuidanceOutput out{};

    if (isDvInrtlCmdValid && isCrossValid) {
        Eigen::Matrix3f dcm_BubN;
        dcm_BubN.row(0) = dvHat_N;
        dcm_BubN.row(1) = cross.normalized();
        dcm_BubN.row(2) = dcm_BubN.row(0).cross(dcm_BubN.row(1)).normalized();

        const float burnTime =
            static_cast<float>(static_cast<int64_t>(callTime) - static_cast<int64_t>(burnStartTime)) * kNano2SecF;

        // current burn frame Bu = base burn frame rotated about its 3rd axis by (dvRotVecMag * burnTime).
        // Below kSmallAngle the rotation is reported as identity: the FP32 noise on prvToDcm * dcm_BubN
        // would otherwise dominate the sub-threshold deviation.
        const float angle = dvRotVecMag * burnTime;
        Eigen::Matrix3f dcm_ButBub = Eigen::Matrix3f::Identity();
        if (fabsf(angle) >= kSmallAngle) {
            dcm_ButBub = prvToDcm(Eigen::Vector3f{0.0F, 0.0F, angle});
        }
        const Eigen::Matrix3f dcm_ButN = dcm_ButBub * dcm_BubN;

        out.sigma_RN = dcmToMrp(dcm_ButN);
        // angular velocity is dvRotVecMag along the 3rd Bu axis, expressed in N
        out.omega_RN_N = dvRotVecMag * dcm_ButN.row(2).transpose();
        out.domega_RN_N = Eigen::Vector3f::Zero();
    }

    return out;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
