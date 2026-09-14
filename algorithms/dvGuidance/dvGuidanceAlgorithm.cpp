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
    // dvHat_N: unit vector along the commanded delta-V direction (the burn frame's 1st axis).
    const Eigen::Vector3f dvHat_N = dvInrtlCmd.stableNormalized();

    // Construct the cross product used to detect degenerate or poorly conditioned
    // burn-frame geometry when the rotation-axis seed is near parallel or antiparallel to dvHat_N.
    const Eigen::Vector3f cross = dvRotVecUnit.stableNormalized().cross(dvHat_N);

    const bool isDvInrtlCmdValid = dvInrtlCmd.squaredNorm() >= kMinNormSq;
    const bool isCrossValid = cross.squaredNorm() >= kMinCrossSq;

    DvGuidanceOutput out{};

    if (isDvInrtlCmdValid && isCrossValid) {
        // Base burn frame Bub: 1st axis along dvHat_N, 2nd axis perpendicular to {dvHat_N, dvRotVecUnit},
        // 3rd axis completes the right-handed triad. The DCM rows are the Bub axes in N coordinates.
        Eigen::Matrix3f dcm_BubN;
        dcm_BubN.row(0) = dvHat_N;
        dcm_BubN.row(1) = cross.normalized();
        dcm_BubN.row(2) = dcm_BubN.row(0).cross(dcm_BubN.row(1)).normalized();

        const float burnTime =
            static_cast<float>(static_cast<int64_t>(callTime) - static_cast<int64_t>(burnStartTime)) * kNano2SecF;

        // Current burn frame = base burn frame rotated about its 3rd axis by
        // dvRotVecMag * burnTime. Rotations below kSmallAngle are treated as zero.
        const float angle = dvRotVecMag * burnTime;
        Eigen::Matrix3f dcm_ButBub = Eigen::Matrix3f::Identity();
        if (fabsf(angle) >= kSmallAngle) {
            dcm_ButBub = prvToDcm(Eigen::Vector3f{0.0F, 0.0F, angle});
        }
        const Eigen::Matrix3f dcm_ButN = dcm_ButBub * dcm_BubN;

        out.sigma_RN = dcmToMrp(dcm_ButN);
        // Angular velocity is dvRotVecMag along the 3rd Bu axis, expressed in N
        out.omega_RN_N = dvRotVecMag * dcm_ButN.row(2).transpose();
        out.domega_RN_N = Eigen::Vector3f::Zero();
    }

    return out;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
