#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <Eigen/Geometry>

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// bugprone-easily-swappable-parameters: the two Vector3f inputs are documented in the header.
DvGuidanceOutput DvGuidanceAlgorithm::update(const Eigen::Vector3f& dvInrtlCmd, const Eigen::Vector3f& dvRotVecUnit) {
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
        dcm_BubN.row(1) = cross.stableNormalized();
        dcm_BubN.row(2) = dcm_BubN.row(0).cross(dcm_BubN.row(1)).stableNormalized();

        // The burn frame is inertially fixed: the reference rate and acceleration stay zero.
        out.sigma_RN = dcmToMrp(dcm_BubN);
    }

    return out;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
