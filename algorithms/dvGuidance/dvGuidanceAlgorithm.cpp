#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <Eigen/Geometry>

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// bugprone-easily-swappable-parameters: the two Vector3f inputs are documented in the header.
DvGuidanceOutput DvGuidanceAlgorithm::update(const Eigen::Vector3f& dvInrtlCmd, const Eigen::Vector3f& dvRotVecUnit) {
    // r3Hat_N: unit vector along the commanded delta-V direction (the burn frame's 3rd axis).
    const Eigen::Vector3f r3Hat_N = dvInrtlCmd.stableNormalized();

    // Construct the cross product used to detect degenerate or poorly conditioned burn-frame
    // geometry when the preferred 2nd-axis direction is near parallel or antiparallel to r3Hat_N.
    const Eigen::Vector3f cross = dvRotVecUnit.stableNormalized().cross(r3Hat_N);

    const bool isDvInrtlCmdValid = dvInrtlCmd.squaredNorm() >= kMinNormSq;
    const bool isCrossValid = cross.squaredNorm() >= kMinCrossSq;

    DvGuidanceOutput out{};

    if (isDvInrtlCmdValid && isCrossValid) {
        // Burn frame R: 3rd axis along r3Hat_N, 1st axis perpendicular to {preferred 2nd axis, r3Hat_N},
        // 2nd axis completes the right-handed triad, i.e. the preferred direction's component
        // perpendicular to r3Hat_N.
        const Eigen::Vector3f r1Hat_N = cross.stableNormalized();
        const Eigen::Vector3f r2Hat_N = r3Hat_N.cross(r1Hat_N).stableNormalized();

        // The DCM rows are the R axes in N coordinates.
        Eigen::Matrix3f dcm_RN;
        dcm_RN.row(0) = r1Hat_N;
        dcm_RN.row(1) = r2Hat_N;
        dcm_RN.row(2) = r3Hat_N;

        // The burn frame is inertially fixed: the reference rate and acceleration stay zero.
        out.sigma_RN = dcmToMrp(dcm_RN);
    }

    return out;
}
// NOLINTEND(bugprone-easily-swappable-parameters)
