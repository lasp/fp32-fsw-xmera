#include "sunlineEphemAlgorithm.h"

#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>

SunlineEphemAlgorithm::SunlineEphemAlgorithm(const SunlineEphemConfig& config) : cfg(config) { setConfig(config); }

void SunlineEphemAlgorithm::setConfig(const SunlineEphemConfig& config) { this->cfg = config; }

// NOLINTBEGIN(readability-convert-member-functions-to-static, bugprone-easily-swappable-parameters)
// readability-convert-member-functions-to-static: SunlineEphemConfig is intentionally empty for this
// algorithm; the cfg member is held for API consistency with the standard two-phase-init pattern.
// bugprone-easily-swappable-parameters: the Vector3d sun/body positions are documented in the header.
Eigen::Vector3f SunlineEphemAlgorithm::update(const Eigen::Vector3d& r_SN_N,
                                              const Eigen::Vector3d& r_BN_N,
                                              const Eigen::Vector3f& sigma_BN) const {
    // Compute sun-to-body relative position in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;

    // Normalize, rotate into body frame, and re-normalize (or zero if colocated)
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_B = (dcm_BN * r_SB_N.stableNormalized().cast<float>()).stableNormalized();

    return rHat_SB_B;
}
// NOLINTEND(readability-convert-member-functions-to-static, bugprone-easily-swappable-parameters)
