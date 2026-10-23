#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <Eigen/Geometry>

namespace {
/// Computes the burn-frame MRP sigma_RN from a validated configuration.
Eigen::Vector3f computeAttitudeReference(const DvGuidanceConfig& config) {
    // Burn frame R: 3rd axis along the commanded delta-V direction, 1st axis perpendicular to {rHatPref_N, r3Hat_N},
    // 2nd axis completes the right-handed triad, i.e. the component of rHatPref_N perpendicular to r3Hat_N.
    // The config guarantees rHatPref_N is not (anti)parallel to r3Hat_N, so the frame is always well defined.
    const Eigen::Vector3f r3Hat_N = config.getCmdDv().stableNormalized();
    const Eigen::Vector3f r1Hat_N = config.getRHatPref().cross(r3Hat_N).stableNormalized();
    const Eigen::Vector3f r2Hat_N = r3Hat_N.cross(r1Hat_N).stableNormalized();

    // The DCM rows are the R axes in N coordinates.
    Eigen::Matrix3f dcm_RN;
    dcm_RN.row(0) = r1Hat_N;
    dcm_RN.row(1) = r2Hat_N;
    dcm_RN.row(2) = r3Hat_N;
    return dcmToMrp(dcm_RN);
}
}  // namespace

DvGuidanceAlgorithm::DvGuidanceAlgorithm(const DvGuidanceConfig& config) : cfg(config) { setConfig(config); }

void DvGuidanceAlgorithm::setConfig(const DvGuidanceConfig& config) {
    this->cfg = config;                                 // copy of the config: cmdDv_N and rHatPref_N
    this->sigma_RN = computeAttitudeReference(config);  // the computed burn-frame MRP
}

Eigen::Vector3f DvGuidanceAlgorithm::update() const { return this->sigma_RN; }
