#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <Eigen/Geometry>

DvGuidanceAlgorithm::DvGuidanceAlgorithm(const DvGuidanceConfig& config) : cfg(config) { setConfig(config); }

void DvGuidanceAlgorithm::setConfig(const DvGuidanceConfig& config) { this->cfg = config; }

DvGuidanceOutput DvGuidanceAlgorithm::update() const {
    // Burn frame R: 3rd axis along the commanded delta-V direction, 1st axis perpendicular to {rHatPref_N, r3Hat_N},
    // 2nd axis completes the right-handed triad, i.e. the component of rHatPref_N perpendicular to r3Hat_N.
    // The config guarantees rHatPref_N is not (anti)parallel to r3Hat_N, so the frame is always well defined.
    const Eigen::Vector3f r3Hat_N = this->cfg.getCmdDv().stableNormalized();
    const Eigen::Vector3f r1Hat_N = this->cfg.getRHatPref().cross(r3Hat_N).stableNormalized();
    const Eigen::Vector3f r2Hat_N = r3Hat_N.cross(r1Hat_N).stableNormalized();

    // The DCM rows are the R axes in N coordinates.
    Eigen::Matrix3f dcm_RN;
    dcm_RN.row(0) = r1Hat_N;
    dcm_RN.row(1) = r2Hat_N;
    dcm_RN.row(2) = r3Hat_N;

    // The burn frame is inertially fixed: the reference rate and acceleration stay zero.
    DvGuidanceOutput out{};
    out.sigma_RN = dcmToMrp(dcm_RN);
    return out;
}
