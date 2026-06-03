#include "forceTorqueThrForceMappingAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <limits>
#include <optional>

namespace {

/*! Truncated-SVD pseudo-inverse of the control mapping matrix DG (singular values below
 *  sigma_max * eps * max(m,n) are dropped). Returns nullopt when a desiredControlAxes_B axis is
 *  uncontrollable or the kept subspace is ill-conditioned (condition number > 100).
 */
std::optional<Eigen::Matrix<float, MAX_EFF_CNT, 6>> computeThrusterMapping(
    const ThrusterArrayConfiguration& thrusters,
    const Eigen::Vector3f& centerOfMass_B,
    const std::array<bool, 6>& desiredControlAxes_B) {
    const uint32_t numThrusters = thrusters.numThrusters;

    // Column-major moment arms (r - CoM) and unit thrust directions.
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThruster_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    Eigen::Matrix<float, 3, MAX_EFF_CNT> gtThruster_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < numThrusters; ++i) {
        rThruster_B.col(i) = Eigen::Vector3f(thrusters.thrusters.at(i).rThrust_B.data());
        gtThruster_B.col(i) = Eigen::Vector3f(thrusters.thrusters.at(i).tHatThrust_B.data()).normalized();
    }
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThrusterRelCOM_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    rThrusterRelCOM_B.leftCols(numThrusters) = rThruster_B.leftCols(numThrusters).colwise() - centerOfMass_B;

    // DG: moment arms (rows 0-2), thrust directions (rows 3-5).
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rCrossGt{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < numThrusters; ++i) {
        rCrossGt.col(i) = rThrusterRelCOM_B.col(i).cross(gtThruster_B.col(i));
    }
    Eigen::Matrix<float, 6, MAX_EFF_CNT> DGwithZeros{};
    DGwithZeros << rCrossGt, gtThruster_B;

    const Eigen::JacobiSVD<Eigen::Matrix<float, 6, MAX_EFF_CNT>> svd(DGwithZeros,
                                                                     Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector<float, 6>& sv = svd.singularValues();
    constexpr int kMaxDim = (6 > MAX_EFF_CNT) ? 6 : MAX_EFF_CNT;
    const float tol = sv(0) * std::numeric_limits<float>::epsilon() * static_cast<float>(kMaxDim);

    Eigen::Vector<float, 6> invSv = Eigen::Vector<float, 6>::Zero();
    for (int i = 0; i < 6; ++i) {
        if (sv(i) > tol) {
            invSv(i) = 1.0F / sv(i);
        }
    }

    // Controllability: an asserted axis projecting onto the truncated (uncontrollable) left singular vectors
    // is not reachable.
    constexpr float kControllabilityResidualSqTol = 1e-6F;
    const Eigen::Matrix<float, 6, 6>& U = svd.matrixU();
    for (int axis = 0; axis < 6; ++axis) {
        if (!desiredControlAxes_B.at(static_cast<std::size_t>(axis))) {
            continue;
        }
        float residualSq = 0.0F;
        for (int k = 0; k < 6; ++k) {
            if (sv(k) <= tol) {
                residualSq += U(axis, k) * U(axis, k);
            }
        }
        if (residualSq > kControllabilityResidualSqTol) {
            return std::nullopt;
        }
    }

    // Conditioning: reject when the smallest kept singular value (the last above tol, sorted descending)
    // drops below kConditioningTol of the largest, i.e. condition number > 100.
    constexpr float kConditioningTol = 1e-2F;
    float minKept = sv(0);
    for (int i = 0; i < 6; ++i) {
        if (sv(i) > tol) {
            minKept = sv(i);
        }
    }
    if (minKept <= sv(0) * kConditioningTol) {
        return std::nullopt;
    }

    Eigen::Matrix<float, MAX_EFF_CNT, 6> pseudoInverseDG{Eigen::Matrix<float, MAX_EFF_CNT, 6>::Zero()};
    pseudoInverseDG.noalias() = svd.matrixV().leftCols<6>() * invSv.asDiagonal() * svd.matrixU().transpose();

    // Clear trailing rows (zero in exact arithmetic) so the padding-is-zero contract holds bitwise.
    if (numThrusters < MAX_EFF_CNT) {
        pseudoInverseDG.bottomRows(MAX_EFF_CNT - numThrusters).setZero();
    }

    return pseudoInverseDG;
}

}  // namespace

bool ForceTorqueThrForceMappingConfig::isValidMapping(const ThrusterArrayConfiguration& thrusters,
                                                      const Eigen::Vector3f& centerOfMass_B,
                                                      const std::array<bool, 6>& desiredControlAxes_B) {
    return computeThrusterMapping(thrusters, centerOfMass_B, desiredControlAxes_B).has_value();
}

// create() already rejected configs that do not yield a valid mapping, so computeThrusterMapping() returns a
// value here; the guard avoids a throw in the impossible nullopt case.
// Config is fixed-size/trivially copyable, so move == copy; pass-by-value would only add an extra copy.
ForceTorqueThrForceMappingAlgorithm::ForceTorqueThrForceMappingAlgorithm(
    const ForceTorqueThrForceMappingConfig& config)  // NOLINT(modernize-pass-by-value)
    : cfg(config) {
    const std::optional<Eigen::Matrix<float, MAX_EFF_CNT, 6>> mapping = computeThrusterMapping(
        this->cfg.getThrusters(), this->cfg.getCenterOfMass_B(), this->cfg.getDesiredControlAxes());
    if (mapping.has_value()) {
        this->pseudoInverseDG = *mapping;
    }
}

//! Replace the configuration and recompute the thruster mapping matrix.
void ForceTorqueThrForceMappingAlgorithm::setConfig(const ForceTorqueThrForceMappingConfig& config) {
    this->cfg = config;
    const std::optional<Eigen::Matrix<float, MAX_EFF_CNT, 6>> mapping = computeThrusterMapping(
        this->cfg.getThrusters(), this->cfg.getCenterOfMass_B(), this->cfg.getDesiredControlAxes());
    if (mapping.has_value()) {
        this->pseudoInverseDG = *mapping;
    }
}

/*! Map the requested body torque and force to per-thruster forces (non-negative, shifted by their minimum).
 @param cmdTorque_B [Nm] requested control torque in body frame
 @param cmdForce_B [N] requested control force in body frame
 @return per-thruster force commands [N]
*/
Eigen::Vector<float, MAX_EFF_CNT> ForceTorqueThrForceMappingAlgorithm::update(const Eigen::Vector3f& cmdTorque_B,
                                                                              const Eigen::Vector3f& cmdForce_B) const {
    Eigen::Vector<float, 6> forceTorque_B{};
    forceTorque_B << cmdTorque_B, cmdForce_B;

    const uint32_t numThrusters = this->cfg.getThrusters().numThrusters;
    Eigen::Vector<float, MAX_EFF_CNT> thrusterForces = this->pseudoInverseDG * forceTorque_B;
    const float minForce = thrusterForces.head(numThrusters).minCoeff();
    thrusterForces.head(numThrusters).array() -= minForce;

    return thrusterForces;
}
