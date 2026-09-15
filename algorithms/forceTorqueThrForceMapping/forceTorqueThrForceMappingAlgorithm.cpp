#include "forceTorqueThrForceMappingAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <limits>
#include <optional>

namespace {

//! Cached pseudo-inverse, and the direction update() shifts along to remove negative thrust.
struct ThrusterMapping {
    Eigen::Matrix<float, kMaxThrusterCount, 6> pseudoInverseDG;
    Eigen::Vector<float, kMaxThrusterCount> nullSpaceShift;
};

/*! The part of the all-ones vector that lies in the null space of the kept row space of DG: one minus
 *  its projection onto the right singular vectors of the kept singular values. Taken one rank-1
 *  projection at a time -- a matrix of the kept vectors would need dynamically sized storage.
 */
Eigen::Vector<float, kMaxThrusterCount> computeNullSpaceShift(
    const Eigen::Matrix<float, kMaxThrusterCount, kMaxThrusterCount>& rightSingularVectors,
    uint32_t numThrusters,
    const Eigen::Vector<float, 6>& sv,
    float tol) {
    Eigen::Vector<float, kMaxThrusterCount> ones{Eigen::Vector<float, kMaxThrusterCount>::Zero()};
    ones.head(numThrusters).setOnes();

    Eigen::Vector<float, kMaxThrusterCount> nullSpaceShift = ones;
    for (int k = 0; k < 6; ++k) {
        if (sv(k) > tol) {
            const Eigen::Vector<float, kMaxThrusterCount> rowSpaceDirection = rightSingularVectors.col(k);
            nullSpaceShift -= rowSpaceDirection.dot(ones) * rowSpaceDirection;
        }
    }
    if (numThrusters < kMaxThrusterCount) {
        nullSpaceShift.tail(kMaxThrusterCount - numThrusters).setZero();
    }
    return nullSpaceShift;
}

/*! Truncated-SVD pseudo-inverse of the control mapping matrix DG (singular values below
 *  sigma_max * eps * max(m,n) are dropped). Only the rows selected by desiredControlAxes_B take part:
 *  the others are zeroed, which removes them from the solve without changing the fixed matrix shape.
 *  Also returns the shift direction: the part of the all-ones vector lying in the null space of those
 *  rows. Returns nullopt when a selected axis is uncontrollable or the kept subspace is
 *  ill-conditioned (condition number > 100).
 */
std::optional<ThrusterMapping> computeThrusterMapping(const ThrusterArrayConfiguration& thrusters,
                                                      const Eigen::Vector3f& centerOfMass_B,
                                                      const std::array<bool, 6>& desiredControlAxes_B) {
    const uint32_t numThrusters = thrusters.numThrusters;

    // Column-major moment arms (r - CoM) and unit thrust directions.
    Eigen::Matrix<float, 3, kMaxThrusterCount> r_TB_B{Eigen::Matrix<float, 3, kMaxThrusterCount>::Zero()};
    Eigen::Matrix<float, 3, kMaxThrusterCount> tHat_B{Eigen::Matrix<float, 3, kMaxThrusterCount>::Zero()};
    for (uint32_t i = 0; i < numThrusters; ++i) {
        r_TB_B.col(i) = Eigen::Vector3f(thrusters.thrusters.at(i).r_TB_B.data());
        tHat_B.col(i) = Eigen::Vector3f(thrusters.thrusters.at(i).tHat_B.data()).normalized();
    }
    Eigen::Matrix<float, 3, kMaxThrusterCount> r_TC_B{Eigen::Matrix<float, 3, kMaxThrusterCount>::Zero()};
    r_TC_B.leftCols(numThrusters) = r_TB_B.leftCols(numThrusters).colwise() - centerOfMass_B;

    // DG: moment arms (rows 0-2), thrust directions (rows 3-5).
    Eigen::Matrix<float, 3, kMaxThrusterCount> torquePntC_B{Eigen::Matrix<float, 3, kMaxThrusterCount>::Zero()};
    for (uint32_t i = 0; i < numThrusters; ++i) {
        torquePntC_B.col(i) = r_TC_B.col(i).cross(tHat_B.col(i));
    }
    Eigen::Matrix<float, 6, kMaxThrusterCount> DGwithZeros{};
    DGwithZeros << torquePntC_B, tHat_B;

    // Remove the axes that the caller does not select. Zeroing a row is equivalent to deleting it: the
    // pseudo-inverse of the reduced matrix reappears as the corresponding columns of the padded one, with
    // zero columns where the rows were dropped. The solve thus applies no condition to an unselected axis,
    // and does not balance such an axis against the selected ones.
    for (int axis = 0; axis < 6; ++axis) {
        if (!desiredControlAxes_B.at(static_cast<std::size_t>(axis))) {
            DGwithZeros.row(axis).setZero();
        }
    }

    const Eigen::JacobiSVD<Eigen::Matrix<float, 6, kMaxThrusterCount>> svd(DGwithZeros,
                                                                           Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector<float, 6>& sv = svd.singularValues();
    constexpr int kMaxDim = (6 > kMaxThrusterCount) ? 6 : kMaxThrusterCount;
    const float tol = sv(0) * std::numeric_limits<float>::epsilon() * static_cast<float>(kMaxDim);

    Eigen::Vector<float, 6> invSv = Eigen::Vector<float, 6>::Zero();
    for (int i = 0; i < 6; ++i) {
        if (sv(i) > tol) {
            invSv(i) = 1.0F / sv(i);
        }
    }

    // Controllability: a selected axis projecting onto the truncated (uncontrollable) left singular vectors
    // is not reachable. Across the selected axes this is exactly the statement that the selected rows of DG
    // have full row rank, so each one can be commanded independently of the others.
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

    // Conditioning: over the selected rows, reject when the smallest kept singular value (the last above tol,
    // sorted descending) drops below kConditioningTol of the largest, i.e. condition number > 100.
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

    Eigen::Matrix<float, kMaxThrusterCount, 6> pseudoInverseDG{Eigen::Matrix<float, kMaxThrusterCount, 6>::Zero()};
    pseudoInverseDG.noalias() = svd.matrixV().leftCols<6>() * invSv.asDiagonal() * svd.matrixU().transpose();

    // Clear trailing rows (zero in exact arithmetic) so the padding-is-zero contract holds bitwise.
    if (numThrusters < kMaxThrusterCount) {
        pseudoInverseDG.bottomRows(kMaxThrusterCount - numThrusters).setZero();
    }

    // Shift direction: lies in the null space of DG, so DG * nullSpaceShift = 0.
    return ThrusterMapping{.pseudoInverseDG = pseudoInverseDG,
                           .nullSpaceShift = computeNullSpaceShift(svd.matrixV(), numThrusters, sv, tol)};
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
    setConfig(config);
}

//! Replace the configuration and recompute the thruster mapping matrix.
void ForceTorqueThrForceMappingAlgorithm::setConfig(const ForceTorqueThrForceMappingConfig& config) {
    this->cfg = config;
    const std::optional<ThrusterMapping> mapping = computeThrusterMapping(
        this->cfg.getThrusters(), this->cfg.getCenterOfMass_B(), this->cfg.getDesiredControlAxes());
    if (mapping.has_value()) {
        this->pseudoInverseDG = mapping->pseudoInverseDG;
        this->nullSpaceShift = mapping->nullSpaceShift;
    }
}

/*! Map the requested body torque and force to per-thruster forces (non-negative).
 *
 * Negative entries in the pseudo-inverse solution are removed by a shift along nullSpaceShift, which
 * leaves the achieved force and torque on every selected axis unchanged. Entries no shift can lift are
 * clamped.
 @param cmdTorque_B [Nm] requested control torque in body frame
 @param cmdForce_B [N] requested control force in body frame
 @return per-thruster force commands [N]
*/
Eigen::Vector<float, kMaxThrusterCount> ForceTorqueThrForceMappingAlgorithm::update(
    const Eigen::Vector3f& cmdTorque_B,
    const Eigen::Vector3f& cmdForce_B) const {
    Eigen::Vector<float, 6> forceTorque_B{};
    forceTorque_B << cmdTorque_B, cmdForce_B;

    const uint32_t numThrusters = this->cfg.getThrusters().numThrusters;
    Eigen::Vector<float, kMaxThrusterCount> thrusterForces = this->pseudoInverseDG * forceTorque_B;

    // Shift along the null space of DG: it leaves the achieved force and torque unchanged. The step is
    // the largest per-entry -F_j / n_j over the entries with n_j > 0, lifting all of them to zero or above at once.
    const float shiftScale = this->nullSpaceShift.head(numThrusters).cwiseAbs().maxCoeff();
    constexpr float kShiftTol = 1e-6F;
    if (shiftScale > kShiftTol) {
        float step = 0.0F;
        for (uint32_t j = 0; j < numThrusters; ++j) {
            if (this->nullSpaceShift(j) > kShiftTol * shiftScale) {
                step = fmaxf(step, -thrusterForces(j) / this->nullSpaceShift(j));
            }
        }
        thrusterForces.head(numThrusters) += step * this->nullSpaceShift.head(numThrusters);
    }

    // The only step that moves the achieved force and torque away from the command.
    for (uint32_t j = 0; j < numThrusters; ++j) {
        thrusterForces(j) = fmaxf(thrusterForces(j), 0.0F);
    }

    return thrusterForces;
}
