#include "forceTorqueThrForceMappingAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <limits>

/*! Construct the algorithm from a validated configuration and immediately compute the thruster
 *  mapping matrix.
 @param config validated configuration (thruster array, center of mass, controllability assertions)
*/
// Config is fixed-size/trivially copyable, so move == copy; pass-by-value would only add an extra copy.
ForceTorqueThrForceMappingAlgorithm::ForceTorqueThrForceMappingAlgorithm(
    const ForceTorqueThrForceMappingConfig& config)  // NOLINT(modernize-pass-by-value)
    : cfg(config) {
    this->computeThrusterMapping();
}

/*! Replace the configuration at runtime and recompute the thruster mapping matrix.
 @param config validated configuration (thruster array, center of mass, controllability assertions)
*/
void ForceTorqueThrForceMappingAlgorithm::setConfig(const ForceTorqueThrForceMappingConfig& config) {
    this->cfg = config;
    this->computeThrusterMapping();
}

/*! Compute thruster force commands from the requested torque and force vectors.
 @return Eigen::Vector<float, MAX_EFF_CNT> thruster force commands (non-negative, shifted by min)
 @param cmdTorque_B [Nm] requested control torque in body frame
 @param cmdForce_B [N] requested control force in body frame
*/
Eigen::Vector<float, MAX_EFF_CNT> ForceTorqueThrForceMappingAlgorithm::update(const Eigen::Vector3f& cmdTorque_B,
                                                                              const Eigen::Vector3f& cmdForce_B) const {
    /* Create the torque and force vector */
    Eigen::Vector<float, 6> forceTorque_B{};
    forceTorque_B << cmdTorque_B, cmdForce_B;

    const uint32_t numThrusters = this->cfg.getThrusters().numThrusters;
    Eigen::Vector<float, MAX_EFF_CNT> thrusterForces = this->pseudoInverseDG * forceTorque_B;
    const float minForce = thrusterForces.head(numThrusters).minCoeff();
    thrusterForces.head(numThrusters).array() -= minForce;

    return thrusterForces;
}

/*! Compute the thruster mapping matrix via a truncated-SVD pseudo-inverse of the control mapping
 *  matrix DG. Singular values below a relative tolerance (sigma_max * eps * max(m,n)) are treated
 *  as zero, which projects uncontrollable directions in the 6-D command space out of the result.
 *  Throws if any axis flagged in the configuration's desiredControlAxes_B lies outside the column space of DG.
 */
void ForceTorqueThrForceMappingAlgorithm::computeThrusterMapping() {
    const ThrusterArrayConfiguration& thrusters = this->cfg.getThrusters();
    const uint32_t numThrusters = thrusters.numThrusters;
    const Eigen::Vector3f centerOfMass_B = this->cfg.getCenterOfMass_B();
    const std::array<bool, 6>& desiredControlAxes_B = this->cfg.getDesiredControlAxes();

    /* Decompose the struct-of-arrays thruster layout into column-major matrices; directions are
       normalized so DG rows 3-5 are exactly unit vectors. */
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThruster_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    Eigen::Matrix<float, 3, MAX_EFF_CNT> gtThruster_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < numThrusters; ++i) {
        rThruster_B.col(i) = Eigen::Vector3f(thrusters.thrusters.at(i).rThrust_B.data());
        gtThruster_B.col(i) = Eigen::Vector3f(thrusters.thrusters.at(i).tHatThrust_B.data()).normalized();
    }

    /* - compute thruster locations relative to COM */
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThrusterRelCOM_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    rThrusterRelCOM_B.leftCols(numThrusters) = rThruster_B.leftCols(numThrusters).colwise() - centerOfMass_B;

    /* Fill DG with moment arms (rows 0-2) and thruster directions (rows 3-5) */
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rCrossGt{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < numThrusters; ++i) {
        rCrossGt.col(i) = rThrusterRelCOM_B.col(i).cross(gtThruster_B.col(i));
    }
    Eigen::Matrix<float, 6, MAX_EFF_CNT> DGwithZeros{};
    DGwithZeros << rCrossGt, gtThruster_B;

    // SVD on the full DG (trailing columns are zero for unused thruster slots).
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

    // Cross-check desiredControlAxes_B against the SVD
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
            FSW_THROW_INVALID_ARGUMENT(
                "forceTorqueThrForceMapping: an axis marked in desiredControlAxes_B is not "
                "controllable by the configured thruster array");
        }
    }

    this->pseudoInverseDG.noalias() = svd.matrixV().leftCols<6>() * invSv.asDiagonal() * svd.matrixU().transpose();

    // Trailing rows are zero in exact arithmetic but may carry fp32 noise. Clear them so the
    // padding-is-zero contract holds bitwise.
    if (numThrusters < MAX_EFF_CNT) {
        this->pseudoInverseDG.bottomRows(MAX_EFF_CNT - numThrusters).setZero();
    }
}
