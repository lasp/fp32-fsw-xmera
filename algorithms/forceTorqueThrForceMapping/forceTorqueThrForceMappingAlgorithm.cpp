#include "forceTorqueThrForceMappingAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <limits>

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

    Eigen::Vector<float, MAX_EFF_CNT> thrusterForces = this->pseudoInverseDG * forceTorque_B;
    const float minForce = thrusterForces.head(this->numThrusters).minCoeff();
    thrusterForces.head(this->numThrusters).array() -= minForce;

    return thrusterForces;
}

/*! Compute the thruster mapping matrix via a truncated-SVD pseudo-inverse of DG. Singular values
 *  below a relative tolerance (sigma_max * eps * max(m,n)) are treated as zero, which projects
 *  uncontrollable directions in the 6-D command space out of the result. Throws if any axis flagged
 *  in desiredControlAxes_B lies outside the column space of DG.
 */
void ForceTorqueThrForceMappingAlgorithm::computeThrusterMapping() {
    /* - compute thruster locations relative to COM */
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThrusterRelCOM_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    rThrusterRelCOM_B.leftCols(this->numThrusters) =
        this->rThruster_B.leftCols(this->numThrusters).colwise() - this->CoM_B;

    /* Fill DG with moment arms (rows 0-2) and thruster directions (rows 3-5) */
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rCrossGt{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        rCrossGt.col(i) = rThrusterRelCOM_B.col(i).cross(this->gtThruster_B.col(i));
    }
    Eigen::Matrix<float, 6, MAX_EFF_CNT> DGwithZeros{};
    DGwithZeros << rCrossGt, this->gtThruster_B;

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
        if (!this->desiredControlAxes_B.at(static_cast<std::size_t>(axis))) {
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
    if (this->numThrusters < MAX_EFF_CNT) {
        this->pseudoInverseDG.bottomRows(MAX_EFF_CNT - this->numThrusters).setZero();
    }
}

/*! Setter for the thruster array configuration. Validates the thruster count and that each active
 *  direction vector has a norm within 1e-3 of 1.0, then decomposes the struct-of-arrays layout into
 *  the internal column-major Eigen matrices.
 @return void
 @param thrusterConfig thruster array configuration (positions and unit direction vectors)
*/
void ForceTorqueThrForceMappingAlgorithm::setThrusters(const ThrusterArrayConfig& thrusterConfig) {
    const uint32_t count = thrusterConfig.numThrusters;
    if (count == 0 || count > MAX_EFF_CNT) {
        FSW_THROW_INVALID_ARGUMENT("forceTorqueThrForceMapping: numThrusters must be in [1, MAX_EFF_CNT]");
    }

    constexpr float normTolerance = 1e-3F;
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThruster_B_new{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    Eigen::Matrix<float, 3, MAX_EFF_CNT> gtThruster_B_new{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < count; ++i) {
        rThruster_B_new.col(i) = Eigen::Vector3f(thrusterConfig.thrusters.at(i).rThrust_B.data());
        Eigen::Vector3f direction(thrusterConfig.thrusters.at(i).tHatThrust_B.data());
        if (fabsf(direction.stableNorm() - 1.0F) > normTolerance) {
            FSW_THROW_INVALID_ARGUMENT("forceTorqueThrForceMapping: thruster direction vector must be a unit vector");
        }
        direction.normalize();
        gtThruster_B_new.col(i) = direction;
    }

    this->rThruster_B = rThruster_B_new;
    this->gtThruster_B = gtThruster_B_new;
    this->numThrusters = count;
}

/*! Getter for the thruster array configuration. Reconstructs the struct-of-arrays layout from the
 *  internal column-major Eigen matrices. Direction vectors are returned normalized.
 @return ThrusterArrayConfig
*/
ThrusterArrayConfig ForceTorqueThrForceMappingAlgorithm::getThrusters() const {
    ThrusterArrayConfig thrusterConfig{};
    thrusterConfig.numThrusters = this->numThrusters;
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            thrusterConfig.thrusters.at(i).rThrust_B.at(j) = this->rThruster_B(j, i);
            thrusterConfig.thrusters.at(i).tHatThrust_B.at(j) = this->gtThruster_B(j, i);
        }
    }
    return thrusterConfig;
}

/*! Setter for the spacecraft center of mass in body frame.
 @return void
 @param centerOfMass [m] center of mass in body frame
*/
void ForceTorqueThrForceMappingAlgorithm::setCoM_B(const Eigen::Vector3f& centerOfMass) { this->CoM_B = centerOfMass; }

/*! Getter for the spacecraft center of mass in body frame.
 @return Eigen::Vector3f [m]
*/
Eigen::Vector3f ForceTorqueThrForceMappingAlgorithm::getCoM_B() const { return this->CoM_B; }

/*! Setter for the desiredControlAxes_B assertion vector. The first three entries are the torque
 *  components xyz in body frame B; the last three are the force components xyz in body frame B. A
 *  true entry asserts that the corresponding axis must be controllable by the configured thruster
 *  array; the assertion is checked against the SVD of DG inside computeThrusterMapping().
 @return void
 @param desiredControlAxes per-axis controllability assertions
*/
void ForceTorqueThrForceMappingAlgorithm::setDesiredControlAxes(const std::array<bool, 6>& desiredControlAxes) {
    this->desiredControlAxes_B = desiredControlAxes;
}

/*! Getter for the desiredControlAxes_B assertion vector.
 @return std::array<bool, 6>
*/
std::array<bool, 6> ForceTorqueThrForceMappingAlgorithm::getDesiredControlAxes() const {
    return this->desiredControlAxes_B;
}
