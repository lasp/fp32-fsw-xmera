/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#include "forceTorqueThrForceMappingAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

#include <Eigen/Geometry>
#include <Eigen/QR>

/*! Compute thruster force commands from the requested torque and force vectors.
 @return Eigen::Vector<float, MAX_EFF_CNT> thruster force commands (non-negative, shifted by min)
 @param cmdTorque [Nm] requested control torque in body frame
 @param cmdForce [N] requested control force in body frame
*/
Eigen::Vector<float, MAX_EFF_CNT> ForceTorqueThrForceMappingAlgorithm::update(
    const Eigen::Vector3f& cmdTorque, const Eigen::Vector3f& cmdForce) const {
    /* Create the torque and force vector */
    Eigen::Vector<float, 6> forceTorque_B{};
    forceTorque_B.head(3) = cmdTorque;
    forceTorque_B.tail(3) = cmdForce;

    /* - compute thruster locations relative to COM */
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rThrusterRelCOM_B{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    rThrusterRelCOM_B.leftCols(this->numThrusters) =
        this->rThruster_B.leftCols(this->numThrusters).colwise() - this->CoM_B;

    /* Fill DG with thruster directions and moment arms */
    Eigen::Matrix<float, 3, MAX_EFF_CNT> rCrossGt{Eigen::Matrix<float, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        rCrossGt.col(i) = rThrusterRelCOM_B.col(i).cross(this->gtThruster_B.col(i));
    }
    Eigen::Matrix<float, 6, MAX_EFF_CNT> DG{};
    DG << rCrossGt, this->gtThruster_B;

    /* Create the DG w/ zero rows removed */
    uint32_t nonZeroRows = 0;
    Eigen::Matrix<float, 6, MAX_EFF_CNT> DG_nonzero{Eigen::Matrix<float, 6, MAX_EFF_CNT>::Zero()};
    Eigen::Vector<float, 6> forceTorque_B_nonzero{Eigen::Vector<float, 6>::Zero()};
    for (uint32_t i = 0; i < 6; ++i) {
        if ((DG.row(i).array().abs() > 1e-4F).any()) {
            DG_nonzero.row(nonZeroRows) = DG.row(i);
            forceTorque_B_nonzero.row(nonZeroRows) = forceTorque_B.row(i);
            nonZeroRows += 1;
        }
    }

    /* Compute the force for each thruster */
    const uint32_t numRows = nonZeroRows;
    const uint32_t numCols = this->numThrusters;

    Eigen::Vector<float, MAX_EFF_CNT> force_B{Eigen::Vector<float, MAX_EFF_CNT>::Zero()};
    force_B.topRows(numCols) =
        DG_nonzero.topLeftCorner(numRows, numCols).completeOrthogonalDecomposition().pseudoInverse() *
        forceTorque_B_nonzero.topRows(numRows);

    /* Find the minimum force */
    const float minForce = force_B.topRows(this->numThrusters).minCoeff();

    /* Subtract the minimum force */
    Eigen::Vector<float, MAX_EFF_CNT> forceSubtracted_B{Eigen::Vector<float, MAX_EFF_CNT>::Zero()};
    forceSubtracted_B.topRows(this->numThrusters) = force_B.topRows(this->numThrusters).array() - minForce;

    return forceSubtracted_B;
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
