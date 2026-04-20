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

/*! Store the thruster configuration for subsequent update() calls.
 @return void
 @param numThrusters number of active thrusters
 @param CoM_B [m] center of mass in body frame
 @param rThruster_B [m] thruster positions in body frame (one column per thruster)
 @param gtThruster_B thruster force unit direction vectors (one column per thruster)
*/
void ForceTorqueThrForceMappingAlgorithm::reset(const uint32_t numThrusters,
                                                const Eigen::Vector3f& CoM_B,
                                                const Eigen::Matrix<float, 3, MAX_EFF_CNT>& rThruster_B,
                                                const Eigen::Matrix<float, 3, MAX_EFF_CNT>& gtThruster_B) {
    if (numThrusters > MAX_EFF_CNT) {
        FSW_THROW_INVALID_ARGUMENT(
            "forceTorqueThrForceMapping: numThrusters is larger than MAX_EFF_CNT");
    }
    this->numThrusters = numThrusters;
    this->CoM_B = CoM_B;
    this->rThruster_B = rThruster_B;
    this->gtThruster_B = gtThruster_B;
}

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
