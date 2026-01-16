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

#include "rwMotorTorqueAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <Eigen/LU>
#include <stdint.h>
#include "../freestandingInvalidArgument.h"

/*! This method configures the module by populating any necessary class members.
 @return void
 @param rwParamsInMsg struct to store message containing RW config parameters
 @param rwAvailIsLinked boolean indicating whether RWAvailabilityMsg is linked
 */
void RwMotorTorqueAlgorithm::configure(RWArrayConfigMsgF32Payload& rwParamsInMsg, bool rwAvailIsLinked) {
    /*!- configure the number of axes that are controlled.
     This is determined by checking for a zero row to determinate search */
    this->numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (this->controlAxes_B.row(i).norm() > 0.0) {
            if (this->numControlAxes < i) {
                FS_THROW_INVALID_ARGUMENT(
                    "rwMotorTorque: found empty control axis. "
                    "Make sure to fill controlAxes matrix from top to bottom, "
                    "with zero axes (no control) at the bottom.");
            }
            this->numControlAxes += 1U;
        }
    }
    if (this->numControlAxes == 0U) {
        FS_THROW_INVALID_ARGUMENT("rwMotorTorque is not setup to control any axes.");
    }

    /*! - Read static RW config data message and store it in module variables */
    this->rwConfigParams = rwParamsInMsg;

    /*! - If no info is provided about RW availability we'll assume that all are available
     and create the [Gs] projection matrix once */
    if (!rwAvailIsLinked) {
        this->numAvailRW = static_cast<uint32_t>(this->rwConfigParams.numRW);
        this->G_s_B.leftCols(this->numAvailRW) = cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B).leftCols(this->numAvailRW);
    }
}

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return RwMotorTorqueMsgF32Payload
 @param LrInputMsg commanded torque on spacecraft
 @param LrInput2Msg second commanded torque on spacecraft
 @param wheelsAvailability availability of reaction wheels
 @param cmdTorque2IsLinked boolean indicating whether a second CmdTorqueBodyMsg is linked
 @param rwAvailIsLinked boolean indicating whether RWAvailabilityMsg is linked
 */
RwMotorTorqueMsgF32Payload RwMotorTorqueAlgorithm::update(CmdTorqueBodyMsgF32Payload& LrInputMsg,
                                                       CmdTorqueBodyMsgF32Payload& LrInput2Msg,
                                                       RWAvailabilityMsgPayload& wheelsAvailability,
                                                       bool cmdTorque2IsLinked,
                                                       bool rwAvailIsLinked) {
    // wheelAvailability set to 0 (AVAILABLE) by default

    /*! - zero control torque and RW motor torque variables */
    Eigen::Vector<float, RW_EFF_CNT> us = Eigen::Vector<float, RW_EFF_CNT>::Zero();

    Eigen::Vector3f Lr_B = cArrayToEigenVector(LrInputMsg.torqueRequestBody);

    /*! - Check if the optional second message is provided */
    if (cmdTorque2IsLinked) {
        Lr_B += cArrayToEigenVector(LrInput2Msg.torqueRequestBody);
    }

    /*! - Check if RW availability message is available */
    if (rwAvailIsLinked) {
        uint32_t numAvailWheels = 0U;
        this->G_s_B.setZero();

        /*! - create the current [Gs] projection matrix with the available RWs */
        for (Eigen::Index i = 0; i < this->rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
                this->G_s_B.col(numAvailWheels) = cArrayToEigenVector3(&this->rwConfigParams.GsMatrix_B[i * 3]);
                numAvailWheels += 1U;
            }
        }
        /*! - update the number of currently available RWs */
        this->numAvailRW = numAvailWheels;
    }

    Eigen::Matrix<float, 3, RW_EFF_CNT> CGs = this->controlAxes_B * this->G_s_B;
    const Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(CGs);
    auto rank = static_cast<uint32_t>(lu_decomp.rank());

    /*! - Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [Lr_C]
     Having at least the same # of RW as # of control axes is necessary condition to guarantee inverse matrix exists. If
     matrix to invert it not full rank, the control torque output is zero. */
    if (rank >= this->numControlAxes) {
        const uint32_t numRows = this->numControlAxes;
        const uint32_t numCols = this->numAvailRW;

        Eigen::Vector3f Lr_C{Eigen::Vector3f::Zero()};
        Lr_C.head(numRows) = -this->controlAxes_B.topRows(numRows) * Lr_B;

        Eigen::Vector<float, RW_EFF_CNT> us_avail{Eigen::Vector<float, RW_EFF_CNT>::Zero()};
        us_avail.topRows(numCols) =
            CGs.topLeftCorner(numRows, numCols).transpose() *
            (CGs.topLeftCorner(numRows, numCols) * CGs.topLeftCorner(numRows, numCols).transpose()).inverse() *
            Lr_C.topRows(numRows);

        /*! - map the desired RW motor torques to the available RWs */
        Eigen::Index j = 0;
        for (Eigen::Index i = 0; i < this->rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
                us[i] = us_avail[j];
                j += 1;
            }
        }
    }

    RwMotorTorqueMsgF32Payload rwMotorTorques{};
    eigenVectorToCArray(us, rwMotorTorques.motorTorque);

    return rwMotorTorques;
}

/*! Setter method for the control axes mapping matrix CB, where each row includes the transpose of a control axis.
 The matrix needs to be 3x3, so if only 2 axes are controlled, the third row should be all zeros.
 @return void
 @param controlMappingMatrix Known external torque expressed in body frame components
*/
void RwMotorTorqueAlgorithm::setControlAxes(const Eigen::Matrix3f& controlMappingMatrix) {
    this->controlAxes_B = controlMappingMatrix;
}

/*! Getter method for the control axes mapping matrix CB.
 @return const Eigen::Matrix3f
*/
Eigen::Matrix3f RwMotorTorqueAlgorithm::getControlAxes() const { return this->controlAxes_B; }
