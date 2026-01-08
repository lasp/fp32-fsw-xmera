#ifndef TEST_RW_MOTOR_TORQUE_H
#define TEST_RW_MOTOR_TORQUE_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "rwMotorTorqueAlgorithm.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <numbers>
#include <vector>

// Reference computation for update
RwMotorTorqueMsgF32Payload referenceUpdate(const RwMotorTorqueAlgorithm& alg,
                                           uint32_t numControlAxes,
                                           uint32_t numAvailRW,
                                           RWArrayConfigMsgF32Payload rwConfigParams,
                                           Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B,
                                           CmdTorqueBodyMsgF32Payload& LrInputMsg,
                                           CmdTorqueBodyMsgF32Payload& LrInput2Msg,
                                           RWAvailabilityMsgPayload& wheelsAvailability,
                                           bool cmdTorque2IsLinked,
                                           bool rwAvailIsLinked) {
    Eigen::Matrix3f controlAxes_B = alg.getControlAxes();

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
        G_s_B.setZero();

        /*! - create the current [Gs] projection matrix with the available RWs */
        for (Eigen::Index i = 0; i < rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
                G_s_B.col(numAvailWheels) = cArrayToEigenVector3(&rwConfigParams.GsMatrix_B[i * 3]);
                numAvailWheels += 1U;
            }
        }
        /*! - update the number of currently available RWs */
        numAvailRW = numAvailWheels;
    }

    /*! - Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [Lr_C]
     Having at least the same # of RW as # of control axes is necessary condition to guarantee inverse matrix exists. If
     matrix to invert it not full rank, the control torque output is zero. */
    if (numAvailRW >= numControlAxes) {
        uint32_t numRows = numControlAxes;
        uint32_t numCols = numAvailRW;

        Eigen::Vector3f Lr_C{Eigen::Vector3f::Zero()};
        Lr_C.head(numRows) = -controlAxes_B.topRows(numRows) * Lr_B;

        Eigen::Matrix<float, 3, RW_EFF_CNT> CGs = controlAxes_B * G_s_B;

        Eigen::Vector<float, RW_EFF_CNT> us_avail{Eigen::Vector<float, RW_EFF_CNT>::Zero()};
        us_avail.topRows(numCols) =
            CGs.topLeftCorner(numRows, numCols).transpose() *
            (CGs.topLeftCorner(numRows, numCols) * CGs.topLeftCorner(numRows, numCols).transpose()).inverse() *
            Lr_C.topRows(numRows);

        /*! - map the desired RW motor torques to the available RWs */
        Eigen::Index j = 0;
        for (Eigen::Index i = 0; i < rwConfigParams.numRW; ++i) {
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

inline void testRwMotorTorqueSetup() {
    RwMotorTorqueAlgorithm alg{};

    // --- Test expected exceptions ---

    // control axes matrix not properly set up (control axes not filled from top to bottom before any zero rows)
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    controlAxes_B.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes_B.row(1) = Eigen::Vector3f{0.0F, 0.0F, 0.0F};
    controlAxes_B.row(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    alg.setControlAxes(controlAxes_B);
    RWArrayConfigMsgF32Payload rwParams{};
    EXPECT_THROW(alg.configure(rwParams, false), fs::invalid_argument);
}

inline void testRwMotorTorque(std::vector<float> Lr1_B,
                              std::vector<float> Lr2_B,
                              std::vector<bool> wheelAvailabilityBool,
                              bool cmdTorque2IsLinked,
                              bool rwAvailIsLinked,
                              int numRW,
                              std::vector<float> GsMatrix_B,
                              uint32_t numControlAxes) {
    RwMotorTorqueAlgorithm alg{};

    // Set up module
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    if (numControlAxes == 3) {
        controlAxes_B.row(0) = Eigen::Vector3f {1.0F, 0.0F, 0.0F};
        controlAxes_B.row(1) = Eigen::Vector3f {0.0F, 1.0F, 0.0F};
        controlAxes_B.row(2) = Eigen::Vector3f {0.0F, 0.0F, 1.0F};
    } else if (numControlAxes == 2) {
        controlAxes_B.row(0) = Eigen::Vector3f {1.0F, 0.0F, 0.0F};
        controlAxes_B.row(1) = Eigen::Vector3f {0.0F, 1.0F, 0.0F};
    } else if (numControlAxes == 1) {
        controlAxes_B.row(0) = Eigen::Vector3f {1.0F, 0.0F, 0.0F};
    }
    alg.setControlAxes(controlAxes_B);

    // Populate messages
    CmdTorqueBodyMsgF32Payload torqueInputMsg{};
    std::copy(Lr1_B.begin(), Lr1_B.end(), torqueInputMsg.torqueRequestBody);

    CmdTorqueBodyMsgF32Payload torqueInput2Msg{};
    if (cmdTorque2IsLinked) { std::copy(Lr2_B.begin(), Lr2_B.end(), torqueInput2Msg.torqueRequestBody); }

    RWAvailabilityMsgPayload wheelsAvailabilityMsg{};
    if (rwAvailIsLinked) {
        for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
            if (wheelAvailabilityBool[i]) {
                wheelsAvailabilityMsg.wheelAvailability[i] = UNAVAILABLE;
            }
        }
    }

    RWArrayConfigMsgF32Payload rwConfigMsg{};
    rwConfigMsg.numRW = numRW;
    std::copy(GsMatrix_B.begin(), GsMatrix_B.end(), rwConfigMsg.GsMatrix_B);

    // Configure module
    uint32_t numAvailRW{};
    Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B{};
    if (!rwAvailIsLinked) {
        numAvailRW = rwConfigMsg.numRW;
        G_s_B = cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(rwConfigMsg.GsMatrix_B);
    }

    if (numControlAxes == 0) {
        EXPECT_THROW(alg.configure(rwConfigMsg, rwAvailIsLinked), fs::invalid_argument);
        return;
    }
    EXPECT_NO_THROW(alg.configure(rwConfigMsg, rwAvailIsLinked));

    // Reference
    RwMotorTorqueMsgF32Payload out{};
    RwMotorTorqueMsgF32Payload ref{};
    EXPECT_NO_THROW(out = alg.update(torqueInputMsg,
                                     torqueInput2Msg,
                                     wheelsAvailabilityMsg,
                                     cmdTorque2IsLinked,
                                     rwAvailIsLinked));
    EXPECT_NO_THROW(ref = referenceUpdate(alg,
                                          numControlAxes,
                                          numAvailRW,
                                          rwConfigMsg,
                                          G_s_B,
                                          torqueInputMsg,
                                          torqueInput2Msg,
                                          wheelsAvailabilityMsg,
                                          cmdTorque2IsLinked,
                                          rwAvailIsLinked));

    for (int i = 0; i < RW_EFF_CNT; ++i) {
        // --- General tests ---

        // Reference correctness
        EXPECT_NEAR(out.motorTorque[i], ref.motorTorque[i], 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(out.motorTorque[i]));
    }
}

#endif  // TEST_MRPSTEERING_H
