#ifndef TEST_RW_MOTOR_TORQUE_H
#define TEST_RW_MOTOR_TORQUE_H

#include "architecture/utilities/eigenSupport.h"
#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
#include <cstdint>
#include <vector>

// Builds the control axes mapping matrix used by the tests for a given number of controlled axes.
inline Eigen::Matrix3f makeControlAxes(uint32_t numControlAxes) {
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    if (numControlAxes >= 1U) {
        controlAxes_B.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    }
    if (numControlAxes >= 2U) {
        controlAxes_B.row(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    }
    if (numControlAxes >= 3U) {
        controlAxes_B.row(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    }
    return controlAxes_B;
}

// Independent reference computation of the RW motor torques for verification.
inline Eigen::Vector<float, kMaxNumRw> referenceUpdate(const Eigen::Matrix3f& controlAxes_B,
                                                       const RwMotorTorqueArrayConfig& rwConfig,
                                                       const RwMotorTorqueAvailability& availability,
                                                       bool rwAvailIsLinked,
                                                       const Eigen::Vector3f& Lr_B) {
    uint32_t numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes_B.row(i).norm() > 0.0F) {
            numControlAxes += 1U;
        }
    }

    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailRW = 0U;
    std::array<FSWdeviceAvailability, kMaxNumRw> wheelsAvailability{};
    if (rwAvailIsLinked) {
        wheelsAvailability = availability.wheelAvailability;
        for (uint32_t i = 0U; i < rwConfig.numRW; ++i) {
            if (wheelsAvailability[i] == AVAILABLE) {
                G_s_B.col(numAvailRW) = rwConfig.GsMatrix_B.col(i).normalized();
                numAvailRW += 1U;
            }
        }
    } else {
        for (uint32_t i = 0U; i < rwConfig.numRW; ++i) {
            G_s_B.col(i) = rwConfig.GsMatrix_B.col(i).normalized();
        }
        numAvailRW = rwConfig.numRW;
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes_B * G_s_B;

    Eigen::Vector<float, kMaxNumRw> us = Eigen::Vector<float, kMaxNumRw>::Zero();
    const uint32_t numRows = numControlAxes;
    const uint32_t numCols = numAvailRW;

    Eigen::Vector3f Lr_C{Eigen::Vector3f::Zero()};
    Lr_C.head(numRows) = -controlAxes_B.topRows(numRows) * Lr_B;

    Eigen::Vector<float, kMaxNumRw> us_avail{Eigen::Vector<float, kMaxNumRw>::Zero()};
    us_avail.topRows(numCols) =
        CGs.topLeftCorner(numRows, numCols).transpose() *
        (CGs.topLeftCorner(numRows, numCols) * CGs.topLeftCorner(numRows, numCols).transpose()).inverse() *
        Lr_C.topRows(numRows);

    uint32_t j = 0U;
    for (uint32_t i = 0U; i < rwConfig.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            us[i] = us_avail[j];
            j += 1U;
        }
    }

    return us;
}

inline void testRwMotorTorqueSetup() {
    // --- Test expected exceptions ---

    const RwMotorTorqueArrayConfig rwConfig{};
    const RwMotorTorqueAvailability availability{};

    // control axes matrix not properly set up (control axes not filled from top to bottom before any zero rows):
    // RwMotorTorqueConfig rejects it at construction time
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    controlAxes_B.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes_B.row(1) = Eigen::Vector3f{0.0F, 0.0F, 0.0F};
    controlAxes_B.row(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B), fsw::invalid_argument);

    // control mapping matrix not full rank (to test, 3 control axes are specified but not a single reaction wheel):
    // the config is valid, but configure() rejects the rank-deficient mapping
    controlAxes_B = makeControlAxes(3U);
    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(controlAxes_B)};
    EXPECT_THROW(alg.configure(rwConfig, availability, false), fsw::invalid_argument);
}

inline void testRwMotorTorque(const Eigen::Vector3f& Lr1_B,
                              const Eigen::Vector3f& Lr2_B,
                              std::vector<bool> wheelAvailabilityBool,
                              bool cmdTorque2IsLinked,
                              bool rwAvailIsLinked,
                              int numRW,
                              std::vector<float> GsMatrix_B,
                              uint32_t numControlAxes) {
    // Set up the control axes mapping matrix. A zero matrix (no control axes) is rejected by the
    // RwMotorTorqueConfig factory, so test that and return early.
    const Eigen::Matrix3f controlAxes_B = makeControlAxes(numControlAxes);
    if (numControlAxes == 0U) {
        EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B), fsw::invalid_argument);
        return;
    }
    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(controlAxes_B)};

    // Build the RW array configuration from the flat spin-axis array
    RwMotorTorqueArrayConfig rwConfig{};
    rwConfig.numRW = static_cast<uint32_t>(numRW);
    rwConfig.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(GsMatrix_B.data());

    // Build the availability: wheelAvailabilityBool[i] == true marks wheel i UNAVAILABLE
    RwMotorTorqueAvailability availability{};
    if (rwAvailIsLinked) {
        for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
            availability.wheelAvailability[i] = wheelAvailabilityBool[i] ? UNAVAILABLE : AVAILABLE;
        }
    }

    // Total commanded torque seen by the algorithm (adapter sums the two messages)
    Eigen::Vector3f Lr_B = Lr1_B;
    if (cmdTorque2IsLinked) {
        Lr_B += Lr2_B;
    }

    // Independently compute the available RW count and rank to predict configure() behavior
    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    if (rwAvailIsLinked) {
        uint32_t numAvailWheels = 0U;
        for (uint32_t i = 0U; i < rwConfig.numRW; ++i) {
            if (availability.wheelAvailability[i] == AVAILABLE) {
                G_s_B.col(numAvailWheels) = rwConfig.GsMatrix_B.col(i).normalized();
                numAvailWheels += 1U;
            }
        }
    } else {
        for (uint32_t i = 0U; i < rwConfig.numRW; ++i) {
            G_s_B.col(i) = rwConfig.GsMatrix_B.col(i).normalized();
        }
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes_B * G_s_B;
    const Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(CGs);
    const auto controlMappingRank = static_cast<uint32_t>(lu_decomp.rank());

    if (controlMappingRank < numControlAxes) {
        EXPECT_THROW(alg.configure(rwConfig, availability, rwAvailIsLinked), fsw::invalid_argument);
        return;
    }
    EXPECT_NO_THROW(alg.configure(rwConfig, availability, rwAvailIsLinked));

    // Compare against the independent reference
    Eigen::Vector<float, kMaxNumRw> out{Eigen::Vector<float, kMaxNumRw>::Zero()};
    Eigen::Vector<float, kMaxNumRw> ref{Eigen::Vector<float, kMaxNumRw>::Zero()};
    EXPECT_NO_THROW(out = alg.update(Lr_B));
    EXPECT_NO_THROW(ref = referenceUpdate(controlAxes_B, rwConfig, availability, rwAvailIsLinked, Lr_B));

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        // Reference correctness
        EXPECT_NEAR(out[i], ref[i], 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

#endif  // TEST_RW_MOTOR_TORQUE_H
