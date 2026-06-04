#ifndef TEST_RW_MOTOR_TORQUE_H
#define TEST_RW_MOTOR_TORQUE_H

#include "architecture/utilities/eigenSupport.h"
#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

// Mirrors the algorithm's SVD controllability cross-check: returns true iff every control axis is
// reachable by the available reaction wheels (i.e. the construction of the algorithm will not throw).
inline bool isControllable(const Eigen::Matrix<float, 3, kMaxNumRw>& CGs, uint32_t numControlAxes) {
    const Eigen::JacobiSVD<Eigen::MatrixXf> svd(CGs.topRows(numControlAxes), Eigen::ComputeFullU);
    const Eigen::VectorXf& singularValues = svd.singularValues();
    const Eigen::MatrixXf& leftSingularVectors = svd.matrixU();
    const float singularValueTol = singularValues(0) * std::numeric_limits<float>::epsilon() *
                                   static_cast<float>(std::max(numControlAxes, kMaxNumRw));
    constexpr float kControllabilityResidualSqTol = 1e-6F;
    for (uint32_t axis = 0U; axis < numControlAxes; ++axis) {
        float nullSpaceResidualSq = 0.0F;
        for (uint32_t k = 0U; k < numControlAxes; ++k) {
            if (singularValues(k) <= singularValueTol) {
                nullSpaceResidualSq += leftSingularVectors(axis, k) * leftSingularVectors(axis, k);
            }
        }
        if (nullSpaceResidualSq > kControllabilityResidualSqTol) {
            return false;
        }
    }
    return true;
}

// Reference [tau], mirroring computeNullSpaceProjection's fp32 computation.
inline Eigen::Matrix<float, kMaxNumRw, kMaxNumRw> referenceTau(const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                                               const RwMotorTorqueAvailability& availability) {
    Eigen::Matrix<float, kMaxNumRw, kMaxNumRw> tau{Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Zero()};
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability = availability.wheelAvailability;

    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailRW = 0U;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(i) = rwConfiguration.GsMatrix_B.col(i).normalized();
            numAvailRW += 1U;
        }
    }

    if (numAvailRW <= 3U) {
        return tau;
    }

    const Eigen::Matrix3f GsGsT = G_s_B * G_s_B.transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3f> gsSvd(GsGsT);
    const Eigen::Vector3f gsSingularValues = gsSvd.singularValues();
    constexpr float kSpanRelativeTol = 1e-6F;
    if (gsSingularValues(2) <= gsSingularValues(0) * kSpanRelativeTol) {
        return tau;
    }

    tau = Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Identity() - G_s_B.transpose() * GsGsT.inverse() * G_s_B;

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= rwConfiguration.numRW || wheelsAvailability[i] != AVAILABLE) {
            tau.row(i).setZero();
        }
    }
    return tau;
}

// Reference RW motor torques (control mapping + null-space despin), mirroring the algorithm's fp32 computation.
inline Eigen::Vector<float, kMaxNumRw> referenceUpdate(const Eigen::Matrix3f& controlAxes_B,
                                                       const RwMotorTorqueArrayConfiguration& rwConfiguration,
                                                       const RwMotorTorqueAvailability& availability,
                                                       const Eigen::Vector3f& Lr_B,
                                                       const RwMotorTorqueSpeeds& speeds,
                                                       float omegaGain) {
    uint32_t numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes_B.row(i).norm() > 0.0F) {
            numControlAxes += 1U;
        }
    }

    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability = availability.wheelAvailability;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(i) = rwConfiguration.GsMatrix_B.col(i).normalized();
        }
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes_B * G_s_B;
    const Eigen::MatrixXf CGsActive = CGs.topRows(numControlAxes);
    const Eigen::Matrix<float, kMaxNumRw, 3> motorTorqueMap = CGsActive.transpose() *
                                                              (CGsActive * CGsActive.transpose()).inverse() *
                                                              (-controlAxes_B.topRows(numControlAxes));

    const Eigen::Vector<float, kMaxNumRw> d = -omegaGain * (speeds.rwSpeeds - speeds.rwDesiredSpeeds);
    const Eigen::Vector<float, kMaxNumRw> nullSpaceTorque = referenceTau(rwConfiguration, availability) * d;

    return motorTorqueMap * Lr_B + nullSpaceTorque;
}

inline void testRwMotorTorqueSetup() {
    // --- Test expected exceptions ---

    const RwMotorTorqueArrayConfiguration rwConfiguration{};
    const RwMotorTorqueAvailability availability{};

    // A non-unit control axis is rejected by RwMotorTorqueConfig.
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    controlAxes_B.row(0) = Eigen::Vector3f{2.0F, 0.0F, 0.0F};
    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability), fsw::invalid_argument);

    // Non-orthogonal control axes are rejected by RwMotorTorqueConfig.
    controlAxes_B = Eigen::Matrix3f::Zero();
    controlAxes_B.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes_B.row(1) = Eigen::Vector3f{0.70710678F, 0.70710678F, 0.0F};
    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability), fsw::invalid_argument);

    // control mapping matrix not full rank (to test, 3 control axes are specified but not a single reaction wheel):
    // the config is valid, but constructing the algorithm (which computes the mapping) rejects the rank-deficient case
    controlAxes_B = makeControlAxes(3U);
    EXPECT_THROW(RwMotorTorqueAlgorithm{RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability)},
                 fsw::invalid_argument);
}

inline void testRwMotorTorque(const Eigen::Vector3f& Lr1_B,
                              const Eigen::Vector3f& Lr2_B,
                              std::vector<bool> wheelAvailabilityBool,
                              bool cmdTorque2IsLinked,
                              bool rwAvailIsLinked,
                              int numRW,
                              std::vector<float> GsMatrix_B,
                              uint32_t numControlAxes,
                              std::vector<float> rwSpeeds,
                              std::vector<float> rwDesiredSpeeds,
                              float omegaGain) {
    // Set up the control axes mapping matrix.
    const Eigen::Matrix3f controlAxes_B = makeControlAxes(numControlAxes);

    // Zero-pad the caller's 3 * numRW spin-axis entries to the full matrix (avoids reading past the vector).
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = static_cast<uint32_t>(numRW);
    std::vector<float> paddedGsMatrix_B(3U * static_cast<size_t>(kMaxNumRw), 0.0F);
    std::copy(GsMatrix_B.begin(), GsMatrix_B.end(), paddedGsMatrix_B.begin());
    rwConfiguration.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(paddedGsMatrix_B.data());

    // The config requires unit spin axes; normalize the active columns. A zero column cannot be normalized,
    // leaving the config invalid so construction must throw.
    bool spinAxesNormalizable = true;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        const float colNorm = rwConfiguration.GsMatrix_B.col(i).norm();
        if (colNorm > 0.0F) {
            rwConfiguration.GsMatrix_B.col(i) /= colNorm;
        } else {
            spinAxesNormalizable = false;
        }
    }

    // Build the availability: wheelAvailabilityBool[i] == true marks wheel i UNAVAILABLE
    RwMotorTorqueAvailability availability{};
    if (rwAvailIsLinked) {
        for (uint32_t i = 0U; i < wheelAvailabilityBool.size(); ++i) {
            availability.wheelAvailability[i] = wheelAvailabilityBool[i] ? UNAVAILABLE : AVAILABLE;
        }
    }

    // A zero control-axes matrix (no control axes) is rejected by the RwMotorTorqueConfig factory.
    if (numControlAxes == 0U) {
        EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability), fsw::invalid_argument);
        return;
    }

    // A non-unit (here, zero) spin axis is rejected by the RwMotorTorqueConfig factory.
    if (!spinAxesNormalizable) {
        EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, omegaGain),
                     fsw::invalid_argument);
        return;
    }

    // Total commanded torque seen by the algorithm (adapter sums the two messages)
    Eigen::Vector3f Lr_B = Lr1_B;
    if (cmdTorque2IsLinked) {
        Lr_B += Lr2_B;
    }

    // Independently compute the available RW count and rank to predict construction behavior. Wheels left
    // at the default AVAILABLE state (no availability message) are always included.
    Eigen::Matrix<float, 3, kMaxNumRw> G_s_B{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailWheels = 0U;
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (availability.wheelAvailability[i] == AVAILABLE) {
            G_s_B.col(numAvailWheels) = rwConfiguration.GsMatrix_B.col(i).normalized();
            numAvailWheels += 1U;
        }
    }

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes_B * G_s_B;

    // RW speeds driving the null-space despin term (zero-padded to the full RW array).
    RwMotorTorqueSpeeds speeds{};
    for (uint32_t i = 0U; i < rwSpeeds.size() && i < kMaxNumRw; ++i) {
        speeds.rwSpeeds(static_cast<Eigen::Index>(i)) = rwSpeeds[i];
    }
    for (uint32_t i = 0U; i < rwDesiredSpeeds.size() && i < kMaxNumRw; ++i) {
        speeds.rwDesiredSpeeds(static_cast<Eigen::Index>(i)) = rwDesiredSpeeds[i];
    }

    // An uncontrollable (rank-deficient) control mapping makes the constructor (which computes the
    // mapping and runs the controllability cross-check) throw.
    if (!isControllable(CGs, numControlAxes)) {
        EXPECT_THROW(RwMotorTorqueAlgorithm{RwMotorTorqueConfig::create(
                         controlAxes_B, rwConfiguration, availability, omegaGain)},
                     fsw::invalid_argument);
        return;
    }
    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, omegaGain)};

    // Compare against the independent reference
    Eigen::Vector<float, kMaxNumRw> out{Eigen::Vector<float, kMaxNumRw>::Zero()};
    Eigen::Vector<float, kMaxNumRw> ref{Eigen::Vector<float, kMaxNumRw>::Zero()};
    EXPECT_NO_THROW(out = alg.update(Lr_B, speeds));
    EXPECT_NO_THROW(ref = referenceUpdate(controlAxes_B, rwConfiguration, availability, Lr_B, speeds, omegaGain));

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        // Reference correctness
        EXPECT_NEAR(out[i], ref[i], 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

#endif  // TEST_RW_MOTOR_TORQUE_H
