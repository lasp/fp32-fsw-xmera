#ifndef TEST_RW_MOTOR_TORQUE_H
#define TEST_RW_MOTOR_TORQUE_H

#include "architecture/utilities/eigenSupport.h"
#include "rwMotorTorqueAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
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

    const Eigen::JacobiSVD<Eigen::Matrix<float, 3, kMaxNumRw>> gsSvd(G_s_B, Eigen::ComputeFullV);
    const Eigen::Vector3f gsSingularValues = gsSvd.singularValues();
    const float rankTol = gsSingularValues(0) * std::numeric_limits<float>::epsilon() * static_cast<float>(kMaxNumRw);
    if (gsSingularValues(2) <= rankTol) {
        return tau;
    }

    const Eigen::Matrix<float, kMaxNumRw, 3> Vr = gsSvd.matrixV().leftCols<3>();
    tau = Eigen::Matrix<float, kMaxNumRw, kMaxNumRw>::Identity() - Vr * Vr.transpose();

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
    // Mirror the config: orthonormalize the control axes (Gram-Schmidt over the non-zero rows).
    Eigen::Matrix3f controlAxes = controlAxes_B;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes.row(i).norm() <= 0.0F) {
            continue;
        }
        for (uint32_t k = 0U; k < i; ++k) {
            if (controlAxes.row(k).norm() > 0.0F) {
                controlAxes.row(i) -= controlAxes.row(i).dot(controlAxes.row(k)) * controlAxes.row(k);
            }
        }
        controlAxes.row(i).normalize();
    }

    uint32_t numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes.row(i).norm() > 0.0F) {
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

    const Eigen::Matrix<float, 3, kMaxNumRw> CGs = controlAxes * G_s_B;
    const Eigen::MatrixXf CGsActive = CGs.topRows(numControlAxes);
    const Eigen::JacobiSVD<Eigen::MatrixXf> svd(CGsActive, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXf& singularValues = svd.singularValues();
    const float singularValueTol = singularValues(0) * std::numeric_limits<float>::epsilon() *
                                   static_cast<float>(std::max(numControlAxes, kMaxNumRw));
    Eigen::VectorXf invSingularValues = Eigen::VectorXf::Zero(singularValues.size());
    for (Eigen::Index i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > singularValueTol) {
            invSingularValues(i) = 1.0F / singularValues(i);
        }
    }
    Eigen::Matrix<float, kMaxNumRw, 3> motorTorqueMap = svd.matrixV() * invSingularValues.asDiagonal() *
                                                        svd.matrixU().transpose() *
                                                        (-controlAxes.topRows(numControlAxes));
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= rwConfiguration.numRW || wheelsAvailability[i] != AVAILABLE) {
            motorTorqueMap.row(i).setZero();
        }
    }

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

    // control mapping matrix not full rank (3 control axes specified but not a single reaction wheel):
    // create() validates the mapping and rejects the rank-deficient configuration.
    controlAxes_B = makeControlAxes(3U);
    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability), fsw::invalid_argument);
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

    // RW speeds driving the null-space despin term (zero-padded to the full RW array).
    RwMotorTorqueSpeeds speeds{};
    for (uint32_t i = 0U; i < rwSpeeds.size() && i < kMaxNumRw; ++i) {
        speeds.rwSpeeds(static_cast<Eigen::Index>(i)) = rwSpeeds[i];
    }
    for (uint32_t i = 0U; i < rwDesiredSpeeds.size() && i < kMaxNumRw; ++i) {
        speeds.rwDesiredSpeeds(static_cast<Eigen::Index>(i)) = rwDesiredSpeeds[i];
    }

    // An uncontrollable (rank-deficient) control mapping is rejected by RwMotorTorqueConfig::create().
    if (!RwMotorTorqueConfig::isValidMapping(controlAxes_B, rwConfiguration, availability)) {
        EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, omegaGain),
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
