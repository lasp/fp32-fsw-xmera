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
    const Eigen::Vector3f& gsSingularValues = gsSvd.singularValues();
    constexpr float kConditioningTol = 1e-2F;
    if (gsSingularValues(2) <= gsSingularValues(0) * kConditioningTol) {
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

// Builds RwMotorTorqueSpeeds from caller-supplied speed vectors, zero-padded to the full RW array.
inline RwMotorTorqueSpeeds makeSpeeds(const std::vector<float>& rwSpeeds, const std::vector<float>& rwDesiredSpeeds) {
    RwMotorTorqueSpeeds speeds{};
    for (uint32_t i = 0U; i < rwSpeeds.size() && i < kMaxNumRw; ++i) {
        speeds.rwSpeeds(static_cast<Eigen::Index>(i)) = rwSpeeds[i];
    }
    for (uint32_t i = 0U; i < rwDesiredSpeeds.size() && i < kMaxNumRw; ++i) {
        speeds.rwDesiredSpeeds(static_cast<Eigen::Index>(i)) = rwDesiredSpeeds[i];
    }
    return speeds;
}

// The available-wheel spin-axis matrix [Gs] in original columns (matching the stored config), used to
// project an output torque vector back onto the body frame.
inline Eigen::Matrix<float, 3, kMaxNumRw> availableGs(const RwMotorTorqueConfig& config) {
    const RwMotorTorqueArrayConfiguration& rwConfiguration = config.getRwConfiguration();
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability = config.getAvailability().wheelAvailability;
    Eigen::Matrix<float, 3, kMaxNumRw> Gs{Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            Gs.col(i) = rwConfiguration.GsMatrix_B.col(i);
        }
    }
    return Gs;
}

// Builds the test config from raw inputs: contiguous control axes, zero-padded + normalized spin axes, and
// availability. Returns false (caller should skip the input) when the config would be invalid (no control
// axes, a non-normalizable spin axis) or not realizable (uncontrollable / ill-conditioned mapping). Shared by
// the regression and property helpers so the fuzz harness drops unusable samples silently.
inline bool buildConfig(uint32_t numControlAxes,
                        int numRW,
                        const std::vector<float>& GsMatrix_B,
                        const std::vector<bool>& wheelAvailabilityBool,
                        bool rwAvailIsLinked,
                        Eigen::Matrix3f& controlAxes_B,
                        RwMotorTorqueArrayConfiguration& rwConfiguration,
                        RwMotorTorqueAvailability& availability) {
    if (numControlAxes == 0U) {
        return false;
    }
    controlAxes_B = makeControlAxes(numControlAxes);

    rwConfiguration = RwMotorTorqueArrayConfiguration{};
    rwConfiguration.numRW = static_cast<uint32_t>(numRW);
    std::vector<float> paddedGsMatrix_B(3U * static_cast<size_t>(kMaxNumRw), 0.0F);
    std::copy(GsMatrix_B.begin(), GsMatrix_B.end(), paddedGsMatrix_B.begin());
    rwConfiguration.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(paddedGsMatrix_B.data());

    // The config requires unit spin axes; normalize the active columns. A zero column cannot be normalized.
    for (uint32_t i = 0U; i < rwConfiguration.numRW; ++i) {
        const float colNorm = rwConfiguration.GsMatrix_B.col(i).norm();
        if (colNorm <= 0.0F) {
            return false;
        }
        rwConfiguration.GsMatrix_B.col(i) /= colNorm;
    }

    availability = RwMotorTorqueAvailability{};
    if (rwAvailIsLinked) {
        for (uint32_t i = 0U; i < wheelAvailabilityBool.size() && i < kMaxNumRw; ++i) {
            availability.wheelAvailability[i] = wheelAvailabilityBool[i] ? UNAVAILABLE : AVAILABLE;
        }
    }

    // Use the algorithm's own validity check (controllable control mapping + well-conditioned null-space
    // geometry) so the harness skips exactly the configs the config factory rejects.
    return RwMotorTorqueConfig::isValidMapping(controlAxes_B, rwConfiguration, availability);
}

// ---------------------------------------------------------------------------
// Regression helper — configures the algorithm and compares update() against
// the independent fp32 reference. Re-run under fuzz inputs in the fuzz file.
// ---------------------------------------------------------------------------
inline void runRegressionCase(Eigen::Vector3f Lr1_B,
                              Eigen::Vector3f Lr2_B,
                              std::vector<bool> wheelAvailabilityBool,
                              bool cmdTorque2IsLinked,
                              bool rwAvailIsLinked,
                              int numRW,
                              std::vector<float> GsMatrix_B,
                              uint32_t numControlAxes,
                              std::vector<float> rwSpeeds,
                              std::vector<float> rwDesiredSpeeds,
                              float omegaGain) {
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    RwMotorTorqueAvailability availability{};
    if (!buildConfig(numControlAxes,
                     numRW,
                     GsMatrix_B,
                     wheelAvailabilityBool,
                     rwAvailIsLinked,
                     controlAxes_B,
                     rwConfiguration,
                     availability)) {
        return;
    }

    Eigen::Vector3f Lr_B = Lr1_B;
    if (cmdTorque2IsLinked) {
        Lr_B += Lr2_B;
    }
    const RwMotorTorqueSpeeds speeds = makeSpeeds(rwSpeeds, rwDesiredSpeeds);

    const RwMotorTorqueAlgorithm alg{
        RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, omegaGain)};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> ref =
        referenceUpdate(controlAxes_B, rwConfiguration, availability, Lr_B, speeds, omegaGain);

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(out[i], ref[i], 1e-6);
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

// ---------------------------------------------------------------------------
// Property helper functions — each asserts an invariant that must hold for any
// valid configuration and bounded inputs. Each guards invalid inputs with an
// early return so the fuzz harness can drop unusable samples silently.
// ---------------------------------------------------------------------------

// All output components are finite for a valid config and finite inputs.
inline void propertyOutputIsFinite(Eigen::Vector3f Lr1_B,
                                   Eigen::Vector3f Lr2_B,
                                   std::vector<bool> wheelAvailabilityBool,
                                   bool cmdTorque2IsLinked,
                                   bool rwAvailIsLinked,
                                   int numRW,
                                   std::vector<float> GsMatrix_B,
                                   uint32_t numControlAxes,
                                   std::vector<float> rwSpeeds,
                                   std::vector<float> rwDesiredSpeeds,
                                   float omegaGain) {
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    RwMotorTorqueAvailability availability{};
    if (!buildConfig(numControlAxes,
                     numRW,
                     GsMatrix_B,
                     wheelAvailabilityBool,
                     rwAvailIsLinked,
                     controlAxes_B,
                     rwConfiguration,
                     availability)) {
        return;
    }

    Eigen::Vector3f Lr_B = Lr1_B;
    if (cmdTorque2IsLinked) {
        Lr_B += Lr2_B;
    }
    const RwMotorTorqueAlgorithm alg{
        RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, omegaGain)};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, makeSpeeds(rwSpeeds, rwDesiredSpeeds));

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

// Excluded wheels (unavailable, or beyond numRW) receive exactly zero motor torque.
inline void propertyExcludedWheelsZeroTorque(Eigen::Vector3f Lr1_B,
                                             Eigen::Vector3f Lr2_B,
                                             std::vector<bool> wheelAvailabilityBool,
                                             bool cmdTorque2IsLinked,
                                             bool rwAvailIsLinked,
                                             int numRW,
                                             std::vector<float> GsMatrix_B,
                                             uint32_t numControlAxes,
                                             std::vector<float> rwSpeeds,
                                             std::vector<float> rwDesiredSpeeds,
                                             float omegaGain) {
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    RwMotorTorqueAvailability availability{};
    if (!buildConfig(numControlAxes,
                     numRW,
                     GsMatrix_B,
                     wheelAvailabilityBool,
                     rwAvailIsLinked,
                     controlAxes_B,
                     rwConfiguration,
                     availability)) {
        return;
    }

    Eigen::Vector3f Lr_B = Lr1_B;
    if (cmdTorque2IsLinked) {
        Lr_B += Lr2_B;
    }
    const RwMotorTorqueAlgorithm alg{
        RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, omegaGain)};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, makeSpeeds(rwSpeeds, rwDesiredSpeeds));

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= rwConfiguration.numRW || availability.wheelAvailability[i] != AVAILABLE) {
            EXPECT_FLOAT_EQ(out[i], 0.0F);
        }
    }
}

// The null-space despin term adds no net body torque: [Gs] applied to the despin output is zero (up to fp32
// round-off scaled by the despin magnitude).
inline void propertyDespinAddsNoBodyTorque(std::vector<bool> wheelAvailabilityBool,
                                           bool rwAvailIsLinked,
                                           int numRW,
                                           std::vector<float> GsMatrix_B,
                                           uint32_t numControlAxes,
                                           std::vector<float> rwSpeeds,
                                           std::vector<float> rwDesiredSpeeds,
                                           float omegaGain) {
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    RwMotorTorqueAvailability availability{};
    if (!buildConfig(numControlAxes,
                     numRW,
                     GsMatrix_B,
                     wheelAvailabilityBool,
                     rwAvailIsLinked,
                     controlAxes_B,
                     rwConfiguration,
                     availability)) {
        return;
    }

    const RwMotorTorqueConfig config =
        RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, omegaGain);
    const RwMotorTorqueAlgorithm alg{config};

    // Zero command: the control term vanishes, so the output is the despin term tau * d on its own.
    const Eigen::Vector<float, kMaxNumRw> despin =
        alg.update(Eigen::Vector3f::Zero(), makeSpeeds(rwSpeeds, rwDesiredSpeeds));

    // The config factory rejects ill-conditioned null-space geometry, so for any constructible config the
    // despin lies cleanly in the null space and produces no body torque (up to fp32 round-off).
    const Eigen::Vector3f bodyTorque = availableGs(config) * despin;
    EXPECT_LE(bodyTorque.norm(), 1e-3F * despin.norm() + 1e-4F);
}

// With a zero gain the despin term vanishes: the output is independent of the RW speeds.
inline void propertyZeroGainDisablesDespin(Eigen::Vector3f Lr1_B,
                                           Eigen::Vector3f Lr2_B,
                                           std::vector<bool> wheelAvailabilityBool,
                                           bool cmdTorque2IsLinked,
                                           bool rwAvailIsLinked,
                                           int numRW,
                                           std::vector<float> GsMatrix_B,
                                           uint32_t numControlAxes,
                                           std::vector<float> rwSpeeds,
                                           std::vector<float> rwDesiredSpeeds) {
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    RwMotorTorqueAvailability availability{};
    if (!buildConfig(numControlAxes,
                     numRW,
                     GsMatrix_B,
                     wheelAvailabilityBool,
                     rwAvailIsLinked,
                     controlAxes_B,
                     rwConfiguration,
                     availability)) {
        return;
    }

    Eigen::Vector3f Lr_B = Lr1_B;
    if (cmdTorque2IsLinked) {
        Lr_B += Lr2_B;
    }
    const RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, 0.0F)};

    const Eigen::Vector<float, kMaxNumRw> withSpeeds = alg.update(Lr_B, makeSpeeds(rwSpeeds, rwDesiredSpeeds));
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(withSpeeds[i], controlOnly[i]);
    }
}

// The control-only output realizes the commanded torque on the controllable axes: projecting the body torque
// the wheels produce, [Gs] u_control, onto each control axis recovers the commanded torque -Lr_B on that axis.
inline void propertyControlTorqueRealized(Eigen::Vector3f Lr1_B,
                                          Eigen::Vector3f Lr2_B,
                                          std::vector<bool> wheelAvailabilityBool,
                                          bool cmdTorque2IsLinked,
                                          bool rwAvailIsLinked,
                                          int numRW,
                                          std::vector<float> GsMatrix_B,
                                          uint32_t numControlAxes) {
    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    RwMotorTorqueAvailability availability{};
    if (!buildConfig(numControlAxes,
                     numRW,
                     GsMatrix_B,
                     wheelAvailabilityBool,
                     rwAvailIsLinked,
                     controlAxes_B,
                     rwConfiguration,
                     availability)) {
        return;
    }

    Eigen::Vector3f Lr_B = Lr1_B;
    if (cmdTorque2IsLinked) {
        Lr_B += Lr2_B;
    }
    const RwMotorTorqueConfig config = RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability);
    const RwMotorTorqueAlgorithm alg{config};

    // Control-only output (zero speeds): the despin term is absent, so this is the pure control mapping.
    const Eigen::Vector3f bodyTorque = availableGs(config) * alg.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Matrix3f& storedAxes = config.getControlAxes();
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (storedAxes.row(i).norm() > 0.0F) {
            const float realized = storedAxes.row(i).dot(bodyTorque);
            const float commanded = -storedAxes.row(i).dot(Lr_B);
            EXPECT_NEAR(realized, commanded, 1e-3F * Lr_B.norm() + 1e-3F);
        }
    }
}

#endif  // TEST_RW_MOTOR_TORQUE_H
