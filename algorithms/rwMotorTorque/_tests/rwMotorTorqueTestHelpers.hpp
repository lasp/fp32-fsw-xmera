#ifndef TEST_RW_MOTOR_TORQUE_H
#define TEST_RW_MOTOR_TORQUE_H

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include "rwMotorTorqueAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
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

// Independent fp64 reference for the null-space projection [tau]. Works entirely in double; the caller casts
// the float spin-axis matrix to double.
inline Eigen::Matrix<double, kMaxNumRw, kMaxNumRw> referenceTau(const Eigen::Matrix<double, 3, kMaxNumRw>& GsMatrix_B,
                                                                uint32_t numRW,
                                                                const RwMotorTorqueAvailability& availability) {
    Eigen::Matrix<double, kMaxNumRw, kMaxNumRw> tau{Eigen::Matrix<double, kMaxNumRw, kMaxNumRw>::Zero()};
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability = availability.wheelAvailability;

    Eigen::Matrix<double, 3, kMaxNumRw> G_s_B{Eigen::Matrix<double, 3, kMaxNumRw>::Zero()};
    uint32_t numAvailRW = 0U;
    for (uint32_t i = 0U; i < numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(i) = GsMatrix_B.col(i).normalized();
            numAvailRW += 1U;
        }
    }

    if (numAvailRW <= 3U) {
        return tau;
    }

    const Eigen::JacobiSVD<Eigen::Matrix<double, 3, kMaxNumRw>> gsSvd(G_s_B, Eigen::ComputeFullV);
    const Eigen::Vector3d& gsSingularValues = gsSvd.singularValues();
    constexpr double kConditioningTol = 1e-2;
    if (gsSingularValues(2) <= gsSingularValues(0) * kConditioningTol) {
        return tau;
    }

    const Eigen::Matrix<double, kMaxNumRw, 3> Vr = gsSvd.matrixV().leftCols<3>();
    tau = Eigen::Matrix<double, kMaxNumRw, kMaxNumRw>::Identity() - Vr * Vr.transpose();

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= numRW || wheelsAvailability[i] != AVAILABLE) {
            tau.row(i).setZero();
        }
    }
    return tau;
}

// Independent fp64 reference for update() (control mapping + null-space). Works entirely in double; the
// caller casts the float inputs to double, and casts the returned result back to float for comparison.
inline Eigen::Vector<double, kMaxNumRw> referenceUpdate(const Eigen::Matrix3d& controlAxes_B,
                                                        const Eigen::Matrix<double, 3, kMaxNumRw>& GsMatrix_B,
                                                        uint32_t numRW,
                                                        const RwMotorTorqueAvailability& availability,
                                                        const Eigen::Vector3d& Lr_B,
                                                        const Eigen::Vector<double, kMaxNumRw>& rwSpeeds,
                                                        const Eigen::Vector<double, kMaxNumRw>& rwDesiredSpeeds,
                                                        double omegaGain) {
    // Orthonormalize the control axes (Gram-Schmidt over the non-zero rows), matching the config.
    Eigen::Matrix3d controlAxes = controlAxes_B;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes.row(i).norm() <= 0.0) {
            continue;
        }
        for (uint32_t k = 0U; k < i; ++k) {
            if (controlAxes.row(k).norm() > 0.0) {
                controlAxes.row(i) -= controlAxes.row(i).dot(controlAxes.row(k)) * controlAxes.row(k);
            }
        }
        controlAxes.row(i).normalize();
    }

    uint32_t numControlAxes = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (controlAxes.row(i).norm() > 0.0) {
            numControlAxes += 1U;
        }
    }

    Eigen::Matrix<double, 3, kMaxNumRw> G_s_B{Eigen::Matrix<double, 3, kMaxNumRw>::Zero()};
    const std::array<FSWdeviceAvailability, kMaxNumRw>& wheelsAvailability = availability.wheelAvailability;
    for (uint32_t i = 0U; i < numRW; ++i) {
        if (wheelsAvailability[i] == AVAILABLE) {
            G_s_B.col(i) = GsMatrix_B.col(i).normalized();
        }
    }

    // Truncated-SVD pseudo-inverse of [CGs]. The cutoff uses fp32 epsilon (the algorithm's noise floor) so the
    // reference keeps the same singular values the fp32 algorithm keeps.
    const Eigen::Matrix<double, 3, kMaxNumRw> CGs = controlAxes * G_s_B;
    const Eigen::MatrixXd CGsActive = CGs.topRows(numControlAxes);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(CGsActive, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& singularValues = svd.singularValues();
    const double singularValueTol = singularValues(0) * static_cast<double>(std::numeric_limits<float>::epsilon()) *
                                    static_cast<double>(std::max(numControlAxes, kMaxNumRw));
    Eigen::VectorXd invSingularValues = Eigen::VectorXd::Zero(singularValues.size());
    for (Eigen::Index i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > singularValueTol) {
            invSingularValues(i) = 1.0 / singularValues(i);
        }
    }
    Eigen::Matrix<double, kMaxNumRw, 3> motorTorqueMap = svd.matrixV() * invSingularValues.asDiagonal() *
                                                         svd.matrixU().transpose() *
                                                         (-controlAxes.topRows(numControlAxes));
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        if (i >= numRW || wheelsAvailability[i] != AVAILABLE) {
            motorTorqueMap.row(i).setZero();
        }
    }

    const Eigen::Vector<double, kMaxNumRw> d = -omegaGain * (rwSpeeds - rwDesiredSpeeds);
    const Eigen::Vector<double, kMaxNumRw> nullSpaceTorque = referenceTau(GsMatrix_B, numRW, availability) * d;

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

    // Validate the control and null-space contributions separately against the fp64 reference, each with the
    // tolerance matched to its error source. By linearity, update(Lr, speeds) = update(Lr, 0) + update(0, speeds),
    // so isolating the two terms lets each comparison use the right scale. A single combined tolerance would
    // let a large ||d|| slacken the control comparison and mask a control-path error.
    const Eigen::Vector<float, kMaxNumRw> controlOut = alg.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Vector<float, kMaxNumRw> nullSpaceOut = alg.update(Eigen::Vector3f::Zero(), speeds);
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, speeds);

    const Eigen::Vector<double, kMaxNumRw> zeroSpeeds = Eigen::Vector<double, kMaxNumRw>::Zero();
    const Eigen::Vector<double, kMaxNumRw> controlRef = referenceUpdate(controlAxes_B.cast<double>(),
                                                                        rwConfiguration.GsMatrix_B.cast<double>(),
                                                                        rwConfiguration.numRW,
                                                                        availability,
                                                                        Lr_B.cast<double>(),
                                                                        zeroSpeeds,
                                                                        zeroSpeeds,
                                                                        static_cast<double>(omegaGain));
    const Eigen::Vector<double, kMaxNumRw> nullSpaceRef = referenceUpdate(controlAxes_B.cast<double>(),
                                                                          rwConfiguration.GsMatrix_B.cast<double>(),
                                                                          rwConfiguration.numRW,
                                                                          availability,
                                                                          Eigen::Vector3d::Zero(),
                                                                          speeds.rwSpeeds.cast<double>(),
                                                                          speeds.rwDesiredSpeeds.cast<double>(),
                                                                          static_cast<double>(omegaGain));

    const float controlScale = static_cast<float>(controlRef.cwiseAbs().maxCoeff());
    const Eigen::Vector<float, kMaxNumRw> nullSpaceInput = omegaGain * (speeds.rwSpeeds - speeds.rwDesiredSpeeds);
    const float controlTol = 1e-4F + 1e-3F * controlScale;
    const float nullSpaceTol = 1e-4F + 1e-5F * nullSpaceInput.norm();
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(controlOut[i], static_cast<float>(controlRef[i]), controlTol);
        EXPECT_NEAR(nullSpaceOut[i], static_cast<float>(nullSpaceRef[i]), nullSpaceTol);
        // The production update() output is exactly the fp32 sum of the two contributions (linearity).
        EXPECT_FLOAT_EQ(out[i], controlOut[i] + nullSpaceOut[i]);
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

// The null-space term adds no net body torque: [Gs] applied to the null-space output is zero (up to fp32
// round-off scaled by the null-space magnitude).
inline void propertyNullSpaceAddsNoBodyTorque(std::vector<bool> wheelAvailabilityBool,
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

    // Zero command: the control term vanishes, so the output is the null-space term tau * d on its own.
    const RwMotorTorqueSpeeds speeds = makeSpeeds(rwSpeeds, rwDesiredSpeeds);
    const Eigen::Vector<float, kMaxNumRw> nullSpaceOut = alg.update(Eigen::Vector3f::Zero(), speeds);

    // The config factory rejects ill-conditioned null-space geometry, so for any constructible config the
    // null-space lies cleanly in the null space and produces no body torque (up to fp32 round-off). [Gs] * tau == 0
    // in exact arithmetic, so the residual is pure round-off whose magnitude scales with the null-space term input
    // d = -omegaGain * (rwSpeeds - rwDesiredSpeeds). The tolerance is therefore scaled by ||d||.
    // Essentially, this compares that the torque on the body is negligible to the torque applied to drive the wheel
    // speeds toward their desired values.
    const Eigen::Vector<float, kMaxNumRw> nullSpaceInput = omegaGain * (speeds.rwSpeeds - speeds.rwDesiredSpeeds);
    const Eigen::Vector3f bodyTorque = availableGs(config) * nullSpaceOut;
    EXPECT_LE(bodyTorque.norm(), 1e-4F + 1e-5F * nullSpaceInput.norm());
}

// With a zero gain the null-space term vanishes: the output is independent of the RW speeds.
inline void propertyZeroGainDisablesNullSpace(Eigen::Vector3f Lr1_B,
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

    // Control-only output (zero speeds): the null-space term is absent, so this is the pure control mapping.
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
