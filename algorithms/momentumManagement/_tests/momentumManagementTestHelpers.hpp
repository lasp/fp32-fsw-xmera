#ifndef TEST_MOMENTUM_MANAGEMENT_H
#define TEST_MOMENTUM_MANAGEMENT_H

#include "momentumManagementAlgorithm.h"
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// Build a reaction-wheel array configuration from a list of spin axes and a common spin-axis inertia.
// The axes are normalized here so callers can pass convenient non-unit directions. Every slot is
// configured, so a caller that names fewer axes than the array is wide gets the rest filled with a unit
// axis and a zero spin-axis inertia, which contributes no momentum.
inline MomentumManagementRwArrayConfiguration makeRwArrayConfig(const std::vector<Eigen::Vector3f>& spinAxes,
                                                                float js) {
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        const bool named = i < spinAxes.size();
        rwArrayConfig.GsMatrix_B.col(i) = named ? spinAxes[i].normalized() : Eigen::Vector3f::UnitX();
        rwArrayConfig.JsList[i] = named ? js : 0.0F;
    }
    return rwArrayConfig;
}

// The canonical four-wheel pyramid used by the Xmera unit test: three body axes plus the (1,1,1) diagonal.
// A caller that needs fewer wheels than the array is wide takes a prefix of it.
inline std::vector<Eigen::Vector3f> standardSpinAxes(std::size_t numWheels = 4U) {
    const std::vector<Eigen::Vector3f> axes{
        {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {1.0F, 1.0F, 1.0F}};
    return {axes.begin(), axes.begin() + static_cast<std::ptrdiff_t>(std::min(numWheels, axes.size()))};
}

inline MomentumManagementRwArrayConfiguration makeStandardRwArrayConfig(float js = 0.1F) {
    return makeRwArrayConfig(standardSpinAxes(), js);
}

// Orthogonal projector onto the plane perpendicular to `undumpableAxis`, i.e. what a single gimbaled thruster
// can dump about. A degenerate axis names no direction, so it yields the identity: everything is dumpable.
inline Eigen::Matrix3f makeDumpableProjection(const Eigen::Vector3f& undumpableAxis) {
    constexpr float kDegenerateTol = 1e-3F;
    if (undumpableAxis.stableNorm() < kDegenerateTol) {
        return Eigen::Matrix3f::Identity();
    }
    const Eigen::Vector3f axis = undumpableAxis.stableNormalized();
    return Eigen::Matrix3f{Eigen::Matrix3f::Identity() - (axis * axis.transpose())};
}

// Pack per-wheel speeds into the algorithm's fixed-size speed vector; unused entries stay zero.
inline Eigen::Vector<float, kMaxNumRw> makeWheelSpeeds(const std::vector<float>& speeds) {
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds = Eigen::Vector<float, kMaxNumRw>::Zero();
    for (std::size_t i = 0; i < speeds.size(); ++i) {
        wheelSpeeds[static_cast<Eigen::Index>(i)] = speeds[i];
    }
    return wheelSpeeds;
}

// Independent reference implementation of the momentum dumping law, written directly from the module
// description rather than from the algorithm source: accumulate the net RW momentum, keep only the part the
// effectors can dump, drop that when it sits below the threshold, and oppose what is left with the
// proportional and integral gains.
//
// The integral is evaluated in closed form rather than by replaying the algorithm's recurrence. For a wheel
// speed set held constant the momentum to dump d is constant, so the trapezoidal integral after numCycles
// updates is (numCycles - 0.5) * controlPeriod * d: the first update contributes half a period, each later one
// a full period. Each component of d keeps its sign, so the integral grows monotonically and clamping once at
// the end gives the same answer as the algorithm's per-update clamp.
inline Eigen::Vector3f referenceTorque(const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                       const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                       const MomentumManagementControlParameters& params,
                                       uint32_t numCycles = 1U) {
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }
    const Eigen::Vector3f hsDumpable_B = params.dumpableProjection_B * hs_B;
    const float hsDumpableNorm = hsDumpable_B.norm();

    const Eigen::Vector3f hsToDump_B = (hsDumpableNorm >= params.hsMin) ? hsDumpable_B : Eigen::Vector3f::Zero();

    const float elapsed = (static_cast<float>(numCycles) - 0.5F) * params.controlPeriod;
    Eigen::Vector3f hsInt_B = elapsed * hsToDump_B;
    for (Eigen::Index i = 0; i < 3; ++i) {
        hsInt_B[i] = std::clamp(hsInt_B[i], -params.integralLimit, params.integralLimit);
    }

    return Eigen::Vector3f{-params.K * hsToDump_B - params.Ki * hsInt_B};
}

// Config helper: assert that a (params, rwArrayConfig) pair is accepted and round-trips through the getters.
inline void testMomentumManagementSetup(const MomentumManagementControlParameters& params,
                                        const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                        float accuracy) {
    const MomentumManagementConfig cfg = MomentumManagementConfig::create(params, rwArrayConfig);

    EXPECT_NEAR(cfg.getControlParameters().hsMin, params.hsMin, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().K, params.K, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().Ki, params.Ki, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().integralLimit, params.integralLimit, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().controlPeriod, params.controlPeriod, accuracy);
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(cfg.getRwArrayConfiguration().JsList[i], rwArrayConfig.JsList[i], accuracy) << "wheel " << i;
        // Spin axes are normalized on construction, so the stored axis is the unit direction of the input.
        const Eigen::Vector3f expectedAxis = rwArrayConfig.GsMatrix_B.col(i).normalized();
        for (Eigen::Index k = 0; k < 3; ++k) {
            EXPECT_NEAR(cfg.getRwArrayConfiguration().GsMatrix_B(k, i), expectedAxis[k], accuracy)
                << "wheel " << i << " component " << k;
        }
    }
}

// Net RW cluster momentum for a configuration and speed set, used by the fuzz properties below.
inline Eigen::Vector3f clusterMomentum(const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                       const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) {
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }
    return hs_B;
}

// ---------------------------------------------------------------------------
// Properties taking any admissible configuration: the unit tests drive these with the four-wheel pyramid, the
// property*/regressionFuzz* adapters below with generated three-wheel geometries.
// ---------------------------------------------------------------------------

// Above the threshold the request opposes the stored momentum with magnitude K * |hs|, below it there is no
// request at all. Requires Ki == 0.
inline void testProportionalTorqueOpposesStoredMomentum(const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                                        const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                                        const MomentumManagementControlParameters& params) {
    ASSERT_EQ(params.Ki, 0.0F) << "this property assumes the integral term is disabled";

    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(params, rwArrayConfig)};
    const Eigen::Vector3f Lr_B = alg.update(wheelSpeeds);
    ASSERT_TRUE(Lr_B.allFinite());

    const Eigen::Vector3f hs_B = params.dumpableProjection_B * clusterMomentum(rwArrayConfig, wheelSpeeds);
    const float hsNorm = hs_B.norm();

    // The zero-momentum carve-out is pinned by the edge-case unit tests instead.
    if (hsNorm < 1e-4F) {
        return;
    }

    // FP32 error grows with the momentum magnitude and is amplified by the gain.
    const float tol = 1e-4F * params.K * std::max(1.0F, hsNorm);
    const float deadbandMargin = 1e-3F * std::max(1.0F, hsNorm);

    EXPECT_LE(Lr_B.norm(), (params.K * hsNorm) + tol);

    // Clear of the threshold either way the request is unambiguous: the whole stored momentum or nothing.
    // Within the margin the reference norm here and the algorithm's own can land on opposite sides of it.
    if (hsNorm > params.hsMin + deadbandMargin) {
        EXPECT_NEAR(Lr_B.norm(), params.K * hsNorm, tol);
        if (params.K > 0.0F) {
            EXPECT_LT(Lr_B.normalized().dot(hs_B.normalized()), 0.0F);
        }
    } else if (hsNorm < params.hsMin - deadbandMargin) {
        EXPECT_TRUE(Lr_B.isZero(tol));
    }
}

// Reversing every wheel speed reverses the requested torque. Every step is odd in the speeds (|hs| is even, so
// the deadband gate is unchanged; the integral and its sign-preserving clamp are odd), so this holds with the
// integral engaged.
inline void testTorqueIsOddInWheelSpeeds(const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                         const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                         const MomentumManagementControlParameters& params,
                                         uint32_t numCycles = 1U) {
    MomentumManagementAlgorithm forwardAlg{MomentumManagementConfig::create(params, rwArrayConfig)};
    MomentumManagementAlgorithm reversedAlg{MomentumManagementConfig::create(params, rwArrayConfig)};
    const Eigen::Vector<float, kMaxNumRw> reversedSpeeds = -wheelSpeeds;

    Eigen::Vector3f forward = Eigen::Vector3f::Zero();
    Eigen::Vector3f reversed = Eigen::Vector3f::Zero();
    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        forward = forwardAlg.update(wheelSpeeds);
        reversed = reversedAlg.update(reversedSpeeds);
    }

    // Negating the speeds negates every intermediate exactly: hs_B flips sign componentwise, its norm
    // depends only on the component magnitudes so it is unchanged, and the clamp preserves sign. Hence this
    // holds bit-for-bit and needs no error budget.
    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_EQ(reversed[i], -forward[i]) << "component " << i;
    }
}

// Every request is finite for any admissible configuration, however many cycles run.
inline void testTorqueStaysFinite(const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                  const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                  const MomentumManagementControlParameters& params,
                                  uint32_t numCycles = 1U) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(params, rwArrayConfig)};

    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        EXPECT_TRUE(alg.update(wheelSpeeds).allFinite()) << "cycle " << cycle;
    }
}

// Anti-windup: however long a momentum is held, the integral term cannot contribute more than Ki * integralLimit
// to any component. This is the property the clamp exists to guarantee.
inline void testIntegralTermStaysBounded(const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                         const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                         const MomentumManagementControlParameters& params,
                                         uint32_t numCycles = 50U) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(params, rwArrayConfig)};

    const Eigen::Vector3f hs_B = params.dumpableProjection_B * clusterMomentum(rwArrayConfig, wheelSpeeds);
    const float hsNorm = hs_B.norm();
    const Eigen::Vector3f hsToDump_B = (hsNorm >= params.hsMin) ? hs_B : Eigen::Vector3f::Zero();

    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        const Eigen::Vector3f Lr_B = alg.update(wheelSpeeds);
        ASSERT_TRUE(Lr_B.allFinite()) << "cycle " << cycle;

        for (Eigen::Index i = 0; i < 3; ++i) {
            const float bound = params.K * std::fabs(hsToDump_B[i]) + params.Ki * params.integralLimit;
            const float tol = 1e-4F * std::max(1.0F, bound);
            EXPECT_LE(std::fabs(Lr_B[i]), bound + tol) << "cycle " << cycle << " component " << i;
        }
    }
}

// The algorithm's output after numCycles updates must match the independent reference implementation.
inline void regressionTestMomentumManagement(const MomentumManagementRwArrayConfiguration& rwArrayConfig,
                                             const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                             const MomentumManagementControlParameters& params,
                                             float accuracy,
                                             uint32_t numCycles = 1U) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(params, rwArrayConfig)};

    Eigen::Vector3f actual = Eigen::Vector3f::Zero();
    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        actual = alg.update(wheelSpeeds);
    }
    const Eigen::Vector3f expected = referenceTorque(rwArrayConfig, wheelSpeeds, params, numCycles);

    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], accuracy) << "component " << i;
    }
}

// ---------------------------------------------------------------------------
// Fuzz adapters: build a three-wheel array from generated spin axes, then delegate to a core property above.
// ---------------------------------------------------------------------------

namespace detail {

// Returns false when the inputs describe a configuration the config validation rejects, so the caller skips it.
inline bool makeFuzzCase(const Eigen::Vector3f& axis0,
                         const Eigen::Vector3f& axis1,
                         const Eigen::Vector3f& axis2,
                         const Eigen::Vector3f& speeds,
                         float js,
                         const MomentumManagementControlParameters& params,
                         MomentumManagementRwArrayConfiguration& rwArrayConfig,
                         Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) {
    constexpr float degenerateTol = 1e-3F;  // an axis this short has no defined direction
    if (axis0.norm() < degenerateTol || axis1.norm() < degenerateTol || axis2.norm() < degenerateTol) {
        return false;
    }
    if (params.Ki > 0.0F && (params.controlPeriod <= 0.0F || params.integralLimit <= 0.0F)) {
        return false;
    }

    rwArrayConfig = makeRwArrayConfig({axis0, axis1, axis2}, js);
    wheelSpeeds = makeWheelSpeeds({speeds[0], speeds[1], speeds[2]});
    return true;
}

}  // namespace detail

inline void propertyProportionalTorqueOpposesStoredMomentum(const Eigen::Vector3f& axis0,
                                                            const Eigen::Vector3f& axis1,
                                                            const Eigen::Vector3f& axis2,
                                                            const Eigen::Vector3f& speeds,
                                                            const Eigen::Vector3f& undumpableAxis,
                                                            float js,
                                                            float hsMin,
                                                            float K) {
    const MomentumManagementControlParameters params{.hsMin = hsMin,
                                                     .K = K,
                                                     .Ki = 0.0F,
                                                     .integralLimit = 0.0F,
                                                     .controlPeriod = 0.0F,
                                                     .dumpableProjection_B = makeDumpableProjection(undumpableAxis)};
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    testProportionalTorqueOpposesStoredMomentum(rwArrayConfig, wheelSpeeds, params);
}

inline void propertyTorqueIsOddInWheelSpeeds(const Eigen::Vector3f& axis0,
                                             const Eigen::Vector3f& axis1,
                                             const Eigen::Vector3f& axis2,
                                             const Eigen::Vector3f& speeds,
                                             const Eigen::Vector3f& undumpableAxis,
                                             float js,
                                             float hsMin,
                                             float K,
                                             float Ki,
                                             float integralLimit,
                                             float controlPeriod,
                                             uint32_t numCycles) {
    const MomentumManagementControlParameters params{.hsMin = hsMin,
                                                     .K = K,
                                                     .Ki = Ki,
                                                     .integralLimit = integralLimit,
                                                     .controlPeriod = controlPeriod,
                                                     .dumpableProjection_B = makeDumpableProjection(undumpableAxis)};
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    testTorqueIsOddInWheelSpeeds(rwArrayConfig, wheelSpeeds, params, numCycles);
}

inline void propertyTorqueStaysFinite(const Eigen::Vector3f& axis0,
                                      const Eigen::Vector3f& axis1,
                                      const Eigen::Vector3f& axis2,
                                      const Eigen::Vector3f& speeds,
                                      const Eigen::Vector3f& undumpableAxis,
                                      float js,
                                      float hsMin,
                                      float K,
                                      float Ki,
                                      float integralLimit,
                                      float controlPeriod,
                                      uint32_t numCycles) {
    const MomentumManagementControlParameters params{.hsMin = hsMin,
                                                     .K = K,
                                                     .Ki = Ki,
                                                     .integralLimit = integralLimit,
                                                     .controlPeriod = controlPeriod,
                                                     .dumpableProjection_B = makeDumpableProjection(undumpableAxis)};
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    testTorqueStaysFinite(rwArrayConfig, wheelSpeeds, params, numCycles);
}

inline void propertyIntegralTermStaysBounded(const Eigen::Vector3f& axis0,
                                             const Eigen::Vector3f& axis1,
                                             const Eigen::Vector3f& axis2,
                                             const Eigen::Vector3f& speeds,
                                             const Eigen::Vector3f& undumpableAxis,
                                             float js,
                                             float hsMin,
                                             float K,
                                             float Ki,
                                             float integralLimit,
                                             float controlPeriod) {
    const MomentumManagementControlParameters params{.hsMin = hsMin,
                                                     .K = K,
                                                     .Ki = Ki,
                                                     .integralLimit = integralLimit,
                                                     .controlPeriod = controlPeriod,
                                                     .dumpableProjection_B = makeDumpableProjection(undumpableAxis)};
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    testIntegralTermStaysBounded(rwArrayConfig, wheelSpeeds, params);
}

inline void regressionFuzzMomentumManagement(const Eigen::Vector3f& axis0,
                                             const Eigen::Vector3f& axis1,
                                             const Eigen::Vector3f& axis2,
                                             const Eigen::Vector3f& speeds,
                                             const Eigen::Vector3f& undumpableAxis,
                                             float js,
                                             float hsMin,
                                             float K,
                                             float Ki,
                                             float integralLimit,
                                             float controlPeriod,
                                             uint32_t numCycles) {
    const MomentumManagementControlParameters params{.hsMin = hsMin,
                                                     .K = K,
                                                     .Ki = Ki,
                                                     .integralLimit = integralLimit,
                                                     .controlPeriod = controlPeriod,
                                                     .dumpableProjection_B = makeDumpableProjection(undumpableAxis)};
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    // The integral accumulates rounding once per cycle, so allow the error to grow with the cycle count.
    const float hsDumpableNorm = (params.dumpableProjection_B * clusterMomentum(rwArrayConfig, wheelSpeeds)).norm();
    const float scale = K * std::max(1.0F, hsDumpableNorm) + Ki * integralLimit;
    const float tol = 1e-4F * static_cast<float>(numCycles) * std::max(1.0F, scale);

    regressionTestMomentumManagement(rwArrayConfig, wheelSpeeds, params, tol, numCycles);
}

#endif
