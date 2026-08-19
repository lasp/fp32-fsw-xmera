#ifndef TEST_THR_MOMENTUM_MANAGEMENT_H
#define TEST_THR_MOMENTUM_MANAGEMENT_H

#include "thrMomentumManagementAlgorithm.h"
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// Build a reaction-wheel array configuration from a list of spin axes and a common spin-axis inertia.
// The axes are normalized here so callers can pass convenient non-unit directions.
inline ThrMomentumManagementRwArrayConfiguration makeRwArrayConfig(const std::vector<Eigen::Vector3f>& spinAxes,
                                                                   float js) {
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
    rwArrayConfig.numRW = static_cast<uint32_t>(spinAxes.size());
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
        rwArrayConfig.GsMatrix_B.col(i) = spinAxes[i].normalized();
        rwArrayConfig.JsList[i] = js;
    }
    return rwArrayConfig;
}

// The canonical four-wheel pyramid used by the Xmera unit test: three body axes plus the (1,1,1) diagonal.
inline ThrMomentumManagementRwArrayConfiguration makeStandardRwArrayConfig(float js = 0.1F) {
    return makeRwArrayConfig({{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {1.0F, 1.0F, 1.0F}}, js);
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
// description rather than from the algorithm source: accumulate the net RW momentum, isolate the part held
// above the threshold, and oppose it with the proportional and integral gains.
//
// The integral is evaluated in closed form rather than by replaying the algorithm's recurrence. For a wheel
// speed set held constant the excess momentum e is constant, so the trapezoidal integral after numCycles
// updates is (numCycles - 0.5) * controlPeriod * e: the first update contributes half a period, each later one
// a full period. Each component of e keeps its sign, so the integral grows monotonically and clamping once at
// the end gives the same answer as the algorithm's per-update clamp.
inline Eigen::Vector3f referenceTorque(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                       const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                       const ThrMomentumManagementControlParameters& params,
                                       uint32_t numCycles = 1U) {
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }
    const float hs = hs_B.norm();

    Eigen::Vector3f hsExcess_B = Eigen::Vector3f::Zero();
    if (hs >= params.hsMin && hs >= 1e-6F) {
        hsExcess_B = (hs - params.hsMin) / hs * hs_B;
    }

    const float elapsed = (static_cast<float>(numCycles) - 0.5F) * params.controlPeriod;
    Eigen::Vector3f hsInt_B = elapsed * hsExcess_B;
    for (Eigen::Index i = 0; i < 3; ++i) {
        hsInt_B[i] = std::clamp(hsInt_B[i], -params.integralLimit, params.integralLimit);
    }

    return Eigen::Vector3f{-params.K * hsExcess_B - params.Ki * hsInt_B};
}

// Config helper: assert that a (params, rwArrayConfig) pair is accepted and round-trips through the getters.
inline void testThrMomentumManagementSetup(const ThrMomentumManagementControlParameters& params,
                                           const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                           float accuracy) {
    const ThrMomentumManagementConfig cfg = ThrMomentumManagementConfig::create(params, rwArrayConfig);

    EXPECT_NEAR(cfg.getControlParameters().hsMin, params.hsMin, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().K, params.K, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().Ki, params.Ki, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().integralLimit, params.integralLimit, accuracy);
    EXPECT_NEAR(cfg.getControlParameters().controlPeriod, params.controlPeriod, accuracy);
    EXPECT_EQ(cfg.getRwArrayConfiguration().numRW, rwArrayConfig.numRW);
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
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
inline Eigen::Vector3f clusterMomentum(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                       const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds) {
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }
    return hs_B;
}

// ---------------------------------------------------------------------------
// Properties taking any admissible configuration: the unit tests drive these with the four-wheel pyramid, the
// property*/regressionFuzz* adapters below with generated three-wheel geometries.
// ---------------------------------------------------------------------------

// The request opposes the stored momentum with magnitude K * (|hs| - hsMin), so it acts on exactly the momentum
// held above the threshold and never on momentum the cluster does not hold. Requires Ki == 0.
inline void testProportionalTorqueOpposesExcessMomentum(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                                        const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                                        const ThrMomentumManagementControlParameters& params) {
    ASSERT_EQ(params.Ki, 0.0F) << "this property assumes the integral term is disabled";

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};
    const Eigen::Vector3f Lr_B = alg.update(wheelSpeeds);
    ASSERT_TRUE(Lr_B.allFinite());

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);
    const float hs = hs_B.norm();

    // The zero-momentum carve-out is pinned by the edge-case unit tests instead.
    if (hs < 1e-4F) {
        return;
    }

    // FP32 error grows with the momentum magnitude and is amplified by the gain.
    const float tol = 1e-4F * params.K * std::max(1.0F, hs);

    EXPECT_NEAR(Lr_B.norm(), params.K * std::max(0.0F, hs - params.hsMin), tol);
    EXPECT_LE(Lr_B.norm(), params.K * hs + tol);

    // Above the deadband the torque must point against the stored momentum.
    if (hs > params.hsMin + (1e-3F * std::max(1.0F, hs))) {
        EXPECT_LT(Lr_B.normalized().dot(hs_B.normalized()), 0.0F);
    }
}

// Reversing every wheel speed reverses the requested torque. Every step is odd in the speeds (hs is even, so
// hsExcess is odd; the integral and its sign-preserving clamp are odd), so this holds with the integral engaged.
inline void testTorqueIsOddInWheelSpeeds(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                         const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                         const ThrMomentumManagementControlParameters& params,
                                         uint32_t numCycles = 1U) {
    ThrMomentumManagementAlgorithm forwardAlg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};
    ThrMomentumManagementAlgorithm reversedAlg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};
    const Eigen::Vector<float, kMaxNumRw> reversedSpeeds = -wheelSpeeds;

    Eigen::Vector3f forward = Eigen::Vector3f::Zero();
    Eigen::Vector3f reversed = Eigen::Vector3f::Zero();
    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        forward = forwardAlg.update(wheelSpeeds);
        reversed = reversedAlg.update(reversedSpeeds);
    }

    // Negating the speeds negates every intermediate exactly: hs_B flips sign componentwise, hs = hs_B.norm()
    // depends only on the component magnitudes so it is unchanged, and the clamp preserves sign. Hence this
    // holds bit-for-bit and needs no error budget.
    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_EQ(reversed[i], -forward[i]) << "component " << i;
    }
}

// Every request is finite for any admissible configuration, however many cycles run.
inline void testTorqueStaysFinite(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                  const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                  const ThrMomentumManagementControlParameters& params,
                                  uint32_t numCycles = 1U) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};

    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        EXPECT_TRUE(alg.update(wheelSpeeds).allFinite()) << "cycle " << cycle;
    }
}

// Anti-windup: however long a momentum is held, the integral term cannot contribute more than Ki * integralLimit
// to any component. This is the property the clamp exists to guarantee.
inline void testIntegralTermStaysBounded(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                         const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                         const ThrMomentumManagementControlParameters& params,
                                         uint32_t numCycles = 50U) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);
    const float hs = hs_B.norm();
    const Eigen::Vector3f hsExcess_B = (hs >= params.hsMin && hs >= 1e-6F)
                                           ? Eigen::Vector3f{(hs - params.hsMin) * hs_B / hs}
                                           : Eigen::Vector3f::Zero();

    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        const Eigen::Vector3f Lr_B = alg.update(wheelSpeeds);
        ASSERT_TRUE(Lr_B.allFinite()) << "cycle " << cycle;

        for (Eigen::Index i = 0; i < 3; ++i) {
            const float bound = params.K * std::fabs(hsExcess_B[i]) + params.Ki * params.integralLimit;
            const float tol = 1e-4F * std::max(1.0F, bound);
            EXPECT_LE(std::fabs(Lr_B[i]), bound + tol) << "cycle " << cycle << " component " << i;
        }
    }
}

// The algorithm's output after numCycles updates must match the independent reference implementation.
inline void regressionTestThrMomentumManagement(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                                const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                                const ThrMomentumManagementControlParameters& params,
                                                float accuracy,
                                                uint32_t numCycles = 1U) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};

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
                         const ThrMomentumManagementControlParameters& params,
                         ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
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

inline void propertyProportionalTorqueOpposesExcessMomentum(const Eigen::Vector3f& axis0,
                                                            const Eigen::Vector3f& axis1,
                                                            const Eigen::Vector3f& axis2,
                                                            const Eigen::Vector3f& speeds,
                                                            float js,
                                                            float hsMin,
                                                            float K) {
    const ThrMomentumManagementControlParameters params{
        .hsMin = hsMin, .K = K, .Ki = 0.0F, .integralLimit = 0.0F, .controlPeriod = 0.0F};
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    testProportionalTorqueOpposesExcessMomentum(rwArrayConfig, wheelSpeeds, params);
}

inline void propertyTorqueIsOddInWheelSpeeds(const Eigen::Vector3f& axis0,
                                             const Eigen::Vector3f& axis1,
                                             const Eigen::Vector3f& axis2,
                                             const Eigen::Vector3f& speeds,
                                             float js,
                                             float hsMin,
                                             float K,
                                             float Ki,
                                             float integralLimit,
                                             float controlPeriod,
                                             uint32_t numCycles) {
    const ThrMomentumManagementControlParameters params{
        .hsMin = hsMin, .K = K, .Ki = Ki, .integralLimit = integralLimit, .controlPeriod = controlPeriod};
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
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
                                      float js,
                                      float hsMin,
                                      float K,
                                      float Ki,
                                      float integralLimit,
                                      float controlPeriod,
                                      uint32_t numCycles) {
    const ThrMomentumManagementControlParameters params{
        .hsMin = hsMin, .K = K, .Ki = Ki, .integralLimit = integralLimit, .controlPeriod = controlPeriod};
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
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
                                             float js,
                                             float hsMin,
                                             float K,
                                             float Ki,
                                             float integralLimit,
                                             float controlPeriod) {
    const ThrMomentumManagementControlParameters params{
        .hsMin = hsMin, .K = K, .Ki = Ki, .integralLimit = integralLimit, .controlPeriod = controlPeriod};
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    testIntegralTermStaysBounded(rwArrayConfig, wheelSpeeds, params);
}

inline void regressionFuzzThrMomentumManagement(const Eigen::Vector3f& axis0,
                                                const Eigen::Vector3f& axis1,
                                                const Eigen::Vector3f& axis2,
                                                const Eigen::Vector3f& speeds,
                                                float js,
                                                float hsMin,
                                                float K,
                                                float Ki,
                                                float integralLimit,
                                                float controlPeriod,
                                                uint32_t numCycles) {
    const ThrMomentumManagementControlParameters params{
        .hsMin = hsMin, .K = K, .Ki = Ki, .integralLimit = integralLimit, .controlPeriod = controlPeriod};
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds;
    if (!detail::makeFuzzCase(axis0, axis1, axis2, speeds, js, params, rwArrayConfig, wheelSpeeds)) {
        return;
    }
    // The integral accumulates rounding once per cycle, so allow the error to grow with the cycle count.
    const float hs = clusterMomentum(rwArrayConfig, wheelSpeeds).norm();
    const float scale = K * std::max(1.0F, hs) + Ki * integralLimit;
    const float tol = 1e-4F * static_cast<float>(numCycles) * std::max(1.0F, scale);

    regressionTestThrMomentumManagement(rwArrayConfig, wheelSpeeds, params, tol, numCycles);
}

#endif
