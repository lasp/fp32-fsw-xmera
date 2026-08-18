#ifndef TEST_THR_MOMENTUM_MANAGEMENT_H
#define TEST_THR_MOMENTUM_MANAGEMENT_H

#include "thrMomentumManagementAlgorithm.h"
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
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

// Regression helper: the algorithm's output after numCycles updates must match the reference implementation.
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

// Fuzz property (proportional law, Ki = 0): for any admissible three-wheel geometry the torque is finite,
// opposes the stored momentum, and has magnitude K * (|hs| - hsMin) -- it acts on exactly the momentum held
// above the threshold, and vanishes when the cluster is already below it.
inline void propertyProportionalTorqueOpposesExcessMomentum(const Eigen::Vector3f& axis0,
                                                            const Eigen::Vector3f& axis1,
                                                            const Eigen::Vector3f& axis2,
                                                            const Eigen::Vector3f& speeds,
                                                            float js,
                                                            float hsMin,
                                                            float K,
                                                            float controlPeriod) {
    // A spin axis too short to normalize has no defined direction; the config would reject it.
    constexpr float degenerateTol = 1e-3F;
    if (axis0.norm() < degenerateTol || axis1.norm() < degenerateTol || axis2.norm() < degenerateTol) {
        return;
    }

    const auto rwArrayConfig = makeRwArrayConfig({axis0, axis1, axis2}, js);
    const auto wheelSpeeds = makeWheelSpeeds({speeds[0], speeds[1], speeds[2]});

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(
        {.hsMin = hsMin, .K = K, .Ki = 0.0F, .integralLimit = 0.0F, .controlPeriod = controlPeriod}, rwArrayConfig)};
    const Eigen::Vector3f Lr_B = alg.update(wheelSpeeds);

    ASSERT_TRUE(Lr_B.allFinite());

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);
    const float hs = hs_B.norm();

    // Negligible momentum takes the zero-tolerance branch, which deliberately declines to dump; that
    // carve-out is pinned by the edge-case unit tests instead.
    if (hs < 1e-4F) {
        return;
    }

    // FP32 error grows with the momentum magnitude and is amplified by the gain.
    const float tol = 1e-4F * K * std::max(1.0F, hs);
    EXPECT_NEAR(Lr_B.norm(), K * std::max(0.0F, hs - hsMin), tol);

    // Above the deadband the torque must point against the stored momentum.
    if (hs > hsMin + (1e-3F * std::max(1.0F, hs))) {
        EXPECT_LT(Lr_B.normalized().dot(hs_B.normalized()), 0.0F);
    }
}

// Fuzz property (anti-windup): however long a sustained momentum is held, the integral term can never
// contribute more than Ki * integralLimit to any torque component, so the total request stays bounded by the
// proportional part plus that clamp. This is the property the anti-windup clamp exists to guarantee.
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
    constexpr float degenerateTol = 1e-3F;
    if (axis0.norm() < degenerateTol || axis1.norm() < degenerateTol || axis2.norm() < degenerateTol) {
        return;
    }

    // An active integral needs a positive step; the config rejects that pair rather than running it.
    if (Ki > 0.0F && controlPeriod <= 0.0F) {
        return;
    }

    const auto rwArrayConfig = makeRwArrayConfig({axis0, axis1, axis2}, js);
    const auto wheelSpeeds = makeWheelSpeeds({speeds[0], speeds[1], speeds[2]});
    const ThrMomentumManagementControlParameters params{
        .hsMin = hsMin, .K = K, .Ki = Ki, .integralLimit = integralLimit, .controlPeriod = controlPeriod};

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);
    const float hs = hs_B.norm();
    const Eigen::Vector3f hsExcess_B =
        (hs >= hsMin && hs >= 1e-6F) ? Eigen::Vector3f{(hs - hsMin) / hs * hs_B} : Eigen::Vector3f::Zero();

    // Run well past the point where an unclamped integral would have blown through the limit.
    for (uint32_t cycle = 0U; cycle < 50U; ++cycle) {
        const Eigen::Vector3f Lr_B = alg.update(wheelSpeeds);
        ASSERT_TRUE(Lr_B.allFinite()) << "cycle " << cycle;

        for (Eigen::Index i = 0; i < 3; ++i) {
            const float bound = K * std::fabs(hsExcess_B[i]) + Ki * integralLimit;
            const float tol = 1e-4F * std::max(1.0F, bound);
            EXPECT_LE(std::fabs(Lr_B[i]), bound + tol) << "cycle " << cycle << " component " << i;
        }
    }
}

// Fuzz regression: the algorithm must agree with the reference implementation on any admissible input.
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
    constexpr float degenerateTol = 1e-3F;
    if (axis0.norm() < degenerateTol || axis1.norm() < degenerateTol || axis2.norm() < degenerateTol) {
        return;
    }

    // An active integral needs a positive step; the config rejects that pair rather than running it.
    if (Ki > 0.0F && controlPeriod <= 0.0F) {
        return;
    }

    const auto rwArrayConfig = makeRwArrayConfig({axis0, axis1, axis2}, js);
    const auto wheelSpeeds = makeWheelSpeeds({speeds[0], speeds[1], speeds[2]});
    const ThrMomentumManagementControlParameters params{
        .hsMin = hsMin, .K = K, .Ki = Ki, .integralLimit = integralLimit, .controlPeriod = controlPeriod};

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(params, rwArrayConfig)};
    Eigen::Vector3f actual = Eigen::Vector3f::Zero();
    for (uint32_t cycle = 0U; cycle < numCycles; ++cycle) {
        actual = alg.update(wheelSpeeds);
    }
    const Eigen::Vector3f expected = referenceTorque(rwArrayConfig, wheelSpeeds, params, numCycles);

    // The integral accumulates rounding once per cycle, so allow the error to grow with the cycle count.
    const float hs = clusterMomentum(rwArrayConfig, wheelSpeeds).norm();
    const float scale = K * std::max(1.0F, hs) + Ki * integralLimit;
    const float tol = 1e-4F * static_cast<float>(numCycles) * std::max(1.0F, scale);
    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], tol) << "component " << i;
    }
}

#endif
