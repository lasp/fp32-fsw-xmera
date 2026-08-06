#ifndef TEST_THR_MOMENTUM_MANAGEMENT_H
#define TEST_THR_MOMENTUM_MANAGEMENT_H

#include "thrMomentumManagementAlgorithm.h"
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <optional>
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
// description rather than from the algorithm source: accumulate the net RW momentum, and request the
// change that brings its magnitude down to hsMin. Returns nullopt when no dumping is required.
inline std::optional<Eigen::Vector3f> referenceDeltaH(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                                      const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                                      float hsMin) {
    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }
    const float hs = hs_B.norm();

    if (hs < hsMin || hs < 1e-6F) {
        return Eigen::Vector3f::Zero();
    }
    return Eigen::Vector3f{-(hs - hsMin) / hs * hs_B};
}

// Regression helper: the algorithm's first (armed) update must match the reference implementation.
inline void regressionTestThrMomentumManagement(const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                                const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds,
                                                float hsMin,
                                                float accuracy) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(hsMin, rwArrayConfig)};

    const std::optional<Eigen::Vector3f> actual = alg.update(wheelSpeeds);
    const std::optional<Eigen::Vector3f> expected = referenceDeltaH(rwArrayConfig, wheelSpeeds, hsMin);

    ASSERT_EQ(actual.has_value(), expected.has_value());
    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR((*actual)[i], (*expected)[i], accuracy) << "component " << i;
    }
}

// Config helper: assert that a (hsMin, rwArrayConfig) pair is accepted and round-trips through the getters.
inline void testThrMomentumManagementSetup(float hsMin,
                                           const ThrMomentumManagementRwArrayConfiguration& rwArrayConfig,
                                           float accuracy) {
    const ThrMomentumManagementConfig cfg = ThrMomentumManagementConfig::create(hsMin, rwArrayConfig);

    EXPECT_NEAR(cfg.getHsMin(), hsMin, accuracy);
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

#endif
