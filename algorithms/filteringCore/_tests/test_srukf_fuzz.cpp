// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz: measurementUpdate is a Kalman update — for any
// initial state, observation, and (P, R) configuration, the post-update
// state must move toward the observation (or not at all) and the
// covariance must remain symmetric PSD with non-increasing trace.

#include <utilities/fsw/validPSDCheck.h>
#include <filteringCore/srukf.hpp>
#include <filteringCore/state.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <limits>
#include <vector>

namespace filtering {
namespace {

using TestState = StateVector<Position<3>>;
struct ZeroDynamics {
    TestState operator()(float, TestState const& s) const { return s.scale(0.0); }
};
using SRuKFType = SRuKF<TestState, ZeroDynamics>;

struct PositionMeasurement {
    static constexpr int size = 3;
    Eigen::Vector3f observed = Eigen::Vector3f::Zero();
    Eigen::Matrix3f noiseCov = Eigen::Matrix3f::Identity();
    Eigen::Vector3f observation() const { return observed; }
    Eigen::Vector3f model(TestState const& s) const { return s.get<Position<3>>(); }
    Eigen::Matrix3f noise() const { return noiseCov; }
    Eigen::Vector3f subtract(Eigen::Vector3f const& a, Eigen::Vector3f const& b) const { return a - b; }
};

}  // namespace

// Vary only the x-component for state and observation. The Kalman is in the form
// K = P/(P+R), which lives in [0, 1]. We should see the covariance shrink
void fuzzMeasurementUpdateInvariants(float x0, float obs, float pDiag, float rDiag) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);

    TestState s0;
    s0.set<Position<3>>(Eigen::Vector3f(x0, 0.0, 0.0));
    filter.setInitialState(s0);
    filter.setInitialCovariance(pDiag * Eigen::Matrix3f::Identity());
    filter.setProcessNoise(Eigen::Matrix3f::Zero());
    filter.reInitialize();
    filter.timeUpdate(0.0);

    PositionMeasurement m;
    m.observed = Eigen::Vector3f(obs, 0.0, 0.0);
    m.noiseCov = rDiag * Eigen::Matrix3f::Identity();

    float const traceBefore = filter.getCovariance().trace();
    filter.measurementUpdate(m);
    float const newX = filter.getState().get<Position<3>>()(0);

    // Float rounding scales with the input magnitude, so the tolerances are relative to it.
    float const scale = std::max({1.0F, std::abs(x0), std::abs(obs)});

    // State moves toward observation, or not at all.
    EXPECT_GE((obs - x0) * (newX - x0), -1e-5F * scale * scale) << "state moved away from observation";
    EXPECT_LE(std::abs(newX - x0), std::abs(obs - x0) + 1e-5F * scale) << "state overshot";

    // Covariance stays symmetric PSD.
    Eigen::Matrix3f const P = filter.getCovariance();
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-5)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "covariance not PSD";

    // Covariance does not grow under a measurement update.
    EXPECT_LE(P.trace(), traceBefore * (1.0F + 1e-5F)) << "trace increased";
}
FUZZ_TEST(SrukfFuzz, fuzzMeasurementUpdateInvariants)
    .WithDomains(fuzztest::InRange(-1e3, 1e3),   // x0
                 fuzztest::InRange(-1e3, 1e3),   // obs
                 fuzztest::InRange(1e-3, 1e3),   // P_diag
                 fuzztest::InRange(1e-3, 1e3));  // R_diag

// For any lower-triangular factor (zeros and non-finite entries included), vector and weight, the rank-1 up/down date
// never evaluates a division by zero: a downdate that would divide by zero fails instead, for the caller to report a
// bad update. The inputs go through a volatile so that the compiler cannot fold the divisions away.
void fuzzCholeskyUpDownDateNeverDividesByZero(Eigen::Matrix3f S, Eigen::Vector3f v, float coef) {
    volatile float opaqueOne = 1.0F;
    float const one = opaqueOne;  // not a compile-time constant
    Eigen::Matrix3f const S0 = S.triangularView<Eigen::Lower>().toDenseMatrix() * one;
    std::feclearexcept(FE_DIVBYZERO);
    (void)SRuKFType::choleskyUpDownDate<3>(S0, v * one, coef * one);
    EXPECT_FALSE(std::fetestexcept(FE_DIVBYZERO)) << "division by zero";
}
FUZZ_TEST(SrukfFuzz, fuzzCholeskyUpDownDateNeverDividesByZero)
    .WithDomains(
        fuzztest::Map(
            [](std::vector<float> const& e) { return Eigen::Matrix3f(Eigen::Map<const Eigen::Matrix3f>(e.data())); },
            fuzztest::VectorOf(fuzztest::OneOf(fuzztest::InRange(-1e3F, 1e3F),
                                               fuzztest::ElementOf<float>({0.0F,
                                                                           -0.0F,
                                                                           1.0F,
                                                                           -1.0F,
                                                                           std::numeric_limits<float>::quiet_NaN(),
                                                                           std::numeric_limits<float>::infinity()})))
                .WithSize(9)),
        fuzztest::Map(
            [](std::vector<float> const& e) { return Eigen::Vector3f(Eigen::Map<const Eigen::Vector3f>(e.data())); },
            fuzztest::VectorOf(fuzztest::OneOf(fuzztest::InRange(-1e3F, 1e3F),
                                               fuzztest::ElementOf<float>({0.0F, 1.0F, -1.0F})))
                .WithSize(3)),
        fuzztest::OneOf(fuzztest::InRange(-4.0F, 4.0F),
                        fuzztest::ElementOf<float>({0.0F, -1.0F, 1.0F, std::numeric_limits<float>::quiet_NaN()})));

}  // namespace filtering
