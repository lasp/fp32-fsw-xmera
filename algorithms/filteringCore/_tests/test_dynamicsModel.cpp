// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for filtering::rk4 and filtering::propagate.

#include <filteringCore/dynamicsModel.hpp>
#include <filteringCore/state.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <math.h>
#include <array>
#include <cmath>
#include <limits>

namespace filtering {

// Bound for comparing two float computations of the same quantity: a few operations of roundoff
// with a 3-10x margin (10 * epsilon ~ 1.2e-6).
inline constexpr float kRoundoffTol = 10.0F * std::numeric_limits<float>::epsilon();

namespace {

// Basic state for testing dynamics integration
struct ScalarState {
    float v = 0.0;
    ScalarState add(ScalarState const& other) const { return {v + other.v}; }
    ScalarState subtract(ScalarState const& other) const { return {v - other.v}; }
    ScalarState scale(float k) const { return {v * k}; }
};

static_assert(LinearlyCombinable<ScalarState>);

// Constant-rate dynamics for the simplest RK4 test.
struct ConstantRate {
    float c;
    ScalarState operator()(float, ScalarState const&) const { return ScalarState{c}; }
};

// dx/dt = a·t. Quadratic x(t); still in RK4's exactness range (degree ≤ 4).
struct TimeRamp {
    float a;
    ScalarState operator()(float t, ScalarState const&) const { return ScalarState{a * t}; }
};

// Spring mass damper for harmonic oscillator testing. The analytic period is T = 2π;
// conserved energy E = ½(x² + v²).
constexpr float kSHOPeriod = 2.0 * M_PI;

using SpringMassState = StateVector<Position<1>, Velocity<1>>;

struct SpringMassDyn {
    SpringMassState operator()(float, SpringMassState const& s) const {
        SpringMassState out;
        Eigen::Vector<float, 1> dx;
        dx(0) = s.get<Velocity<1>>()(0);
        Eigen::Vector<float, 1> dv;
        dv(0) = -s.get<Position<1>>()(0);
        out.set<Position<1>>(dx);
        out.set<Velocity<1>>(dv);
        return out;
    }
};

float energy(SpringMassState const& s) {
    float const x = s.get<Position<1>>()(0);
    float const v = s.get<Velocity<1>>()(0);
    return 0.5 * (x * x + v * v);
}

// Free motion: x' = v, v' = 0. Linear, so a deviation is integrated without truncation error.
struct FreeMotionDyn {
    SpringMassState operator()(float, SpringMassState const& s) const {
        SpringMassState out;
        out.set<Position<1>>(s.get<Velocity<1>>());
        out.set<Velocity<1>>(Eigen::Vector<float, 1>(0.0F));
        return out;
    }
};

// Pendulum: x' = v, v' = -sin(x). Nonlinear, for checking that a deviation is not linearized.
struct PendulumDyn {
    SpringMassState operator()(float, SpringMassState const& s) const {
        SpringMassState out;
        out.set<Position<1>>(s.get<Velocity<1>>());
        out.set<Velocity<1>>(Eigen::Vector<float, 1>(-sinf(s.get<Position<1>>()(0))));
        return out;
    }
};

SpringMassState makeSpringMassState(float x, float v) {
    SpringMassState s;
    s.set<Position<1>>(Eigen::Vector<float, 1>(x));
    s.set<Velocity<1>>(Eigen::Vector<float, 1>(v));
    return s;
}

}  // namespace

// Check zero propagation
TEST(Rk4AndPropagate, ReturnInputUnchangedAtZeroTime) {
    SpringMassState s;
    s.set<Position<1>>(Eigen::Vector<float, 1>(2.0));
    Eigen::Vector<float, 1> v0;
    v0(0) = -0.5;
    s.set<Velocity<1>>(v0);
    SpringMassDyn const d;

    EXPECT_TRUE(rk4(d, s, 1.5, 0.0).raw().isApprox(s.raw(), 1e-12)) << "rk4(dt=0) should be identity";
    EXPECT_TRUE(propagate(d, s, {5.0, 5.0}).raw().isApprox(s.raw(), 1e-12))
        << "propagate over zero-length interval should be identity";
}

// Check simple dynamics is correct
TEST(Rk4, IsExactForPolynomialDynamics) {
    // dx/dt = c leads to x(t) = x0 + ct
    {
        ScalarState const s0{10.0};
        ConstantRate const d{3.0};
        EXPECT_NEAR(rk4(d, s0, 0.0, 2.0).v, 16.0, 16.0 * kRoundoffTol) << "constant dx/dt = c";
    }
    // dx/dt = a·t leads to x (t)= 0.5 a t*t
    {
        ScalarState const s0{0.0};
        TimeRamp const d{2.0};
        EXPECT_NEAR(rk4(d, s0, 0.0, 3.0).v, 9.0, 9.0 * kRoundoffTol) << "linear-in-t dx/dt = a·t";
    }
}

// Test spring-mass (m, k =1 )
TEST(Propagate, HarmonicOscillatorInvariants) {
    SpringMassState s0;
    s0.set<Position<1>>(Eigen::Vector<float, 1>(1.0));  // x0 = 1, v0 = 0
    float const E0 = energy(s0);
    SpringMassDyn const d;

    // (1) Energy conserved over one full period.
    {
        SpringMassState const sFinal = propagate(d, s0, {0.0, kSHOPeriod});
        EXPECT_NEAR(energy(sFinal), E0, 1e-2) << "energy drift over one period";
    }
    // (2) Phase: state returns to (1, 0) after one full period.
    {
        SpringMassState const sFinal = propagate(d, s0, {0.0, kSHOPeriod});
        EXPECT_NEAR(sFinal.get<Position<1>>()(0), 1.0, 1e-2) << "x not at start";
        EXPECT_NEAR(sFinal.get<Velocity<1>>()(0), 0.0, 1e-2) << "v not at start";
    }
    // (3) Quarter period: (1, 0) → (0, -1). Pins direction of rotation.
    {
        SpringMassState const sFinal = propagate(d, s0, {0.0, kSHOPeriod / 4.0});
        EXPECT_NEAR(sFinal.get<Position<1>>()(0), 0.0, 5e-3) << "quarter-period x";
        EXPECT_NEAR(sFinal.get<Velocity<1>>()(0), -1.0, 5e-3) << "quarter-period v";
    }
}

// The optional integrationStep is honored: a finer sub-step integrates the harmonic
// oscillator more accurately
TEST(Propagate, IntegrationStepIsHonored) {
    SpringMassState s0;
    s0.set<Position<1>>(Eigen::Vector<float, 1>(1.0));  // x0 = 1, v0 = 0
    float const E0 = energy(s0);
    SpringMassDyn const d;

    // Steps stay under the kMaxNumberOfSteps cap over one period (T/0.1 ~ 63 < 100).
    float const coarseDrift = std::abs(energy(propagate(d, s0, {0.0, kSHOPeriod}, 1.0)) - E0);
    float const fineDrift = std::abs(energy(propagate(d, s0, {0.0, kSHOPeriod}, 0.1)) - E0);
    EXPECT_LT(fineDrift, coarseDrift) << "smaller integration step should integrate more accurately";

    // Passing the default explicitly equals omitting the argument.
    SpringMassState const withDefault = propagate(d, s0, {0.0, kSHOPeriod}, kIntegrationStep);
    SpringMassState const omitted = propagate(d, s0, {0.0, kSHOPeriod});
    EXPECT_TRUE(withDefault.raw().isApprox(omitted.raw(), 1e-15))
        << "default argument should match explicit kIntegrationStep";
}

// An increment of a few ulps added to a large state rounds the same way at every sub-step. Here one ulp of 5e5 is
// 0.03125 and each of the 100 sub-steps adds 2.002, which plain addition rounds to 2.0: a bias of 0.2 at the end.
// The Kahan compensation in propagate() adds the rounded-away part back, so the error stays at about one ulp.
TEST(Propagate, KahanCompensationRemovesTheRoundingBiasOfALargeState) {
    float const x0 = 5e5F;
    float const rate = 10.01F;
    float const span = 20.0F;  // 100 sub-steps of kIntegrationStep
    ScalarState const sFinal = propagate(ConstantRate{rate}, ScalarState{x0}, {0.0F, span});

    double const expected = static_cast<double>(x0) + static_cast<double>(rate) * static_cast<double>(span);
    float const ulp = nextafterf(x0, 1e6F) - x0;
    EXPECT_NEAR(static_cast<double>(sFinal.v), expected, 2.0 * ulp) << "sub-step rounding accumulated into a bias";
}

// A deviation from a large state, propagated on its own scale, keeps the bits that the absolute propagation of the
// sigma point loses. Here the position deviation grows by 2e-5 per sub-step, below one ulp of 5e5 (0.03125).
// The deviation rate is still formed as f(reference + deviation) - f(reference), which rounds the velocity
// deviation to one ulp of the reference velocity, so the position deviation is accurate to ulp(7) * span.
TEST(PropagateDeviation, KeepsASmallDeviationNextToALargeState) {
    SpringMassState const reference = makeSpringMassState(5e5F, 7.0F);
    SpringMassState const deviation = makeSpringMassState(0.0F, 1e-4F);
    FreeMotionDyn const d;
    std::array<float, 2> const interval = {0.0F, 20.0F};

    SpringMassState const propagated = propagateDeviation(d, reference, deviation, interval);
    float const rateUlp = nextafterf(7.0F, 8.0F) - 7.0F;
    EXPECT_NEAR(propagated.get<Position<1>>()(0), 2e-3F, rateUlp * interval[1]) << "position deviation";
    EXPECT_NEAR(propagated.get<Velocity<1>>()(0), 1e-4F, 1e-4F * kRoundoffTol) << "velocity deviation";

    // Check of the setup: the difference of the two absolute propagations loses the position deviation.
    SpringMassState const absolute =
        propagate(d, reference.add(deviation), interval).add(propagate(d, reference, interval).scale(-1.0F));
    EXPECT_GT(std::abs(absolute.get<Position<1>>()(0) - 2e-3F), 1e-3F)
        << "absolute propagation was expected to lose the deviation";
}

// propagateDeviation() does not linearize the dynamics: for a deviation that is not small, it gives the difference
// of the two absolute propagations of a nonlinear system, to float roundoff.
TEST(PropagateDeviation, MatchesAbsolutePropagationForNonlinearDynamics) {
    SpringMassState const reference = makeSpringMassState(1.0F, 0.0F);
    SpringMassState const deviation = makeSpringMassState(0.5F, -0.3F);
    PendulumDyn const d;
    std::array<float, 2> const interval = {0.0F, 5.0F};

    SpringMassState const propagated = propagateDeviation(d, reference, deviation, interval);
    SpringMassState const absolute =
        propagate(d, reference.add(deviation), interval).add(propagate(d, reference, interval).scale(-1.0F));
    EXPECT_TRUE(propagated.raw().isApprox(absolute.raw(), 1e-5F))
        << "deviation propagation differs from the absolute propagation: " << propagated.raw().transpose() << " vs "
        << absolute.raw().transpose();
}

}  // namespace filtering
