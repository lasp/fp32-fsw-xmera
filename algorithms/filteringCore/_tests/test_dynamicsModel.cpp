// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for filtering::rk4 and filtering::propagate.

#include <filteringCore/dynamicsModel.hpp>
#include <filteringCore/state.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <math.h>
#include <cmath>

namespace filtering {
namespace {

// Basic state for testing dynamics integration
struct ScalarState {
    double v = 0.0;
    ScalarState add(ScalarState const& other) const { return {v + other.v}; }
    ScalarState scale(double k) const { return {v * k}; }
};

static_assert(LinearlyCombinable<ScalarState>);

// Constant-rate dynamics for the simplest RK4 test.
struct ConstantRate {
    double c;
    ScalarState operator()(double, ScalarState const&) const { return ScalarState{c}; }
};

// dx/dt = a·t. Quadratic x(t); still in RK4's exactness range (degree ≤ 4).
struct TimeRamp {
    double a;
    ScalarState operator()(double t, ScalarState const&) const { return ScalarState{a * t}; }
};

// Spring mass damper for harmonic oscillator testing. The analytic period is T = 2π;
// conserved energy E = ½(x² + v²).
constexpr double kSHOPeriod = 2.0 * M_PI;

using SpringMassState = StateVector<Position<1>, Velocity<1>>;

struct SpringMassDyn {
    SpringMassState operator()(double, SpringMassState const& s) const {
        SpringMassState out;
        Eigen::Vector<double, 1> dx;
        dx(0) = s.get<Velocity<1>>()(0);
        Eigen::Vector<double, 1> dv;
        dv(0) = -s.get<Position<1>>()(0);
        out.set<Position<1>>(dx);
        out.set<Velocity<1>>(dv);
        return out;
    }
};

double energy(SpringMassState const& s) {
    double const x = s.get<Position<1>>()(0);
    double const v = s.get<Velocity<1>>()(0);
    return 0.5 * (x * x + v * v);
}

}  // namespace

// Check zero propagation
TEST(Rk4AndPropagate, ReturnInputUnchangedAtZeroTime) {
    SpringMassState s;
    s.set<Position<1>>(Eigen::Vector<double, 1>(2.0));
    Eigen::Vector<double, 1> v0;
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
        EXPECT_NEAR(rk4(d, s0, 0.0, 2.0).v, 16.0, 1e-12) << "constant dx/dt = c";
    }
    // dx/dt = a·t leads to x (t)= 0.5 a t*t
    {
        ScalarState const s0{0.0};
        TimeRamp const d{2.0};
        EXPECT_NEAR(rk4(d, s0, 0.0, 3.0).v, 9.0, 1e-12) << "linear-in-t dx/dt = a·t";
    }
}

// Test spring-mass (m, k =1 )
TEST(Propagate, HarmonicOscillatorInvariants) {
    SpringMassState s0;
    s0.set<Position<1>>(Eigen::Vector<double, 1>(1.0));  // x0 = 1, v0 = 0
    double const E0 = energy(s0);
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
// oscillator more accurately (less energy drift) than a coarse one, and passing the
// default explicitly matches omitting the argument.
TEST(Propagate, IntegrationStepIsHonored) {
    SpringMassState s0;
    s0.set<Position<1>>(Eigen::Vector<double, 1>(1.0));  // x0 = 1, v0 = 0
    double const E0 = energy(s0);
    SpringMassDyn const d;

    // Steps stay under the kMaxNumberOfSteps cap over one period (T/0.1 ~ 63 < 100).
    double const coarseDrift = std::abs(energy(propagate(d, s0, {0.0, kSHOPeriod}, 1.0)) - E0);
    double const fineDrift = std::abs(energy(propagate(d, s0, {0.0, kSHOPeriod}, 0.1)) - E0);
    EXPECT_LT(fineDrift, coarseDrift) << "smaller integration step should integrate more accurately";

    // Passing the default explicitly equals omitting the argument.
    SpringMassState const withDefault = propagate(d, s0, {0.0, kSHOPeriod}, kIntegrationStep);
    SpringMassState const omitted = propagate(d, s0, {0.0, kSHOPeriod});
    EXPECT_TRUE(withDefault.raw().isApprox(omitted.raw(), 1e-15))
        << "default argument should match explicit kIntegrationStep";
}

}  // namespace filtering
