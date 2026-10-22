// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz test: undamped spring-mass oscillator
// (m·x'' = -k·x, i.e. x' = v, v' = -(k/m)·x) conserves energy
// E = ½(k·x² + m·v²) over one period T = 2π·√(m/k), for any initial
// conditions and any (mass, spring constant).

#include <filteringCore/dynamicsModel.hpp>
#include <filteringCore/state.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <math.h>

namespace filtering {
namespace {

using SpringMassState = StateVector<Position<1>, Velocity<1>>;

// Undamped spring-mass oscillator parameterized by mass m and spring constant k.
struct SpringMassDynamics {
    float m;  // [kg]  mass
    float k;  // [N/m] spring constant
    SpringMassState operator()(float, SpringMassState const& s) const {
        SpringMassState out;
        Eigen::Vector<float, 1> dx;
        dx(0) = s.get<Velocity<1>>()(0);
        Eigen::Vector<float, 1> dv;
        dv(0) = -(k / m) * s.get<Position<1>>()(0);
        out.set<Position<1>>(dx);
        out.set<Velocity<1>>(dv);
        return out;
    }
};

float energy(SpringMassState const& s, float m, float k) {
    float const x = s.get<Position<1>>()(0);
    float const v = s.get<Velocity<1>>()(0);
    return 0.5 * (k * x * x + m * v * v);
}

}  // namespace

// For any initial (x0, v0) and any (m, k), propagating one period through the
// undamped spring-mass oscillator preserves E = ½(k·x² + m·v²) up to RK4's
// secular drift.
void fuzzEnergyConservedOverOnePeriod(float x0, float v0, float m, float k) {
    SpringMassState s;
    Eigen::Vector<float, 1> px;
    px(0) = x0;
    Eigen::Vector<float, 1> pv;
    pv(0) = v0;
    s.set<Position<1>>(px);
    s.set<Velocity<1>>(pv);

    float const E0 = energy(s, m, k);

    SpringMassDynamics const d{m, k};
    float const period = 2.0 * M_PI * sqrt(m / k);
    SpringMassState const sFinal = propagate(d, s, {0.0, period});

    // Relative tolerance: the fixed RK4 sub-step makes absolute drift scale
    // with the energy and with (ω·dt); the floor covers near-zero energies.
    EXPECT_NEAR(energy(sFinal, m, k), E0, 0.05 * E0 + 1e-5);
}
FUZZ_TEST(DynamicsModelFuzz, fuzzEnergyConservedOverOnePeriod)
    .WithDomains(fuzztest::InRange(-1e3, 1e3),
                 fuzztest::InRange(-1e3, 1e3),
                 fuzztest::InRange(0.25, 4.0),
                 fuzztest::InRange(0.25, 4.0));

}  // namespace filtering
