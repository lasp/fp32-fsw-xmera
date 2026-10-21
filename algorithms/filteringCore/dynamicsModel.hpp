// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_DYNAMICS_MODEL_HPP
#define FILTERING_CORE_DYNAMICS_MODEL_HPP

#include <filteringCore/concepts.hpp>

#include <math.h>
#include <algorithm>
#include <array>

namespace filtering {

/*! Runge-Kutta 4
 *  @return state advanced by dt
 *  @param dynamics [-] callable (t, x) -> dx/dt
 *  @param X0       [-] state at time t0
 *  @param t0       [s] start time
 *  @param dt       [s] step size */
template <LinearlyCombinable State, Dynamics<State> D>
constexpr State rk4(D const& dynamics, State const& X0, float t0, float dt) {
    auto k1 = dynamics(t0, X0);
    auto k2 = dynamics(t0 + dt / 2.0F, X0.add(k1.scale(dt / 2.0F)));
    auto k3 = dynamics(t0 + dt / 2.0F, X0.add(k2.scale(dt / 2.0F)));
    auto k4 = dynamics(t0 + dt, X0.add(k3.scale(dt)));

    return X0.add(k1.scale(dt / 6.0F).add(k2.scale(dt / 3.0F)).add(k3.scale(dt / 3.0F)).add(k4.scale(dt / 6.0F)));
}

/*! Fixed RK4 max steps used by `propagate`. */
inline constexpr unsigned int kMaxNumberOfSteps = 100;

/*! Default RK4 sub-step size used by `propagate` when none is supplied. */
inline constexpr float kIntegrationStep = 0.2F;

/*! Propagate state across a time interval in RK4 sub-steps.
 *  @return state at interval[1]
 *  @param dynamics [-]   callable (t, x) -> dx/dt
 *  @param state    [-]   state at interval[0]
 *  @param interval [s,s] {start, end} time pair
 *  @param integrationStep [s] integration time step (optional, defaults to kIntegrationStep) */
template <LinearlyCombinable State, Dynamics<State> D>
constexpr State propagate(D const& dynamics,
                          State state,
                          std::array<float, 2> interval,
                          float integrationStep = kIntegrationStep) {
    float t = interval[0];
    float const tFinal = interval[1];

    unsigned int N = static_cast<unsigned int>(ceilf((tFinal - t) / integrationStep));
    if (N > kMaxNumberOfSteps) {
        integrationStep = (tFinal - t) / static_cast<float>(kMaxNumberOfSteps);
        N = kMaxNumberOfSteps;
    }
    for (unsigned int i = 0; i < N; ++i) {
        float const step = std::min(integrationStep, tFinal - t);
        state = rk4(dynamics, state, t, step);
        t += step;
    }

    return state;
}

}  // namespace filtering

#endif
