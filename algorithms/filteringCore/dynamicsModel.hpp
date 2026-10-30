// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_DYNAMICS_MODEL_HPP
#define FILTERING_CORE_DYNAMICS_MODEL_HPP

#include <filteringCore/concepts.hpp>

#include <math.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

namespace filtering {

/*! Runge-Kutta 4 increment, kept separate from the state so callers can accumulate it without
 *  absorbing it into a much larger state.
 *  @return x(t0 + dt) - x(t0)
 *  @param dynamics [-] callable (t, x) -> dx/dt
 *  @param X0       [-] state at time t0
 *  @param t0       [s] start time
 *  @param dt       [s] step size */
template <LinearlyCombinable State, Dynamics<State> D>
constexpr State rk4Increment(D const& dynamics, State const& X0, float t0, float dt) {
    auto k1 = dynamics(t0, X0);
    auto k2 = dynamics(t0 + dt / 2.0F, X0.add(k1.scale(dt / 2.0F)));
    auto k3 = dynamics(t0 + dt / 2.0F, X0.add(k2.scale(dt / 2.0F)));
    auto k4 = dynamics(t0 + dt, X0.add(k3.scale(dt)));

    return k1.scale(dt / 6.0F).add(k2.scale(dt / 3.0F)).add(k3.scale(dt / 3.0F)).add(k4.scale(dt / 6.0F));
}

/*! Runge-Kutta 4
 *  @return state advanced by dt
 *  @param dynamics [-] callable (t, x) -> dx/dt
 *  @param X0       [-] state at time t0
 *  @param t0       [s] start time
 *  @param dt       [s] step size */
template <LinearlyCombinable State, Dynamics<State> D>
constexpr State rk4(D const& dynamics, State const& X0, float t0, float dt) {
    return X0.add(rk4Increment(dynamics, X0, t0, dt));
}

/*! Fixed RK4 max steps used by `propagate`. */
inline constexpr unsigned int kMaxNumberOfSteps = 100;

/*! Default RK4 sub-step size used by `propagate` when none is supplied. */
inline constexpr float kIntegrationStep = 0.2F;

/*! Number of RK4 sub-steps for an interval, and the (possibly stretched) step size: at most
 *  kMaxNumberOfSteps, so a long interval is integrated coarsely rather than slowly.
 *  @return {number of steps, step size}
 *  @param interval        [s,s] {start, end} time pair
 *  @param integrationStep [s] requested integration time step */
inline std::pair<unsigned int, float> integrationSteps(std::array<float, 2> interval, float integrationStep) {
    float const span = interval[1] - interval[0];
    auto numberOfSteps = static_cast<unsigned int>(ceilf(span / integrationStep));
    if (numberOfSteps > kMaxNumberOfSteps) {
        integrationStep = span / static_cast<float>(kMaxNumberOfSteps);
        numberOfSteps = kMaxNumberOfSteps;
    }
    return {numberOfSteps, integrationStep};
}

/*! Propagate state across a time interval in RK4 sub-steps. The sub-step increments are accumulated
 *  with Kahan compensation: in float an increment is often only a few ulps of the state (e.g. a
 *  position of 1e5 km advancing a few km per step), and plain addition rounds it the same way every
 *  step, a bias that grows linearly with the number of steps.
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
    auto const [numberOfSteps, step] = integrationSteps(interval, integrationStep);

    State compensation{};
    for (unsigned int i = 0; i < numberOfSteps; ++i) {
        float const h = std::min(step, tFinal - t);
        State const correctedIncrement = rk4Increment(dynamics, state, t, h).subtract(compensation);
        State const next = state.add(correctedIncrement);
        compensation = next.subtract(state).subtract(correctedIncrement);
        state = next;
        t += h;
    }

    return state;
}

/*! Propagate a reference state and small deviations from it. Each deviation obeys d' = f(c + d) - f(c)
 *  and is integrated on its own scale, alongside the reference c, rather than as the difference of two
 *  propagated states: in float that difference keeps only the few bits of d that survive next to |c|,
 *  which collapses the spread of the sigma points (and so the covariance) for a large state. The
 *  reference is integrated once, exactly as propagate() does it (RK4 with Kahan-compensated
 *  accumulation), and each of its RK4 stages is shared by every deviation: a sub-step costs 4 + 4 * M
 *  dynamics calls.
 *  @return reference state at interval[1], equal to propagate(dynamics, reference, interval, integrationStep)
 *  @param dynamics   [-]   callable (t, x) -> dx/dt
 *  @param reference  [-]   reference state at interval[0]
 *  @param deviations [-]   deviations from the reference at interval[0], replaced by the deviations at interval[1]
 *  @param interval   [s,s] {start, end} time pair
 *  @param integrationStep [s] integration time step (optional, defaults to kIntegrationStep) */
template <LinearlyCombinable State, Dynamics<State> D, std::size_t M>
constexpr State propagateDeviations(D const& dynamics,
                                    State reference,
                                    std::array<State, M>& deviations,
                                    std::array<float, 2> interval,
                                    float integrationStep = kIntegrationStep) {
    float t = interval[0];
    float const tFinal = interval[1];
    auto const [numberOfSteps, step] = integrationSteps(interval, integrationStep);

    State compensation{};
    for (unsigned int i = 0; i < numberOfSteps; ++i) {
        float const h = std::min(step, tFinal - t);
        float const tMid = t + h / 2.0F;

        auto const k1 = dynamics(t, reference);
        auto const ref2 = reference.add(k1.scale(h / 2.0F));
        auto const k2 = dynamics(tMid, ref2);
        auto const ref3 = reference.add(k2.scale(h / 2.0F));
        auto const k3 = dynamics(tMid, ref3);
        auto const ref4 = reference.add(k3.scale(h));
        auto const k4 = dynamics(t + h, ref4);

        //! > Each deviation reuses the stages of the reference
        for (State& deviation : deviations) {
            auto const j1 = dynamics(t, reference.add(deviation)).subtract(k1);
            auto const j2 = dynamics(tMid, ref2.add(deviation.add(j1.scale(h / 2.0F)))).subtract(k2);
            auto const j3 = dynamics(tMid, ref3.add(deviation.add(j2.scale(h / 2.0F)))).subtract(k3);
            auto const j4 = dynamics(t + h, ref4.add(deviation.add(j3.scale(h)))).subtract(k4);
            deviation = deviation.add(
                j1.scale(h / 6.0F).add(j2.scale(h / 3.0F)).add(j3.scale(h / 3.0F)).add(j4.scale(h / 6.0F)));
        }

        //! > Kahan-compensated accumulation of the reference increment, as in propagate()
        State const increment =
            k1.scale(h / 6.0F).add(k2.scale(h / 3.0F)).add(k3.scale(h / 3.0F)).add(k4.scale(h / 6.0F));
        State const correctedIncrement = increment.subtract(compensation);
        State const next = reference.add(correctedIncrement);
        compensation = next.subtract(reference).subtract(correctedIncrement);
        reference = next;
        t += h;
    }

    return reference;
}

}  // namespace filtering

#endif
