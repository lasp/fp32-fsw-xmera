// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Compile-time "test" for the filteringCore concepts and the StateVector
// compile-time machinery. There is nothing to run: every check below is a
// static_assert, so a successful BUILD of this translation unit *is* the
// passing test. The trivial main() exists only so the build produces a target
// the suite can list alongside the runtime tests.
//
// Covers:
//   * LinearlyCombinable / FilterState         (concepts.hpp)
//   * Measurement<M, State>                     (concepts.hpp)
//   * Dynamics<D, State>                        (concepts.hpp)
//   * SequentialFilter<Filter, Measurement>     (kalmanFilter.hpp)
//   * StateVector offset / contains / size      (state.hpp)
//
// Each concept is pinned with a positive case (a type that must satisfy it)
// and a negative near-miss (a type that must NOT), so a regression in either
// direction breaks the build.

#include <filteringCore/concepts.hpp>
#include <filteringCore/kalmanFilter.hpp>
#include <filteringCore/srukf.hpp>
#include <filteringCore/state.hpp>

#include <Eigen/Core>

namespace filtering {
namespace {

// Representative types
using TestState = StateVector<Position<3>, Velocity<3>, Bias<1>>;

struct ZeroDynamics {
    TestState operator()(float, TestState const& s) const { return s.scale(0.0); }
};

struct PositionMeasurement {
    static constexpr int size = 3;
    Eigen::Vector3f observed = Eigen::Vector3f::Zero();
    Eigen::Matrix3f noiseCov = Eigen::Matrix3f::Identity();
    Eigen::Vector3f observation() const { return observed; }
    Eigen::Vector3f model(TestState const& s) const { return s.get<Position<3>>(); }
    Eigen::Matrix3f noise() const { return noiseCov; }
    Eigen::Vector3f subtract(Eigen::Vector3f const& a, Eigen::Vector3f const& b) const { return a - b; }
};

// add() but no scale() -> not LinearlyCombinable.
struct NotCombinable {
    NotCombinable add(NotCombinable const&) const { return *this; }
};

// LinearlyCombinable, but no `size` / `raw()` -> not a FilterState.
struct CombinableButNotState {
    CombinableButNotState scale(float) const { return *this; }
    CombinableButNotState add(CombinableButNotState const&) const { return *this; }
};

// Measurement missing noise() -> not a Measurement.
struct MeasurementNoNoise {
    static constexpr int size = 3;
    Eigen::Vector3f observation() const { return Eigen::Vector3f::Zero(); }
    Eigen::Vector3f model(TestState const&) const { return Eigen::Vector3f::Zero(); }
    Eigen::Vector3f subtract(Eigen::Vector3f const& a, Eigen::Vector3f const& b) const { return a - b; }
};

// Returns float instead of State -> not Dynamics<.., TestState>.
struct NotDynamics {
    float operator()(float, TestState const&) const { return 0.0; }
};

// LinearlyCombinable
static_assert(LinearlyCombinable<TestState>);
static_assert(LinearlyCombinable<StateVector<Position<1>, Velocity<1>>>);
static_assert(!LinearlyCombinable<NotCombinable>);

// FilterState
static_assert(FilterState<TestState>);
static_assert(!FilterState<CombinableButNotState>);

// Measurement
static_assert(Measurement<PositionMeasurement, TestState>);
static_assert(!Measurement<MeasurementNoNoise, TestState>);

// Dynamics
static_assert(Dynamics<ZeroDynamics, TestState>);
static_assert(!Dynamics<NotDynamics, TestState>);
static_assert(!Dynamics<int, TestState>);

// SequentialFilter
static_assert(SequentialFilter<SRuKF<TestState, ZeroDynamics>, PositionMeasurement>);

// StateVector compile-time machinery (moved from test_state.cpp) -
static_assert(TestState::size == 7, "size should sum component widths");

// Offsets accumulate the widths of preceding components.
static_assert(detail::offsetOf<Position<3>, Position<3>, Velocity<3>, Bias<1>>::value == 0);
static_assert(detail::offsetOf<Velocity<3>, Position<3>, Velocity<3>, Bias<1>>::value == 3);
static_assert(detail::offsetOf<Bias<1>, Position<3>, Velocity<3>, Bias<1>>::value == 6);

// `contains` gates the requires-clause on get<>/set<>.
static_assert(detail::contains<Position<3>, Position<3>, Velocity<3>, Bias<1>>);
static_assert(!detail::contains<Acceleration<3>, Position<3>, Velocity<3>, Bias<1>>);

}  // namespace
}  // namespace filtering

// The static_asserts above are the test.
int main() { return 0; }
