// Property-based fuzz tests for FlybyFilterAlgorithm. Over reasonable, randomized filter
// parameters / initial states / measurements, the filter must return valid, finite, symmetric-PSD
// results from time and measurement updates, and its time-update mean must follow the two-body flow.
//
// All quantities are in the filter's internal units (km, km/s).

#include "flybyFilterAlgorithm.h"
#include "flybyFilterSpecs.h"

#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/dynamicsModel.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

namespace filtering::flybyFilter {
namespace {

using State = FlybyFilterAlgorithm::State;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// A base flyby geometry the fuzzed offsets perturb, keeping |r| well away from zero.
Eigen::Vector3d baseR() { return {3000.0, 1000.0, 500.0}; }
Eigen::Vector3d baseV() { return {1.0, -2.0, 0.5}; }

State makeState(Eigen::Vector3d const& r, Eigen::Vector3d const& v) {
    State s;
    s.set<filtering::Position<3>>(r);
    s.set<filtering::Velocity<3>>(v);
    return s;
}

Matrix6 diagCovariance(double posVar, double velVar) {
    Eigen::Matrix<double, 6, 1> d;
    d << posVar, posVar, posVar, velVar, velVar, velVar;
    return d.asDiagonal();
}

FlybyFilterConfig makeConfig(double alpha,
                             double beta,
                             double mu,
                             double posStd,
                             double velStd,
                             double q,
                             double headingStd,
                             State const& initial) {
    return FlybyFilterConfig::create(alpha,
                                     beta,
                                     mu,
                                     Matrix6::Identity() * q,
                                     initial,
                                     diagCovariance(posStd * posStd, velStd * velStd),
                                     headingStd);
}

bool finiteSymmetricPsd(Matrix6 const& P) {
    return P.allFinite() && P.isApprox(P.transpose(), 1E-8) && isPositiveSemiDefinite<6>(P);
}

}  // namespace

// ============================================================================
// Generic regression: a time update + a heading measurement update stay valid, finite, symmetric, PSD.
// ============================================================================
void fuzzTimeAndMeasurementUpdates(double alpha,
                                   double beta,
                                   double mu,
                                   double posStd,
                                   double velStd,
                                   double q,
                                   double headingStd,
                                   double r0x,
                                   double r0y,
                                   double r0z,
                                   double v0x,
                                   double v0y,
                                   double v0z,
                                   double hx,
                                   double hy,
                                   double hz,
                                   double dt) {
    State const initial = makeState(baseR() + Eigen::Vector3d(r0x, r0y, r0z), baseV() + Eigen::Vector3d(v0x, v0y, v0z));
    FlybyFilterAlgorithm algo(makeConfig(alpha, beta, mu, posStd, velStd, q, headingStd, initial));

    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance())) << "covariance after timeUpdate";

    // A heading that is never degenerate (the base x-direction keeps the norm >= 0.5).
    Eigen::Vector3d rhat = Eigen::Vector3d(1.0, 0.0, 0.0) + Eigen::Vector3d(hx, hy, hz);
    rhat.normalize();
    HeadingMeasurement m;
    m.timeTag = 0.0;
    m.rhat_BN_N = rhat;
    m.covar = (headingStd * headingStd + 1E-12) * Eigen::Matrix3d::Identity();
    m.valid = true;
    algo.measurementUpdate(m);

    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance())) << "covariance after measurementUpdate";
    EXPECT_TRUE(algo.getState().raw().allFinite()) << "state after measurementUpdate";
}
FUZZ_TEST(FlybyFilterFuzz, fuzzTimeAndMeasurementUpdates)
    .WithDomains(fuzztest::InRange(1e-2, 1.0 - 1e-9),  // alpha in (0, 1)
                 fuzztest::InRange(0.0, 2.0),          // beta
                 fuzztest::InRange(1e3, 1e5),          // mu [km^3/s^2]
                 fuzztest::InRange(1.0, 3e2),          // initial position std [km]
                 fuzztest::InRange(1e-3, 1.0),         // initial velocity std [km/s]
                 fuzztest::InRange(0.0, 1e-4),         // process noise
                 fuzztest::InRange(1e-5, 1e-1),        // heading measurement std
                 fuzztest::InRange(-500.0, 500.0),     // initial r offset x/y/z [km]
                 fuzztest::InRange(-500.0, 500.0),
                 fuzztest::InRange(-500.0, 500.0),
                 fuzztest::InRange(-1.0, 1.0),  // initial v offset x/y/z [km/s]
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-0.5, 0.5),  // heading observation offset x/y/z
                 fuzztest::InRange(-0.5, 0.5),
                 fuzztest::InRange(-0.5, 0.5),
                 fuzztest::InRange(0.0, 60.0));  // dt [s]

// ============================================================================
// Targeted: a time update advances the mean along the two-body flow and adds process noise to the
// covariance (no smaller than a noise-free propagation; stays symmetric + PSD + finite).
// ============================================================================
void fuzzTimeUpdatePropagatesStateAndGrowsCovariance(double alpha,
                                                     double beta,
                                                     double mu,
                                                     double posStd,
                                                     double velStd,
                                                     double q,
                                                     double r0x,
                                                     double r0y,
                                                     double r0z,
                                                     double v0x,
                                                     double v0y,
                                                     double v0z,
                                                     double dt) {
    State const initial = makeState(baseR() + Eigen::Vector3d(r0x, r0y, r0z), baseV() + Eigen::Vector3d(v0x, v0y, v0z));
    FlybyFilterAlgorithm algo(makeConfig(alpha, beta, mu, posStd, velStd, q, 1e-4, initial));

    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";

    // The central sigma point (== reported state) follows the same two-body propagation.
    State const predicted = filtering::propagate(FlybyDynamics{mu}, initial, {0.0, dt});
    EXPECT_TRUE(algo.getState().raw().isApprox(predicted.raw(), 1e-9)) << "state must follow the two-body flow";

    Matrix6 const P = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(P)) << "covariance after timeUpdate";

    // Compare against the same propagation with no process noise: P(withQ) = P(noQ) + Q.
    FlybyFilterAlgorithm noiseFree(makeConfig(alpha, beta, mu, posStd, velStd, 0.0, 1e-4, initial));
    ASSERT_TRUE(noiseFree.timeUpdate(dt)) << "noise-free timeUpdate should be valid";
    EXPECT_GE(P.trace(), noiseFree.getCovariance().trace() - 1e-6) << "process noise should not shrink the covariance";
}
FUZZ_TEST(FlybyFilterFuzz, fuzzTimeUpdatePropagatesStateAndGrowsCovariance)
    .WithDomains(fuzztest::InRange(1e-2, 1.0 - 1e-9),  // alpha
                 fuzztest::InRange(0.0, 2.0),          // beta
                 fuzztest::InRange(1e3, 1e5),          // mu
                 fuzztest::InRange(1.0, 3e2),          // position std
                 fuzztest::InRange(1e-3, 1.0),         // velocity std
                 fuzztest::InRange(0.0, 1e-4),         // process noise
                 fuzztest::InRange(-500.0, 500.0),     // initial r offset x/y/z
                 fuzztest::InRange(-500.0, 500.0),
                 fuzztest::InRange(-500.0, 500.0),
                 fuzztest::InRange(-1.0, 1.0),  // initial v offset x/y/z
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(-1.0, 1.0),
                 fuzztest::InRange(0.0, 60.0));  // dt

}  // namespace filtering::flybyFilter
