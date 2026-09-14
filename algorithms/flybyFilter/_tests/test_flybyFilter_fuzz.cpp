// Property-based fuzz tests for FlybyFilterAlgorithm. Each FUZZ_TEST drives one of the property
// helpers in flybyFilterTestHelpers.hpp over randomized inputs; the helpers guard unusable samples
// with an early return so the harness drops them silently. Two targeted fuzzers additionally
// cross-check the time update against an independent propagation and a noise-free twin, and check
// that reInitialize() restores the configured seed exactly.
//
// All quantities are in the filter's internal units (km, km/s).

#include "flybyFilterTestHelpers.hpp"

#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <filteringCore/dynamicsModel.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>

namespace {

using filtering::flybyFilter::Vector6;

// A measurement component that is either bounded-finite (the normal case) or one of the non-finite
// specials. It deliberately excludes absurd-magnitude finite values (e.g. 1e308): the filter only
// guarantees that *non-finite* headings are skipped, not that arbitrarily large finite ones avoid
// overflow.
inline auto boundedOrNonFinite() {
    return fuzztest::OneOf(fuzztest::InRange(-2.0, 2.0),
                           fuzztest::ElementOf<double>({std::numeric_limits<double>::quiet_NaN(),
                                                        std::numeric_limits<double>::infinity(),
                                                        -std::numeric_limits<double>::infinity()}));
}

// Thin free-function wrappers so FUZZ_TEST references an unqualified name (the property helpers
// themselves live in namespace filtering::flybyFilter).
inline void fuzzUpdateKeepsStateValidAndBounded(Eigen::Vector3d rOffset,
                                                Eigen::Vector3d vOffset,
                                                double mu,
                                                Vector6 covDiag,
                                                double q,
                                                Eigen::Vector3d rhat,
                                                double dt) {
    filtering::flybyFilter::propertyUpdateKeepsStateValidAndBounded(rOffset, vOffset, mu, covDiag, q, rhat, dt);
}

inline void fuzzArbitraryMeasurementsPreserveState(Eigen::Vector3d rOffset,
                                                   Eigen::Vector3d vOffset,
                                                   double mu,
                                                   Vector6 covDiag,
                                                   double q,
                                                   Eigen::Vector3d rhat,
                                                   double dt) {
    filtering::flybyFilter::propertyArbitraryMeasurementsPreserveState(rOffset, vOffset, mu, covDiag, q, rhat, dt);
}

inline void fuzzMeasurementDoesNotIncreaseCovariance(Eigen::Vector3d rOffset,
                                                     Eigen::Vector3d vOffset,
                                                     double mu,
                                                     Vector6 covDiag,
                                                     Eigen::Vector3d rhat) {
    filtering::flybyFilter::propertyMeasurementDoesNotIncreaseCovariance(rOffset, vOffset, mu, covDiag, rhat);
}

}  // namespace

// The full update() path -- enqueue, drain through applySequentialRobust, snapshot -- over bounded
// finite inputs leaves the estimate finite, the covariance symmetric-PSD, and the returned snapshot
// consistent with the accessors.
FUZZ_TEST(FlybyFilterPropertyFuzz, fuzzUpdateKeepsStateValidAndBounded)
    .WithDomains(xmera::fuzz::Vector3dInRange(-500.0, 500.0),                          // initial r offset [km]
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),                              // initial v offset [km/s]
                 fuzztest::InRange(1E3, 1E5),                                          // mu [km^3/s^2]
                 xmera::fuzz::EigenVectorOf<double, 6>(fuzztest::InRange(-1E4, 1E4)),  // covariance diagonal
                 fuzztest::InRange(0.0, 1E-4),                                         // process noise
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),                              // heading observation
                 fuzztest::InRange(0.0, 60.0));                                        // dt [s]

// Same finite config inputs, but the heading is fully arbitrary (including NaN / Inf). A non-finite
// measurement must be skipped, leaving the estimate finite and the covariance PSD.
FUZZ_TEST(FlybyFilterPropertyFuzz, fuzzArbitraryMeasurementsPreserveState)
    .WithDomains(xmera::fuzz::Vector3dInRange(-500.0, 500.0),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(1E3, 1E5),
                 xmera::fuzz::EigenVectorOf<double, 6>(fuzztest::InRange(-1E4, 1E4)),
                 fuzztest::InRange(0.0, 1E-4),
                 xmera::fuzz::EigenVectorOf<double, 3>(boundedOrNonFinite()),
                 fuzztest::InRange(0.0, 60.0));

// A heading measurement folded in with no time propagation never increases the covariance trace.
FUZZ_TEST(FlybyFilterPropertyFuzz, fuzzMeasurementDoesNotIncreaseCovariance)
    .WithDomains(xmera::fuzz::Vector3dInRange(-500.0, 500.0),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(1E3, 1E5),
                 xmera::fuzz::EigenVectorOf<double, 6>(fuzztest::InRange(-1E4, 1E4)),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0));

namespace filtering::flybyFilter {

// ============================================================================
// Targeted: a time update advances the mean along the two-body flow and adds process noise to the
// covariance (no smaller than a noise-free propagation; stays symmetric + PSD + finite).
// ============================================================================
void fuzzTimeUpdatePropagatesStateAndGrowsCovariance(Eigen::Vector3d rOffset,
                                                     Eigen::Vector3d vOffset,
                                                     double mu,
                                                     Vector6 covDiag,
                                                     double q,
                                                     double dt) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiag, q, kHeadingStd);
    if (!cfg) {
        return;
    }
    TestState const initial = cfg->getInitialState();
    FlybyFilterAlgorithm algo(*cfg);

    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";

    // The central sigma point (== reported state) follows the same two-body propagation.
    TestState const predicted = filtering::propagate(FlybyDynamics{mu}, initial, {0.0, dt});
    EXPECT_TRUE(algo.getState().raw().isApprox(predicted.raw(), 1E-9)) << "state must follow the two-body flow";
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance())) << "covariance after timeUpdate";

    // Compare against the same propagation with no process noise: P(withQ) >= P(noQ).
    std::optional<FlybyFilterConfig> const noiseFreeCfg =
        tryFuzzConfig(rOffset, vOffset, mu, covDiag, 0.0, kHeadingStd);
    ASSERT_TRUE(noiseFreeCfg.has_value());
    FlybyFilterAlgorithm noiseFree(*noiseFreeCfg);
    ASSERT_TRUE(noiseFree.timeUpdate(dt)) << "noise-free timeUpdate should be valid";
    EXPECT_GE(algo.getCovariance().trace(), noiseFree.getCovariance().trace() - 1E-6)
        << "process noise should not shrink the covariance";
}
FUZZ_TEST(FlybyFilterFuzz, fuzzTimeUpdatePropagatesStateAndGrowsCovariance)
    .WithDomains(xmera::fuzz::Vector3dInRange(-500.0, 500.0),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(1E3, 1E5),
                 xmera::fuzz::EigenVectorOf<double, 6>(fuzztest::InRange(-1E4, 1E4)),
                 fuzztest::InRange(0.0, 1E-4),
                 fuzztest::InRange(0.0, 60.0));

// ============================================================================
// Targeted: reInitialize() restores exactly the configured seed, for any valid configuration and
// any amount of intervening propagation and measurement activity.
//
// Note the invariant deliberately *not* asserted here: "a measurement update moves the estimate
// toward the observation". Unlike the identity measurement model of a rate or attitude filter, the
// heading model r/|r| is strongly nonlinear, so the linear UKF correction can overshoot for an
// ill-conditioned prior -- fuzzing found such a case immediately. The information-gain property
// that does hold is covered by fuzzMeasurementDoesNotIncreaseCovariance above.
// ============================================================================
void fuzzReInitializeRestoresTheConfiguredSeed(Eigen::Vector3d rOffset,
                                               Eigen::Vector3d vOffset,
                                               double mu,
                                               Vector6 covDiag,
                                               double q,
                                               Eigen::Vector3d rhatRaw,
                                               double dt) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiag, q, kHeadingStd);
    if (!cfg || !rhatRaw.allFinite() || rhatRaw.norm() < 1E-3 || !std::isfinite(dt) || dt < 0.0) {
        return;
    }
    TestState const seed = cfg->getInitialState();
    Matrix6 const seedCovariance = cfg->getInitialCovariance();
    FlybyFilterAlgorithm algo(*cfg);

    // Drive the filter away from its seed with a propagation and a measurement.
    HeadingData heading;
    heading.timeTag = dt;
    heading.rhat_BN_N = rhatRaw.normalized();
    algo.update(dt, heading);

    algo.reInitialize();
    EXPECT_TRUE(algo.getState().raw().isApprox(seed.raw(), 1E-9)) << "reInitialize must restore the configured state";
    EXPECT_TRUE(algo.getCovariance().isApprox(seedCovariance, 1E-9))
        << "reInitialize must restore the configured covariance";

    // A second cycle after re-seeding still behaves: nothing is left in an inconsistent state.
    algo.update(dt + 1.0, HeadingData{});
    EXPECT_TRUE(algo.getState().raw().allFinite());
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance()));
}
FUZZ_TEST(FlybyFilterFuzz, fuzzReInitializeRestoresTheConfiguredSeed)
    .WithDomains(xmera::fuzz::Vector3dInRange(-500.0, 500.0),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(1E3, 1E5),
                 xmera::fuzz::EigenVectorOf<double, 6>(fuzztest::InRange(-1E4, 1E4)),
                 fuzztest::InRange(0.0, 1E-4),
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),
                 fuzztest::InRange(0.0, 60.0));

}  // namespace filtering::flybyFilter
