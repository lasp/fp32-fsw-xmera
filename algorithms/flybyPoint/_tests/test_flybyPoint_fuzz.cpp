#include "flybyPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and nav inputs must agree with the
// independent reference implementation across multiple steps. Both the
// extrapolation branch and the validity-gated re-seeding branch are exercised
// depending on how stepNanos compares to timeBetweenFilterData.
// ---------------------------------------------------------------------------
static void fuzzRegressionFlybyPoint(double timeBetweenFilterData,
                                     float toleranceForCollinearity,
                                     int signOfOrbitNormalFrameVector,
                                     float maximumRateThreshold,
                                     float maximumAccelerationThreshold,
                                     float positionKnowledgeSigma,
                                     const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3d& v_BN_N,
                                     uint64_t stepNanos,
                                     int numSteps) {
    const FlybyPointConfig cfg = FlybyPointConfig::create(timeBetweenFilterData,
                                                          toleranceForCollinearity,
                                                          signOfOrbitNormalFrameVector,
                                                          maximumRateThreshold,
                                                          maximumAccelerationThreshold,
                                                          positionKnowledgeSigma);
    regressionTestFlybyPoint(cfg, r_BN_N, v_BN_N, stepNanos, numSteps);
}

FUZZ_TEST(FlybyPointAlgorithmFuzz, fuzzRegressionFlybyPoint)
    .WithDomains(fuzztest::InRange(1e-6, 1e6),                                 // timeBetweenFilterData [s]
                 fuzztest::InRange(1e-6F, 1.0F),                               // toleranceForCollinearity [-]
                 fuzztest::ElementOf<int>({-1, 1}),                            // signOfOrbitNormalFrameVector
                 fuzztest::InRange(1e-6F, 1e6F),                               // maximumRateThreshold [deg/s]
                 fuzztest::InRange(1e-6F, 1e6F),                               // maximumAccelerationThreshold [deg/s^2]
                 fuzztest::InRange(1.0F, 1e9F),                                // positionKnowledgeSigma [m]
                 xmera::fuzz::Vector3dInRange(-1e14, 1e14),                    // r_BN_N [m]
                 xmera::fuzz::Vector3dInRange(-1e14, 1e14),                    // v_BN_N [m/s]
                 fuzztest::InRange<uint64_t>(1'000'000ULL, 1'000'000'000ULL),  // stepNanos [ns],
                 fuzztest::InRange(1, 120));                                   // numSteps [-]
