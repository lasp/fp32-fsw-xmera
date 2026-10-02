#include "flybyPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and nav inputs must agree with the
// independent reference implementation across multiple steps. Both the
// extrapolation branch and the validity-gated re-seeding branch are exercised
// depending on how numSteps compares to filterReadPeriods.
// ---------------------------------------------------------------------------
static void fuzzRegressionFlybyPoint(double controlPeriod,
                                     uint32_t filterReadPeriods,
                                     float toleranceForCollinearity,
                                     int signOfOrbitNormalFrameVector,
                                     float maximumRateThreshold,
                                     float maximumAccelerationThreshold,
                                     float positionKnowledgeSigma,
                                     const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3d& v_BN_N,
                                     int numSteps) {
    // Skip (near-)collinear r/v: r x v vanishes and the orbit frame is undefined, so both the algorithm and the
    // reference refuse the seed, and close to the collinearity tolerance the refusal can depend on platform
    // rounding (FMA).
    if (r_BN_N.normalized().cross(v_BN_N.normalized()).norm() < 1e-6) {
        return;
    }
    const FlybyPointConfig cfg = FlybyPointConfig::create(controlPeriod,
                                                          filterReadPeriods,
                                                          toleranceForCollinearity,
                                                          signOfOrbitNormalFrameVector,
                                                          maximumRateThreshold,
                                                          maximumAccelerationThreshold,
                                                          positionKnowledgeSigma);
    regressionTestFlybyPoint(cfg, r_BN_N, v_BN_N, numSteps);
}

FUZZ_TEST(FlybyPointAlgorithmFuzz, fuzzRegressionFlybyPoint)
    .WithDomains(fuzztest::InRange(1e-3, 1.0),               // controlPeriod [s]
                 fuzztest::InRange<uint32_t>(1U, 200U),      // filterReadPeriods [-]
                 fuzztest::InRange(1e-6F, 1e6F),             // toleranceForCollinearity [-]
                 fuzztest::ElementOf<int>({-1, 1}),          // signOfOrbitNormalFrameVector
                 fuzztest::InRange(1e-6F, 1e6F),             // maximumRateThreshold [deg/s]
                 fuzztest::InRange(1e-6F, 1e6F),             // maximumAccelerationThreshold [deg/s^2]
                 fuzztest::InRange(1e-6F, 1e6F),             // positionKnowledgeSigma [m]
                 xmera::fuzz::Vector3dInRange(-1e14, 1e14),  // r_BN_N [m]
                 xmera::fuzz::Vector3dInRange(-1e14, 1e14),  // v_BN_N [m/s]
                 fuzztest::InRange(1, 120));                 // numSteps [-]
