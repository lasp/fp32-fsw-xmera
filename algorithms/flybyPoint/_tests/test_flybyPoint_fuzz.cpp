#include "../architecture/testUtilities/eigenFuzzDomains.hpp"
#include "flybyPointTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and reference inputs must agree with the
// independent reference implementation across multiple steps.
// ---------------------------------------------------------------------------
FUZZ_TEST(FlybyPointAlgorithmFuzz, regressionTestFlybyPoint)
    .WithDomains(fuzztest::InRange(1.0e-6, 1.0e3),       // timeBetweenFilterData
                 fuzztest::InRange(1.0e-6F, 1.0e3F),     // toleranceForCollinearity
                 fuzztest::ElementOf<int>({-1, 1}),      // signOfOrbitNormalFrameVector
                 fuzztest::InRange(1.0e-6F, 1.0e6F),     // maxRateThreshold
                 fuzztest::InRange(1.0e-6F, 1.0e6F),     // maxAccelerationThreshold
                 fuzztest::InRange(1.0e-6F, 1.0e6F),     // positionKnowledgeSigma
                 fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e-3; },
                                  xmera::fuzz::Vector3dInRange(-1.0e6, 1.0e6)),  // r
                 fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e-3; },
                                  xmera::fuzz::Vector3dInRange(-1.0e6, 1.0e6))); // v