#include "celestialTwoBodyPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

namespace {

constexpr float kCelestialBodyAlignmentThreshold = 0.017453292519943F;  // [rad] 1 deg

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and reference inputs must agree with the
// independent reference implementation across multiple steps.
// ---------------------------------------------------------------------------

FUZZ_TEST(CelestialTwoBodyPointAlgorithmFuzz, testCelestialTwoBodyPointRegression)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // r_PN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // v_PN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // r_SN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // v_SN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // r_BN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // v_BN_N
                 fuzztest::InRange(1e-6F, 1e6F));            // celestialBodyAlignmentThreshold

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(CelestialTwoBodyPointPropertyFuzz, propertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // r_PN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // v_PN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // r_SN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // v_SN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // r_BN_N
                 xmera::fuzz::Vector3dInRange(-1e6F, 1e6F),  // v_BN_N
                 fuzztest::InRange(1e-6F, 1e6F));            // celestialBodyAlignmentThreshold

}  // namespace

FUZZ_TEST(CelestialTwoBodyPointFuzz, fuzzCelestialTwoBodyPointWithSecondaryProperties)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2e13, 2e13),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4),
                 fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e4; },
                                  xmera::fuzz::Vector3dInRange(-2e13, 2e13)),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4),
                 xmera::fuzz::Vector3dInRange(-2e13, 2e13),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4));
