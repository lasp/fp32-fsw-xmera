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

// With a secondary body the constraint-validity branch decision is computed in float by the
// algorithm and in double by the reference, so near-threshold inputs may legitimately diverge.
// Check branch-independent properties instead: the output is always finite and the MRP norm is
// bounded by 1.
void fuzzCelestialTwoBodyPointWithSecondaryProperties(const Eigen::Vector3d& r_BN_N,
                                                      const Eigen::Vector3d& v_BN_N,
                                                      const Eigen::Vector3d& r_PB_N,
                                                      const Eigen::Vector3d& v_PB_N,
                                                      const Eigen::Vector3d& r_SB_N,
                                                      const Eigen::Vector3d& v_SB_N) {
    const double rNorm = r_PB_N.norm();
    const double vNorm = v_PB_N.norm();
    if (vNorm == 0.0 || r_PB_N.cross(v_PB_N).norm() < 1.0e-3 * rNorm * vNorm) {
        return;
    }
    // The secondary body must not sit on top of the spacecraft, otherwise its direction is
    // undefined (normalizing a zero vector).
    if (r_SB_N.norm() < 1.0e3) {
        return;
    }

    propertyOutputIsFinite(r_BN_N + r_PB_N,
                           v_BN_N + v_PB_N,
                           r_BN_N + r_SB_N,
                           v_BN_N + v_SB_N,
                           r_BN_N,
                           v_BN_N,
                           kCelestialBodyAlignmentThreshold);
    propertySigmaNormBounded(r_BN_N + r_PB_N,
                             v_BN_N + v_PB_N,
                             r_BN_N + r_SB_N,
                             v_BN_N + v_SB_N,
                             r_BN_N,
                             v_BN_N,
                             kCelestialBodyAlignmentThreshold);
}

}  // namespace

FUZZ_TEST(CelestialTwoBodyPointFuzz, fuzzCelestialTwoBodyPointWithSecondaryProperties)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2e13, 2e13),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4),
                 fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e4; },
                                  xmera::fuzz::Vector3dInRange(-2e13, 2e13)),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4),
                 xmera::fuzz::Vector3dInRange(-2e13, 2e13),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4));
