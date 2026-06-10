#include "celestialTwoBodyPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

namespace {

constexpr float kSingularityThreshold = 0.017453292519943F;  // [rad] 1 deg

// With a secondary body the constraint-validity branch decision is computed in float by the
// algorithm and in double by the reference, so near-threshold inputs may legitimately diverge.
// Check branch-independent properties instead: the output is always finite and the MRP norm is
// bounded by 1.
void fuzzCelestialTwoBodyPointWithSecondaryProperties(const Eigen::Vector3d& r_BN_N,
                                                      const Eigen::Vector3d& v_BN_N,
                                                      const Eigen::Vector3d& relPosPrimary,
                                                      const Eigen::Vector3d& relVelPrimary,
                                                      const Eigen::Vector3d& relPosSecondary,
                                                      const Eigen::Vector3d& relVelSecondary) {
    const double rNorm = relPosPrimary.norm();
    const double vNorm = relVelPrimary.norm();
    if (vNorm == 0.0 || relPosPrimary.cross(relVelPrimary).norm() < 1.0e-3 * rNorm * vNorm) {
        return;
    }
    // The secondary body must not sit on top of the spacecraft, otherwise its direction is
    // undefined (normalizing a zero vector).
    if (relPosSecondary.norm() < 1.0e3) {
        return;
    }

    propertyOutputIsFinite(r_BN_N + relPosPrimary,
                           v_BN_N + relVelPrimary,
                           r_BN_N + relPosSecondary,
                           v_BN_N + relVelSecondary,
                           r_BN_N,
                           v_BN_N,
                           kSingularityThreshold);
    propertySigmaNormBounded(r_BN_N + relPosPrimary,
                             v_BN_N + relVelPrimary,
                             r_BN_N + relPosSecondary,
                             v_BN_N + relVelSecondary,
                             r_BN_N,
                             v_BN_N,
                             kSingularityThreshold);
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
