#include "celestialTwoBodyPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

namespace {

constexpr float kSingularityThreshold = 0.017453292519943F;  // [rad] 1 deg
constexpr float kRateThreshold = 0.17453292519943F;          // [rad/s] 10 deg/s

// Fuzz the spacecraft state and the *relative* primary-body state, then reconstruct the absolute
// primary-body state. Without a secondary body the constraint axis is always r_PB x v_PB, so the
// branch decision is deterministic and the FP32 output can be compared against the double
// reference. A physically meaningful lower bound on the relative distance keeps the float32
// outputs from losing precision against the double reference in the r -> 0 regime.
void fuzzCelestialTwoBodyPointNoSecondary(const Eigen::Vector3d& r_BN_N,
                                          const Eigen::Vector3d& v_BN_N,
                                          const Eigen::Vector3d& relPos,
                                          const Eigen::Vector3d& relVel) {
    // Joint domain filter: skip configurations where the fallback constraint axis is degenerate
    // (relVel zero, or relVel parallel to relPos). The reference frame is undefined in those cases
    // because r3 comes from normalizing R_n = r_PB x (r_PB x v_PB); with |R_n| ~ 0 the DCM is
    // rank-deficient. Require sin(angle(relPos, relVel)) >= 1e-3 to stay clear of that regime.
    const double rNorm = relPos.norm();
    const double vNorm = relVel.norm();
    if (vNorm == 0.0 || relPos.cross(relVel).norm() < 1.0e-3 * rNorm * vNorm) {
        return;
    }

    testCelestialTwoBodyPoint(r_BN_N + relPos,
                              v_BN_N + relVel,
                              Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(),
                              r_BN_N,
                              v_BN_N,
                              kSingularityThreshold,
                              kRateThreshold,
                              false);
}

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
                           kSingularityThreshold,
                           kRateThreshold,
                           true);
    propertySigmaNormBounded(r_BN_N + relPosPrimary,
                             v_BN_N + relVelPrimary,
                             r_BN_N + relPosSecondary,
                             v_BN_N + relVelSecondary,
                             r_BN_N,
                             v_BN_N,
                             kSingularityThreshold,
                             kRateThreshold,
                             true);
}

}  // namespace

FUZZ_TEST(CelestialTwoBodyPointFuzz, fuzzCelestialTwoBodyPointNoSecondary)
    .WithDomains(
        // Inertial spacecraft position [m], bounded by heliosphere (~120 AU, ~1.8e13 m)
        xmera::fuzz::Vector3dInRange(-2e13, 2e13),
        // Inertial spacecraft velocity [m/s], well above any orbital regime in the solar system
        xmera::fuzz::Vector3dInRange(-1e4, 1e4),
        // Relative primary-body position [m], with |relPos| >= 10 km to stay clear of the
        // small-radius regime where the reference rates blow up.
        fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e4; },
                         xmera::fuzz::Vector3dInRange(-2e13, 2e13)),
        // Relative primary-body velocity [m/s]
        xmera::fuzz::Vector3dInRange(-1e4, 1e4));

FUZZ_TEST(CelestialTwoBodyPointFuzz, fuzzCelestialTwoBodyPointWithSecondaryProperties)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2e13, 2e13),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4),
                 fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e4; },
                                  xmera::fuzz::Vector3dInRange(-2e13, 2e13)),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4),
                 xmera::fuzz::Vector3dInRange(-2e13, 2e13),
                 xmera::fuzz::Vector3dInRange(-1e4, 1e4));
