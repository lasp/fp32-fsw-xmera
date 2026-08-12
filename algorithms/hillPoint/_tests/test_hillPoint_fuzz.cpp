#include "hillPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>
#include <Eigen/Geometry>

namespace {

// ---------------------------------------------------------------------------
// Regression fuzz test
// ---------------------------------------------------------------------------

// Fuzz the planet state and the *relative* spacecraft state, then reconstruct the absolute
// spacecraft state. This lets us place a physically meaningful lower bound on the relative
// orbital radius: dfdt = |h| / r^2 amplifies large relative velocities into unphysically huge
// rates as r -> 0, and the float32 outputs lose precision against the double reference well
// before any real spacecraft would be that close to its primary body.
void fuzzHillPoint(const Eigen::Vector3d& r_PN_N,
                   const Eigen::Vector3d& v_PN_N,
                   const Eigen::Vector3d& r_BP_N,
                   const Eigen::Vector3d& v_BP_N) {
    testHillPointRegression(r_PN_N + r_BP_N, v_PN_N + v_BP_N, r_PN_N, v_PN_N);
}

}  // namespace

FUZZ_TEST(HillPointFuzz, fuzzHillPoint)
    .WithDomains(
        // Inertial primary-body position [m], bounded by heliosphere (~120 AU, ~1.8e13 m)
        xmera::fuzz::Vector3dInRange(-2e13, 2e13),
        // Inertial primary-body velocity [m/s], well above any orbital regime in the solar system
        xmera::fuzz::Vector3dInRange(-1e5, 1e5),
        // Relative spacecraft position [m], same scale as r_PN_N, with |r_BP_N| >= 10 km to
        // stay clear of the small-radius regime where dfdt blows up.
        fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e4; },
                         xmera::fuzz::Vector3dInRange(-2e13, 2e13)),
        // Relative spacecraft velocity [m/s]
        xmera::fuzz::Vector3dInRange(-1e5, 1e5));

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

void fuzzPropertyOutputIsFinite(const Eigen::Vector3d& r_PN_N,
                                const Eigen::Vector3d& v_PN_N,
                                const Eigen::Vector3d& r_BP_N,
                                const Eigen::Vector3d& v_BP_N) {
    propertyOutputIsFinite(r_PN_N + r_BP_N, v_PN_N + v_BP_N, r_PN_N, v_PN_N);
}

FUZZ_TEST(HillPointPropertyFuzz, fuzzPropertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2e13, 2e13),  // r_PN_N
                 xmera::fuzz::Vector3dInRange(-1e5, 1e5),    // v_PN_N
                 xmera::fuzz::Vector3dInRange(-2e13, 2e13),  // r_BP_N
                 xmera::fuzz::Vector3dInRange(-1e5, 1e5));   // v_BP_N

void fuzzPropertySigmaNormBounded(const Eigen::Vector3d& r_PN_N,
                                  const Eigen::Vector3d& v_PN_N,
                                  const Eigen::Vector3d& r_BP_N,
                                  const Eigen::Vector3d& v_BP_N) {
    propertySigmaNormBounded(r_PN_N + r_BP_N, v_PN_N + v_BP_N, r_PN_N, v_PN_N);
}

FUZZ_TEST(HillPointPropertyFuzz, fuzzPropertySigmaNormBounded)
    .WithDomains(xmera::fuzz::Vector3dInRange(-2e13, 2e13),  // r_PN_N
                 xmera::fuzz::Vector3dInRange(-1e5, 1e5),    // v_PN_N
                 xmera::fuzz::Vector3dInRange(-2e13, 2e13),  // r_BP_N
                 xmera::fuzz::Vector3dInRange(-1e5, 1e5));   // v_BP_N
