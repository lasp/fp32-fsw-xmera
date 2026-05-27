#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "hillPointTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

namespace {

// Fuzz the planet state and the *relative* spacecraft state, then reconstruct the absolute
// spacecraft state. This lets us place a physically meaningful lower bound on the relative
// orbital radius: dfdt = |h| / r^2 amplifies large relative velocities into unphysically huge
// rates as r -> 0, and the float32 outputs lose precision against the double reference well
// before any real spacecraft would be that close to its primary body.
void fuzzHillPoint(const Eigen::Vector3d& r_planet_N,
                   const Eigen::Vector3d& v_planet_N,
                   const Eigen::Vector3d& relPos,
                   const Eigen::Vector3d& relVel) {
    // Joint domain filter: skip configurations where the orbital angular momentum is degenerate
    // (relVel zero, or relVel parallel to relPos). The Hill frame is undefined in those cases
    // because the i_h axis comes from normalizing h = relPos x relVel; with |h| ~ 0 the DCM is
    // rank-deficient and dcmToMrp returns a value unrelated to the (also rank-deficient)
    // reference. Require sin(angle(relPos, relVel)) >= 1e-3 to stay clear of that regime.
    const double rNorm = relPos.norm();
    const double vNorm = relVel.norm();
    if (vNorm == 0.0 || relPos.cross(relVel).norm() < 1.0e-3 * rNorm * vNorm) {
        return;
    }

    testHillPoint(r_planet_N + relPos, v_planet_N + relVel, r_planet_N, v_planet_N);
}

}  // namespace

FUZZ_TEST(HillPointFuzz, fuzzHillPoint)
    .WithDomains(
        // Inertial primary-body position [m], bounded by heliosphere (~120 AU, ~1.8e13 m)
        xmera::fuzz::Vector3dInRange(-2e13, 2e13),
        // Inertial primary-body velocity [m/s], well above any orbital regime in the solar system
        xmera::fuzz::Vector3dInRange(-1e5, 1e5),
        // Relative spacecraft position [m], same scale as r_planet_N, with |relPos| >= 10 km to
        // stay clear of the small-radius regime where dfdt blows up.
        fuzztest::Filter([](const Eigen::Vector3d& v) { return v.norm() >= 1.0e4; },
                         xmera::fuzz::Vector3dInRange(-2e13, 2e13)),
        // Relative spacecraft velocity [m/s]
        xmera::fuzz::Vector3dInRange(-1e5, 1e5));
