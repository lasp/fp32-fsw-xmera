#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "dvGuidanceTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

namespace {

// Fuzz the burn command inputs and the (callTime - burnStartTime) elapsed-time delta. We bound:
//   |dvInrtlCmd|    >= 1e-3  -- avoid normalize() producing NaN (Xmera semantic, not robustified).
//   |dvRotVecUnit|  >= 1e-3  -- same reason: cross(dvRotVecUnit, dvHat).normalized() needs a non-zero
//                               cross product, which also requires the seed not be parallel to dvHat.
//   The angle between dvInrtlCmd and dvRotVecUnit cannot drop below ~1e-4 rad without the cross
//   product collapsing in float; filter that case out.
//   dvRotVecMag    -- bounded to a reasonable burn rate envelope; magnitude only.
//   burnTime       -- bounded so the accumulated PRV doesn't grow unbounded; the dcm/MRP conversions
//                     are well-defined for any rotation, but very large rotations stress shadow-set
//                     switching and produce noise that exceeds the 1e-5 reference tolerance.
void fuzzDvGuidance(const Eigen::Vector3f& dvInrtlCmd,
                    const Eigen::Vector3f& dvRotVecUnit,
                    float dvRotVecMag,
                    int64_t burnTime_ns) {
    // Joint precondition: dvRotVecUnit must not be parallel to dvInrtlCmd, otherwise
    // cross(dvRotVecUnit, dvHat).normalized() yields NaN. Require sin(angle) >= 1e-3.
    if (dvRotVecUnit.cross(dvInrtlCmd).norm() < 1.0e-3F * dvRotVecUnit.norm() * dvInrtlCmd.norm()) {
        return;
    }

    constexpr uint64_t burnStartTime = 1'000'000'000ULL;  // arbitrary 1 s anchor
    const uint64_t callTime = static_cast<uint64_t>(static_cast<int64_t>(burnStartTime) + burnTime_ns);
    testDvGuidance(dvInrtlCmd, dvRotVecUnit, dvRotVecMag, burnStartTime, callTime);
}

}  // namespace

FUZZ_TEST(DvGuidanceFuzz, fuzzDvGuidance)
    .WithDomains(
        // dvInrtlCmd [m/s]: realistic delta-V envelope, rejected if too close to zero.
        fuzztest::Filter([](const Eigen::Vector3f& v) { return v.norm() >= 1.0e-3F; },
                         xmera::fuzz::Vector3fInRange(-1.0e4F, 1.0e4F)),
        // dvRotVecUnit: physically a unit vector but the algorithm only uses its direction. Allow
        // any finite non-degenerate direction; the cross product with dvHat must remain large
        // enough that .normalized() doesn't blow up.
        fuzztest::Filter([](const Eigen::Vector3f& v) { return v.norm() >= 1.0e-3F; },
                         xmera::fuzz::Vector3fInRange(-1.0F, 1.0F)),
        // dvRotVecMag [rad/s]: bounded to a generous burn-rate envelope.
        fuzztest::InRange(-1.0F, 1.0F),
        // burnTime [ns]: +/- 30 s so the accumulated rotation stays within ~30 rad, well within
        // the regime where the FP32 reference comparison holds at 1e-5 tolerance.
        fuzztest::InRange<int64_t>(-30'000'000'000LL, 30'000'000'000LL));
