#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "dvGuidanceTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

namespace {

/// Fuzzes the burn-command inputs and the (callTime - burnStartTime) elapsed-time delta across the
/// full domain, including the degenerate cases the algorithm now guards (zero-norm dvInrtlCmd and
/// dvRotVecUnit (anti)parallel to it). testDvGuidance asserts the output is always finite and
/// reference-matches at 1e-5 only where the inputs are non-degenerate by margin. See dvGuidance.rst
/// ("Numerical conditioning") for why the guard thresholds bound the FP32 reference error.
void fuzzDvGuidance(const Eigen::Vector3f& dvInrtlCmd,
                    const Eigen::Vector3f& dvRotVecUnit,
                    float dvRotVecMag,
                    int64_t burnTime_ns) {
    constexpr uint64_t burnStartTime = 1'000'000'000ULL;  // arbitrary 1 s anchor
    const uint64_t callTime = static_cast<uint64_t>(static_cast<int64_t>(burnStartTime) + burnTime_ns);
    testDvGuidance(dvInrtlCmd, dvRotVecUnit, dvRotVecMag, burnStartTime, callTime);
}

}  // namespace

FUZZ_TEST(DvGuidanceFuzz, fuzzDvGuidance)
    .WithDomains(
        // dvInrtlCmd [m/s]: realistic delta-V envelope, unfiltered so near-/zero-norm inputs exercise
        // the zero-norm guard (the algorithm returns a safe default instead of NaN).
        xmera::fuzz::Vector3fInRange(-1.0e4F, 1.0e4F),
        // dvRotVecUnit: only its direction is used. Unfiltered so (anti)parallel-to-dvInrtlCmd seeds
        // exercise the collapsed-cross-product guard.
        xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),
        // dvRotVecMag [rad/s]: bounded to a generous burn-rate envelope.
        fuzztest::InRange(-1.0F, 1.0F),
        // burnTime [ns]: +/- 30 s so the accumulated rotation stays within ~30 rad, well within
        // the regime where the FP32 reference comparison holds at 1e-5 tolerance.
        fuzztest::InRange<int64_t>(-30'000'000'000LL, 30'000'000'000LL));
