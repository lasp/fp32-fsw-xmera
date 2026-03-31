#include "thrFiringSchmittTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Fuzzed regression test
// ---------------------------------------------------------------------------

FUZZ_TEST(ThrFiringSchmittFuzz, testThrFiringSchmittRegression)
    .WithDomains(fuzztest::InRange(1e-6F, 1.0F),   // levelOn
                 fuzztest::InRange(0.0F, 0.999F),  // levelOff
                 fuzztest::InRange(1e-3F, 2.0F),   // thrMinFireTime
                 fuzztest::OneOf(fuzztest::Just(ThrustPulsingRegime::ON_PULSING),
                                 fuzztest::Just(ThrustPulsingRegime::OFF_PULSING)),               // thrustPulsingRegime
                 fuzztest::InRange(0U, static_cast<uint32_t>(kMaxThrusterCount)),                 // numThrusters
                 fuzztest::VectorOf(fuzztest::InRange(1e-3F, 1e4F)).WithSize(kMaxThrusterCount),  // maxThrustVec
                 fuzztest::VectorOf(fuzztest::InRange(-1e4F, 1e4F)).WithSize(kMaxThrusterCount),  // thrForceVec
                 fuzztest::InRange(1e-6F, 5.0F)                                                   // dt
    );
