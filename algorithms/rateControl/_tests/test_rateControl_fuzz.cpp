#include "rateControlTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

static auto Vec3(float lo, float hi) { return fuzztest::ArrayOf<3>(fuzztest::InRange(lo, hi)); }

FUZZ_TEST(rateControlFuzz, fuzzAdapter_rateControl)
    .WithDomains(fuzztest::ArrayOf<6>(fuzztest::InRange(-100.0f, 100.0f)),  // inertia upper-tri params
                 fuzztest::InRange(0.0f, 100.0f),                           // P must be >= 0
                 Vec3(-100.0f, 100.0f),                                     // known torque
                 Vec3(-100.0f, 100.0f),                                     // omega_BR
                 Vec3(-100.0f, 100.0f),                                     // omega_RN
                 Vec3(-100.0f, 100.0f)                                      // domega_RN
    );
