#include "ephemeridesRecenterTestHelpers.hpp"
#include <fuzztest/fuzztest.h>
#include <array>
#include <vector>

static std::vector<BodyName> bodyListInOrder{
    makeBodyName("SUN"),
    makeBodyName("EARTH"),
    makeBodyName("MOON"),
    makeBodyName("SATURN"),
};

FUZZ_TEST(EphemeridesRecenterAlgorithmFuzz, regressionTestEphemeridesRecenter)
    .WithDomains(fuzztest::Just(bodyListInOrder),
                 fuzztest::InRange(0, 3),  // previousCommonIdx
                 fuzztest::InRange(0, 3),  // newZeroIdx

                 // r0, v0, isMoon0
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e12, 1e12)),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e9, 1e9)),
                 fuzztest::Just(false),

                 // r1, v1, isMoon1
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e12, 1e12)),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e9, 1e9)),
                 fuzztest::Just(false),

                 // r2, v2, isMoon2
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e12, 1e12)),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e9, 1e9)),
                 fuzztest::Arbitrary<bool>(),

                 // r3, v3, isMoon3
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e12, 1e12)),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e9, 1e9)),
                 fuzztest::Just(false));
