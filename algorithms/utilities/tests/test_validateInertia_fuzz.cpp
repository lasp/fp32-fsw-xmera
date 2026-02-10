#include "utilitiesHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(InertiaFuzz, testInertiaValidity)
    .WithDomains(fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F));
