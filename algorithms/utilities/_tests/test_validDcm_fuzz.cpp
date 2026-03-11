#include "utilitiesHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(DcmFuzz, testDcmValidity)
    .WithDomains(fuzztest::InRange(1e-6F, 1.0F), fuzztest::InRange(1e-6F, 1.0F), fuzztest::InRange(1e-6F, 1.0F));
