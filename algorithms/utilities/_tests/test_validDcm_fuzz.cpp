#include "utilitiesHelpers.hpp"
#include <fuzztest/fuzztest.h>

inline void testFloatDcmValidity(double const sigma1, double const sigma2, double const sigma3) {
    const Eigen::Matrix3f dcm = generateValidDCM(sigma1, sigma2, sigma3).cast<float>();

    EXPECT_TRUE(isValidDcm(dcm));
}

FUZZ_TEST(DcmFuzz, testFloatDcmValidity)
    .WithDomains(fuzztest::InRange(1e-6, 1.0), fuzztest::InRange(1e-6, 1.0), fuzztest::InRange(1e-6, 1.0));
