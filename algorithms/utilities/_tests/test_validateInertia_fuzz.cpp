#include "utilitiesHelpers.hpp"
#include <fuzztest/fuzztest.h>

/* Test validity against generated inertia matrix */
inline void testInertiaValidity(const double eigen1,
                                const double eigen2,
                                const double sigma1,
                                const double sigma2,
                                const double sigma3) {
    const Eigen::Matrix3f inertia = generateValidInertiaMatrix(eigen1, eigen2, sigma1, sigma2, sigma3).cast<float>();

    EXPECT_TRUE(inertiaIsValid(inertia));
}

FUZZ_TEST(InertiaFuzz, testInertiaValidity)
    .WithDomains(fuzztest::InRange(1.0, 1e6),
                 fuzztest::InRange(1.0, 1e6),
                 fuzztest::InRange(1e-6, 1.0),
                 fuzztest::InRange(1e-6, 1.0),
                 fuzztest::InRange(1e-6, 1.0));
