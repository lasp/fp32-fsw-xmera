#include "attTrackingErrorTestHelpers.hpp"
#include "fp32-fsw-xmera/algorithms/attTrackingError/attTrackingErrorTypes.h"

#include <gtest/gtest.h>
#include <cmath>

TEST(attTrackingErrorTest, RegressionTest) {
    regressionTestAttTrackingError(std::vector<float>{0.2, -0.1, -0.4},
                                   std::vector<float>{-0.1, -0.4, 0.0},
                                   std::vector<float>{0.009, 0.007, -0.006},
                                   std::vector<float>{0.08, -0.001, -0.003},
                                   std::vector<float>{0.01, 0.02, -0.04});
}

TEST(attTrackingErrorTest, ZeroPropertyTest) {
    // --- Test module with targeted inputs to ensure individual terms are properly implemented ---
    AttTrackingErrorAlgorithm alg{};

    AttNavInput navIn{};
    AttRefInput refIn{};

    navIn.sigma_BN << 0.2, -0.1, -0.4;
    refIn.sigma_RN = navIn.sigma_BN;

    navIn.omega_BN_B << 0.0, 0.0, 0.0;
    refIn.omega_RN_N << 0.0, 0.0, 0.0;
    refIn.domega_RN_N << 0.0, 0.0, 0.0;

    AttGuidOutput out = alg.update(navIn, refIn);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(out.sigma_BR[i], 0.0, 1e-6);
        EXPECT_NEAR(out.omega_BR_B[i], 0.0, 1e-6);

        // Finiteness
        EXPECT_TRUE(std::isfinite(out.sigma_BR[i]));
        EXPECT_TRUE(std::isfinite(out.omega_BR_B[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_B[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_B[i]));
    }
}

TEST(attTrackingErrorTest, IdentityPropertyTest) {
    AttTrackingErrorAlgorithm alg{};

    AttNavInput navIn{};
    AttRefInput refIn{};

    navIn.sigma_BN << 0.0, 0.0, 0.0;
    refIn.omega_RN_N << 0.08, -0.001, -0.003;
    refIn.domega_RN_N << -0.1, -0.4, 0.0;

    AttGuidOutput out = alg.update(navIn, refIn);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(out.omega_RN_B[i], refIn.omega_RN_N[i], 1e-6);
        EXPECT_NEAR(out.domega_RN_B[i], refIn.domega_RN_N[i], 1e-6);

        EXPECT_TRUE(std::isfinite(out.omega_RN_B[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_B[i]));
    }
}
