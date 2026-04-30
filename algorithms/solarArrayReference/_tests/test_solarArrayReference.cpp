#include "solarArrayReferenceTestHelpers.hpp"

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, RegressionTest) {
    regressionTestSolarArrayReference({0.1F, 0.2F, 0.3F},    // sigma_BN
                                      {0.3F, 0.2F, 0.1F},    // sigma_RN
                                      {1.0F, 0.0F, 0.0F},    // vehSunPntBdy
                                      {1.0F, 0.0F, 0.0F},    // a1Hat_B
                                      {0.0F, 1.0F, 0.0F},    // a2Hat_B
                                      0.0F                    // theta
    );
}

TEST(SolarArrayReferenceTest, RegressionTestNonZeroTheta) {
    regressionTestSolarArrayReference({0.5F, 0.4F, 0.3F},    // sigma_BN
                                      {0.9F, 0.7F, 0.8F},    // sigma_RN
                                      {0.0F, 0.0F, 1.0F},    // vehSunPntBdy
                                      {1.0F, 0.0F, 0.0F},    // a1Hat_B
                                      {0.0F, 1.0F, 0.0F},    // a2Hat_B
                                      1.5F                    // theta
    );
}

TEST(SolarArrayReferenceTest, RegressionTestArbitraryAxes) {
    regressionTestSolarArrayReference({0.1F, -0.3F, 0.2F},   // sigma_BN
                                      {0.2F, 0.1F, -0.1F},   // sigma_RN
                                      {1.0F, 1.0F, 1.0F},    // vehSunPntBdy
                                      {0.0F, 0.0F, 1.0F},    // a1Hat_B
                                      {1.0F, 0.0F, 0.0F},    // a2Hat_B
                                      -0.5F                   // theta
    );
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, SetupTest) {
    SolarArrayReferenceAlgorithm alg{};

    // a1Hat_B: zero vector should throw
    EXPECT_THROW(alg.setA1Hat_B(Eigen::Vector3f::Zero()), fsw::invalid_argument);

    // a2Hat_B: zero vector should throw
    EXPECT_THROW(alg.setA2Hat_B(Eigen::Vector3f::Zero()), fsw::invalid_argument);

    // a1Hat_B: verify auto-normalization
    alg.setA1Hat_B(Eigen::Vector3f{2.0F, 0.0F, 0.0F});
    Eigen::Vector3f retrieved = alg.getA1Hat_B();
    EXPECT_NEAR(retrieved(0), 1.0F, 1e-6F);
    EXPECT_NEAR(retrieved(1), 0.0F, 1e-6F);
    EXPECT_NEAR(retrieved(2), 0.0F, 1e-6F);

    // a2Hat_B: verify auto-normalization
    alg.setA2Hat_B(Eigen::Vector3f{3.0F, 4.0F, 0.0F});
    retrieved = alg.getA2Hat_B();
    EXPECT_NEAR(retrieved.norm(), 1.0F, 1e-6F);
    EXPECT_NEAR(retrieved(0), 0.6F, 1e-6F);
    EXPECT_NEAR(retrieved(1), 0.8F, 1e-6F);
}
