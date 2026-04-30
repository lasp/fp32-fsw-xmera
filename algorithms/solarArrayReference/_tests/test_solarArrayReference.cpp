#include "solarArrayReferenceTestHelpers.hpp"
#include <numbers>

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
    
    // Zero drive axis should throw (norm far from 1.0)
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}),
                 fsw::invalid_argument);

    // Zero surface normal should throw
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f::Zero()),
                 fsw::invalid_argument);

    // Non-unit drive axis (norm far from 1.0) should throw
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{2.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F}),
                 fsw::invalid_argument);

    // Non-unit surface normal should throw
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 3.0F, 0.0F}),
                 fsw::invalid_argument);

    // Non-orthogonal axes should throw
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                         Eigen::Vector3f{1.0F, 1.0F, 0.0F}.normalized()),
                 fsw::invalid_argument);

    // Alignment threshold: negative should throw
    EXPECT_THROW(alg.setAlignmentThreshold(-0.01F), fsw::invalid_argument);

    // Alignment threshold: above pi/2 should throw
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    EXPECT_THROW(alg.setAlignmentThreshold(halfPi + 0.01F), fsw::invalid_argument);

    // Valid alignment threshold should not throw
    EXPECT_NO_THROW(alg.setAlignmentThreshold(0.0F));
    EXPECT_NO_THROW(alg.setAlignmentThreshold(halfPi));

    // Alignment threshold round-trip
    alg.setAlignmentThreshold(0.05F);
    EXPECT_FLOAT_EQ(alg.getAlignmentThreshold(), 0.05F);

    // Valid orthogonal unit axes should not throw
    EXPECT_NO_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F}));

    // Getter round-trip
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    const auto axes = alg.getSolarArrayAxes_B();
    EXPECT_NEAR(axes[0](0), 1.0F, 1e-6F);
    EXPECT_NEAR(axes[0](1), 0.0F, 1e-6F);
    EXPECT_NEAR(axes[0](2), 0.0F, 1e-6F);
    EXPECT_NEAR(axes[1](0), 0.0F, 1e-6F);
    EXPECT_NEAR(axes[1](1), 1.0F, 1e-6F);
    EXPECT_NEAR(axes[1](2), 0.0F, 1e-6F);
}
