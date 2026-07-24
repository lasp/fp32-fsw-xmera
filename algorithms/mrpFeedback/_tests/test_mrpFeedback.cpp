#include "mrpFeedbackTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MrpFeedbackTest, ReferenceTest) {
    testMrpFeedback(Eigen::Vector3f{0.4, 0.1, -0.3},
                    1.0,
                    0.4,
                    0.1,
                    1.1,
                    0,
                    Eigen::Vector3f{0.1, -0.2, 0.3},
                    Eigen::Vector3f{-0.4, 0.5, -0.6},
                    Eigen::Vector3f{0.7, -0.8, 0.9},
                    Eigen::Vector3f{-1.0, 1.1, -1.2},
                    std::vector<float>{1.9, -2.0, 2.1, -2.2},
                    std::vector<bool>{false, true, false, false},
                    2,
                    std::vector<float>{2.7, -2.8, 2.9, -3.0},
                    std::vector<float>{0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2},
                    std::vector<float>{1000.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 800.0},
                    true,
                    0.1);
}

TEST(MrpFeedbackTest, SetupTest) { testMrpFeedbackSetup(); }

TEST(MrpFeedbackTest, IntegralFeedbackDisabledWhenKiIsZero) {
    // With Ki = 0, the integral feedback torque must be zero on every cycle.
    const MrpFeedbackControlParameters params{
        .K = 1.0F,
        .P = 0.5F,
        .Ki = 0.0F,
        .integralLimit = 1.0F,
        .controlLawType = ControlLawType::NORMAL,
        .controlPeriod = 0.1F,
    };
    Eigen::Matrix3f inertia{};
    inertia << 1000.0F, 0.0F, 0.0F, 0.0F, 800.0F, 0.0F, 0.0F, 0.0F, 800.0F;
    const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(params, Eigen::Vector3f::Zero(), inertia);
    MrpFeedbackAlgorithm alg(cfg);

    AttGuidMsgF32Payload guidCmd{};
    eigenVectorToCArray(Eigen::Vector3f{0.4F, 0.1F, -0.3F}, guidCmd.sigma_BR);
    eigenVectorToCArray(Eigen::Vector3f{-0.4F, 0.5F, -0.6F}, guidCmd.omega_BR_B);
    eigenVectorToCArray(Eigen::Vector3f{0.7F, -0.8F, 0.9F}, guidCmd.omega_RN_B);
    eigenVectorToCArray(Eigen::Vector3f{-1.0F, 1.1F, -1.2F}, guidCmd.domega_RN_B);

    const RWSpeedMsgF32Payload wheelSpeeds{};
    const RWAvailabilityMsgPayload availability{};

    EXPECT_NO_THROW(alg.reset());
    for (int step = 0; step < 5; ++step) {
        MrpFeedbackOutput out{};
        EXPECT_NO_THROW(out = alg.update(guidCmd, wheelSpeeds, availability));
        for (int i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(out.intFeedbackOut.torqueRequestBody[i], 0.0F);
            EXPECT_TRUE(std::isfinite(out.controlOut.torqueRequestBody[i]));
        }
    }
}

TEST(MrpFeedbackTest, IntegralLimitClampsLargeError) {
    // Drive a sustained sigma_BR with Ki > 0 and a tight integralLimit so the integral state
    // saturates within a few steps. After saturation, |int_sigma_i| should equal integralLimit.
    constexpr float K = 1.0F;
    constexpr float Ki = 1.0F;
    constexpr float intLimit = 0.5F;
    const MrpFeedbackControlParameters params{
        .K = K,
        .P = 1.0F,
        .Ki = Ki,
        .integralLimit = intLimit,
        .controlLawType = ControlLawType::NORMAL,
        .controlPeriod = 1.0F,
    };
    const MrpFeedbackConfig cfg =
        MrpFeedbackConfig::create(params, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity());
    MrpFeedbackAlgorithm alg(cfg);

    AttGuidMsgF32Payload guidCmd{};
    eigenVectorToCArray(Eigen::Vector3f{1.0F, 1.0F, 1.0F}, guidCmd.sigma_BR);
    eigenVectorToCArray(Eigen::Vector3f::Zero(), guidCmd.omega_BR_B);
    eigenVectorToCArray(Eigen::Vector3f::Zero(), guidCmd.omega_RN_B);
    eigenVectorToCArray(Eigen::Vector3f::Zero(), guidCmd.domega_RN_B);

    const RWSpeedMsgF32Payload wheelSpeeds{};
    const RWAvailabilityMsgPayload availability{};

    EXPECT_NO_THROW(alg.reset());

    // Drive enough integration steps to saturate (each step accumulates K*controlPeriod*sigma = 1.0 per axis).
    constexpr int steps = 10;
    MrpFeedbackOutput out{};
    for (int step = 0; step < steps; ++step) {
        EXPECT_NO_THROW(out = alg.update(guidCmd, wheelSpeeds, availability));
        for (int i = 0; i < 3; ++i) {
            EXPECT_TRUE(std::isfinite(out.controlOut.torqueRequestBody[i]));
            EXPECT_TRUE(std::isfinite(out.intFeedbackOut.torqueRequestBody[i]));
        }
    }
    // After saturation, the integral feedback torque magnitude per axis is bounded by P*Ki*intLimit.
    constexpr float bound = 1.0F * Ki * intLimit + 1e-5F;  // P=1 in this test
    for (int i = 0; i < 3; ++i) {
        EXPECT_LE(std::abs(out.intFeedbackOut.torqueRequestBody[i]), bound);
    }
}
