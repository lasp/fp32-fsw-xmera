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
                    std::vector<float>{2.3, -2.4, 2.5, -2.6},
                    std::vector<float>{2.7, -2.8, 2.9, -3.0},
                    std::vector<float>{0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2},
                    std::vector<float>{10.0, 1.0, 1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 10.0},  // valid SPD inertia
                    false,
                    0.1);
}

TEST(MrpFeedbackTest, SetupTest) { testMrpFeedbackSetup(); }

TEST(MrpFeedbackTest, IntegralFeedbackDisabledWhenKiIsZero) {
    // With Ki = 0, the integral feedback torque must be zero on every cycle.
    const std::vector<float> isc{1000.0F, 0.0F, 0.0F, 0.0F, 800.0F, 0.0F, 0.0F, 0.0F, 800.0F};
    const Eigen::Matrix3f ISCPntB_B = cArrayToEigenMatrix3(isc.data());
    const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(1.0F,
                                                            0.5F,
                                                            0.0F,
                                                            1.0F,
                                                            ControlLawType::NORMAL,
                                                            Eigen::Vector3f::Zero(),
                                                            ISCPntB_B,
                                                            /*numRW=*/0,
                                                            Eigen::Matrix<float, 3, RW_EFF_CNT>::Zero(),
                                                            std::array<float, RW_EFF_CNT>{});
    MrpFeedbackAlgorithm alg(cfg);

    MrpFeedbackGuidInput guid;
    guid.sigma_BR = Eigen::Vector3f{0.4F, 0.1F, -0.3F};
    guid.omega_BR_B = Eigen::Vector3f{-0.4F, 0.5F, -0.6F};
    guid.omega_RN_B = Eigen::Vector3f{0.7F, -0.8F, 0.9F};
    guid.domega_RN_B = Eigen::Vector3f{-1.0F, 1.1F, -1.2F};

    const Eigen::Vector<float, RW_EFF_CNT> wheelSpeeds = Eigen::Vector<float, RW_EFF_CNT>::Zero();
    const std::array<bool, RW_EFF_CNT> availability{};

    EXPECT_NO_THROW(alg.reset());
    for (int step = 0; step < 5; ++step) {
        const auto callTime = static_cast<uint64_t>(step + 1) * static_cast<uint64_t>(0.1F / kNano2Sec);
        MrpFeedbackOutput out{};
        EXPECT_NO_THROW(out = alg.update(callTime, guid, wheelSpeeds, availability));
        for (int i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(out.intFeedbackTorque[i], 0.0F);
            EXPECT_TRUE(std::isfinite(out.controlTorque[i]));
        }
    }
}

TEST(MrpFeedbackTest, IntegralLimitClampsLargeError) {
    // Drive a sustained sigma_BR with Ki > 0 and a tight integralLimit so the integral state
    // saturates within a few steps. After saturation, |int_sigma_i| should equal integralLimit.
    constexpr float K = 1.0F;
    constexpr float Ki = 1.0F;
    constexpr float intLimit = 0.5F;
    const std::vector<float> isc{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
    const Eigen::Matrix3f ISCPntB_B = cArrayToEigenMatrix3(isc.data());
    const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(K,
                                                            1.0F,
                                                            Ki,
                                                            intLimit,
                                                            ControlLawType::NORMAL,
                                                            Eigen::Vector3f::Zero(),
                                                            ISCPntB_B,
                                                            /*numRW=*/0,
                                                            Eigen::Matrix<float, 3, RW_EFF_CNT>::Zero(),
                                                            std::array<float, RW_EFF_CNT>{});
    MrpFeedbackAlgorithm alg(cfg);

    MrpFeedbackGuidInput guid;
    guid.sigma_BR = Eigen::Vector3f{1.0F, 1.0F, 1.0F};
    guid.omega_BR_B = Eigen::Vector3f::Zero();
    guid.omega_RN_B = Eigen::Vector3f::Zero();
    guid.domega_RN_B = Eigen::Vector3f::Zero();

    const Eigen::Vector<float, RW_EFF_CNT> wheelSpeeds = Eigen::Vector<float, RW_EFF_CNT>::Zero();
    const std::array<bool, RW_EFF_CNT> availability{};

    EXPECT_NO_THROW(alg.reset());

    // Drive enough integration steps to saturate (each step accumulates K*dt*sigma = 1.0 * 1.0 * 1.0 in each axis).
    constexpr float dt = 1.0F;
    constexpr int steps = 10;
    MrpFeedbackOutput out{};
    for (int step = 0; step < steps; ++step) {
        const auto callTime = static_cast<uint64_t>(step + 1) * static_cast<uint64_t>(dt / kNano2Sec);
        EXPECT_NO_THROW(out = alg.update(callTime, guid, wheelSpeeds, availability));
        for (int i = 0; i < 3; ++i) {
            EXPECT_TRUE(std::isfinite(out.controlTorque[i]));
            EXPECT_TRUE(std::isfinite(out.intFeedbackTorque[i]));
        }
    }
    // After saturation, the integral feedback torque magnitude per axis is bounded by P*Ki*intLimit.
    constexpr float bound = 1.0F * Ki * intLimit + 1e-5F;  // P=1 in this test
    for (int i = 0; i < 3; ++i) {
        EXPECT_LE(std::abs(out.intFeedbackTorque[i]), bound);
    }
}
