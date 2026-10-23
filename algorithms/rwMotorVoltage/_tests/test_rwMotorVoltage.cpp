#include "rwMotorVoltageAlgorithm.h"

#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

namespace {

constexpr uint64_t kOneSecNs = 1'000'000'000ULL;
constexpr float kJs = 0.1F;           // [kg m^2] spin-axis inertia
constexpr float kUMax = 0.2F;         // [Nm] max motor torque
constexpr float kVoltageMax = 10.0F;  // [V]
constexpr float kTorqueCmd = 0.05F;   // [Nm]
constexpr float kGainK = 1.0F;        // [V/Nm]

// Feed-forward only: voltageMax / uMax * torque (voltageMin = 0).
constexpr float kFeedForwardVoltage = kVoltageMax / kUMax * kTorqueCmd;

class RwMotorVoltageDtTest : public ::testing::Test {
   protected:
    void SetUp() override {
        RwMotorVoltageRWConfig config{};
        config.numRW = 1;
        config.JsList[0] = kJs;
        config.uMax[0] = kUMax;
        alg.setGainK(kGainK);
        alg.reset(config);
        // Two warm-up calls at a constant speed. The first call latches priorTime. The second call clears
        // the reset flag. The two calls do not apply feedback. Start at 1 s so that priorTime is not zero.
        (void)step(1U * kOneSecNs, 0.0F);
        (void)step(2U * kOneSecNs, 0.0F);
    }

    // One update with a new torque command (update() changes the command in place).
    float step(uint64_t callTime, float wheelSpeed) {
        RwMotorVoltageTorqueInput torque{};
        torque.motorTorque[0] = kTorqueCmd;
        RwMotorVoltageAvailInput avail{};
        avail.wheelAvailability[0] = RW_MOTOR_VOLTAGE_AVAILABLE;
        RwMotorVoltageSpeedInput speed{};
        speed.wheelSpeeds[0] = wheelSpeed;
        return alg.update(callTime, torque, avail, speed, true).voltage[0];
    }

    RwMotorVoltageAlgorithm alg{0.0F, kVoltageMax};
};

// Baseline: a 1 s step with a 1 rad/s speed change gives OmegaDot = 1 rad/s^2, so the feedback
// term K * (Js * OmegaDot - torque) = 0.05 Nm cancels the 0.05 Nm command exactly.
TEST_F(RwMotorVoltageDtTest, ForwardStepAppliesFeedback) { EXPECT_FLOAT_EQ(step(3U * kOneSecNs, 1.0F), 0.0F); }

// A repeated callTime gives a period of 0 s. The algorithm does not apply the feedback, and thus does
// not divide by zero. It latches the latest speed, and the next step uses that speed.
TEST_F(RwMotorVoltageDtTest, RepeatedCallTimeSkipsFeedback) {
    const float repeated = step(2U * kOneSecNs, 1.0F);
    EXPECT_TRUE(std::isfinite(repeated));
    EXPECT_FLOAT_EQ(repeated, kFeedForwardVoltage);

    // The speed did not change after the latch. Thus OmegaDot = 0, and the feedback adds K * torque to
    // the command.
    EXPECT_FLOAT_EQ(step(3U * kOneSecNs, 1.0F), 2.0F * kFeedForwardVoltage);
}

// A backwards clock gives a period of 0 s. The algorithm does not apply the feedback and does not use
// the wrapped unsigned difference (approximately 1.8e10 s). It latches the latest speed and time.
TEST_F(RwMotorVoltageDtTest, BackwardsClockSkipsFeedback) {
    const uint64_t earlier = 3U * kOneSecNs / 2U;  // 1.5 s, before the 2 s prior call
    const float backwards = step(earlier, 1.0F);
    EXPECT_TRUE(std::isfinite(backwards));
    EXPECT_FLOAT_EQ(backwards, kFeedForwardVoltage);

    // priorTime is 1.5 s and the latched speed is 1 rad/s. A 1 s step at the same speed gives OmegaDot = 0.
    EXPECT_FLOAT_EQ(step(earlier + kOneSecNs, 1.0F), 2.0F * kFeedForwardVoltage);
}

}  // namespace
