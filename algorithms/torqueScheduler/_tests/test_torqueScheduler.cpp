// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueSchedulerTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TorqueSchedulerTest, Setup) { testTorqueSchedulerSetup(); }

TEST(TorqueSchedulerTest, BothFree) { testTorqueScheduler(LockFlag::BothFree, 5.0F, 2.0F, 1.0F, 3.0F); }

TEST(TorqueSchedulerTest, LockSecondThenFirstBefore) {
    testTorqueScheduler(LockFlag::LockSecondThenFirst, 5.0F, 2.0F, 1.0F, 3.0F);
}

TEST(TorqueSchedulerTest, LockSecondThenFirstAfter) {
    testTorqueScheduler(LockFlag::LockSecondThenFirst, 5.0F, 7.0F, 1.0F, 3.0F);
}

TEST(TorqueSchedulerTest, LockFirstThenSecondBefore) {
    testTorqueScheduler(LockFlag::LockFirstThenSecond, 5.0F, 2.0F, 1.0F, 3.0F);
}

TEST(TorqueSchedulerTest, LockFirstThenSecondAfter) {
    testTorqueScheduler(LockFlag::LockFirstThenSecond, 5.0F, 7.0F, 1.0F, 3.0F);
}

TEST(TorqueSchedulerTest, BothLocked) { testTorqueScheduler(LockFlag::BothLocked, 5.0F, 2.0F, 1.0F, 3.0F); }

TEST(TorqueSchedulerTest, TransitionAtExactSwitch) {
    // The transition uses strict `t > tSwitch`, so t == tSwitch falls in the "before" branch.
    testTorqueScheduler(LockFlag::LockSecondThenFirst, 5.0F, 5.0F, 0.0F, 0.0F);
    testTorqueScheduler(LockFlag::LockFirstThenSecond, 5.0F, 5.0F, 0.0F, 0.0F);
}

TEST(TorqueSchedulerTest, ZeroSwitchTimeImmediateTransition) {
    // tSwitch == 0 with t > 0 immediately selects the "after" branch.
    testTorqueScheduler(LockFlag::LockSecondThenFirst, 0.0F, 1.0F, 0.0F, 0.0F);
    testTorqueScheduler(LockFlag::LockFirstThenSecond, 0.0F, 1.0F, 0.0F, 0.0F);
}

TEST(TorqueSchedulerTest, MotorTorquePassThroughIsExact) {
    // The motor-torque path is a pure copy of element [0]; verify both directions of sign and a
    // zero pass through unchanged.
    const TorqueSchedulerConfig cfg = TorqueSchedulerConfig::create(LockFlag::BothFree, 0.0F);
    const TorqueSchedulerAlgorithm alg(cfg);

    ArrayMotorTorqueMsgF32Payload m1{};
    ArrayMotorTorqueMsgF32Payload m2{};
    for (const float v : {0.0F, 1.0F, -1.0F, 1.0e-6F, 1.0e6F}) {
        m1.motorTorque[0] = v;
        m2.motorTorque[0] = -v;
        const TorqueSchedulerOutput out = alg.update(0.0F, m1, m2);
        EXPECT_FLOAT_EQ(out.motorTorqueOut.motorTorque[0], v);
        EXPECT_FLOAT_EQ(out.motorTorqueOut.motorTorque[1], -v);
    }
}

TEST(TorqueSchedulerTest, EffectorLockFlagsAreAlwaysBinary) {
    // Property: regardless of inputs / config, every emitted effectorLockFlag entry that the
    // algorithm sets is in {0, 1}. (Indices >= 2 stay at the zero-init default.)
    const ArrayMotorTorqueMsgF32Payload m1{};
    const ArrayMotorTorqueMsgF32Payload m2{};

    for (const LockFlag flag :
         {LockFlag::BothFree, LockFlag::LockSecondThenFirst, LockFlag::LockFirstThenSecond, LockFlag::BothLocked}) {
        const TorqueSchedulerConfig cfg = TorqueSchedulerConfig::create(flag, 1.0F);
        const TorqueSchedulerAlgorithm alg(cfg);
        for (const float t : {0.0F, 0.5F, 1.0F, 1.5F, 100.0F}) {
            const TorqueSchedulerOutput out = alg.update(t, m1, m2);
            EXPECT_TRUE(out.effectorLockOut.effectorLockFlag[0] == 0 || out.effectorLockOut.effectorLockFlag[0] == 1);
            EXPECT_TRUE(out.effectorLockOut.effectorLockFlag[1] == 0 || out.effectorLockOut.effectorLockFlag[1] == 1);
        }
    }
}
