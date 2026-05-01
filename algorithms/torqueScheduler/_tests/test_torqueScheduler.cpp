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
