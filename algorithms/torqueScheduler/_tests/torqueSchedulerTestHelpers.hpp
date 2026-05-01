// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TEST_TORQUE_SCHEDULER_HELPERS_H
#define TEST_TORQUE_SCHEDULER_HELPERS_H

#include "torqueSchedulerAlgorithm.h"
#include "torqueSchedulerTypes.h"

#include <gtest/gtest.h>
#include <array>

// Reference computation, written independently from the algorithm so a divergence catches a
// regression in either implementation. Mirrors the schedule logic in update().
inline std::array<int, 2> referenceLockFlags(LockFlag lockFlag, float t, float tSwitch) {
    switch (lockFlag) {
        case LockFlag::BothFree:
            return {0, 0};
        case LockFlag::LockSecondThenFirst:
            return (t > tSwitch) ? std::array<int, 2>{1, 0} : std::array<int, 2>{0, 1};
        case LockFlag::LockFirstThenSecond:
            return (t > tSwitch) ? std::array<int, 2>{0, 1} : std::array<int, 2>{1, 0};
        case LockFlag::BothLocked:
            return {1, 1};
    }
    return {0, 0};
}

inline void testTorqueScheduler(LockFlag lockFlag, float tSwitch, float t, float t1, float t2) {
    const TorqueSchedulerConfig cfg = TorqueSchedulerConfig::create(lockFlag, tSwitch);
    const TorqueSchedulerAlgorithm alg(cfg);

    ArrayMotorTorqueMsgF32Payload motorTorque1{};
    motorTorque1.motorTorque[0] = t1;
    ArrayMotorTorqueMsgF32Payload motorTorque2{};
    motorTorque2.motorTorque[0] = t2;

    TorqueSchedulerOutput out;
    EXPECT_NO_THROW(out = alg.update(t, motorTorque1, motorTorque2));

    EXPECT_FLOAT_EQ(out.motorTorqueOut.motorTorque[0], t1);
    EXPECT_FLOAT_EQ(out.motorTorqueOut.motorTorque[1], t2);

    const std::array<int, 2> expectedLock = referenceLockFlags(lockFlag, t, tSwitch);
    EXPECT_EQ(out.effectorLockOut.effectorLockFlag[0], expectedLock[0]);
    EXPECT_EQ(out.effectorLockOut.effectorLockFlag[1], expectedLock[1]);
}

inline void testTorqueSchedulerSetup() {
    // Valid config builds without throwing.
    EXPECT_NO_THROW({
        const TorqueSchedulerConfig cfg = TorqueSchedulerConfig::create(LockFlag::BothFree, 0.0F);
        const TorqueSchedulerAlgorithm alg(cfg);
        (void)alg;
    });

    // Invalid: lockFlag value outside the four enumerators.
    EXPECT_ANY_THROW({ (void)TorqueSchedulerConfig::create(static_cast<LockFlag>(99), 1.0F); });

    // Invalid: tSwitch must be >= 0.
    EXPECT_ANY_THROW({ (void)TorqueSchedulerConfig::create(LockFlag::BothFree, -0.1F); });
}

#endif  // TEST_TORQUE_SCHEDULER_HELPERS_H
