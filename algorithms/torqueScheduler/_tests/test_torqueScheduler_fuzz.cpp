// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueSchedulerTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

namespace {
void fuzzTorqueScheduler(int lockFlagInt, float tSwitch, float t, float t1, float t2) {
    testTorqueScheduler(static_cast<LockFlag>(lockFlagInt), tSwitch, t, t1, t2);
}
}  // namespace

FUZZ_TEST(TorqueSchedulerFuzz, fuzzTorqueScheduler)
    .WithDomains(fuzztest::InRange(0, 3),         // lockFlagInt: only the four valid enumerators
                 fuzztest::InRange(0.0F, 1e3F),   // tSwitch: must be >= 0 per Config validator
                 fuzztest::InRange(-1e3F, 1e4F),  // t: typical sim-elapsed range, allow some negative slack
                 fuzztest::InRange(-1e3F, 1e3F),  // motorTorque1[0]
                 fuzztest::InRange(-1e3F, 1e3F)   // motorTorque2[0]
    );
