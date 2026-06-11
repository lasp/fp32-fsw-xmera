// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz tests for filtering::measurement_queue. Invariants
// (sort order, capacity, payload conservation) hold for ALL enqueue
// orderings, so the fuzzer drives randomized timeTags rather than the
// handful of hand-picked permutations in test_measurementQueue.cpp.

#include <filteringCore/measurementQueue.h>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <set>

namespace filtering {
namespace {

constexpr std::size_t kCapacity = 4;
using Queue = measurement_queue<int, kCapacity>;

}  // namespace

// For any 4 timeTags, the queue holds the max capacity of items, empties them in ascending
// timeTag order, ends empty, and preserves every payload exactly once.
void fuzzQueueInvariants(double t0, double t1, double t2, double t3) {
    Queue q;
    q.enqueue(t0, 0);
    q.enqueue(t1, 1);
    q.enqueue(t2, 2);
    q.enqueue(t3, 3);
    EXPECT_TRUE(q.isFull()) << "CAPACITY enqueues should saturate";

    auto p0 = q.popEarliest();
    auto p1 = q.popEarliest();
    auto p2 = q.popEarliest();
    auto p3 = q.popEarliest();
    ASSERT_TRUE(p0 && p1 && p2 && p3);

    EXPECT_LE(p0->first, p1->first) << "pop order not ascending";
    EXPECT_LE(p1->first, p2->first) << "pop order not ascending";
    EXPECT_LE(p2->first, p3->first) << "pop order not ascending";
    EXPECT_TRUE(q.isEmpty()) << "drained queue not empty";

    std::set<int> popped = {p0->second, p1->second, p2->second, p3->second};
    EXPECT_EQ(popped.size(), 4u) << "payload lost or duplicated";
}
FUZZ_TEST(MeasurementQueueFuzz, fuzzQueueInvariants)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::Finite<double>(),
                 fuzztest::Finite<double>(),
                 fuzztest::Finite<double>());

}  // namespace filtering
