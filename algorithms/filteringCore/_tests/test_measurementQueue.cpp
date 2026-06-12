// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for filtering::measurement_queue<Measurement, Capacity>.

#include <filteringCore/measurementQueue.h>

#include <gtest/gtest.h>

namespace filtering {
namespace {

// Trivial payload for the queue's value type. The queue never inspects the
// measurement itself; it only orders by timeTag, so an int is sufficient.
using TestMeasurement = int;
constexpr std::size_t kCapacity = 4;
using Queue = measurement_queue<TestMeasurement, kCapacity>;

}  // namespace

// Make empty queue and check it is empty
TEST(MeasurementQueue, DefaultConstructedQueueIsEmpty) {
    Queue q;
    EXPECT_TRUE(q.isEmpty());
    EXPECT_FALSE(q.isFull());
    EXPECT_FALSE(q.popEarliest().has_value());
}

// Max out the queue and see it fail to add another element
TEST(MeasurementQueue, EnqueueReturnsFalseWhenFull) {
    Queue q;
    EXPECT_TRUE(q.enqueue(1.0, 10));
    EXPECT_TRUE(q.enqueue(2.0, 20));
    EXPECT_TRUE(q.enqueue(3.0, 30));
    EXPECT_TRUE(q.enqueue(4.0, 40));
    EXPECT_TRUE(q.isFull());
    EXPECT_FALSE(q.enqueue(5.0, 50));
}

// Populate elements out of order and check that pop earliest returns them in chronological order
TEST(MeasurementQueue, PopEarliestReturnsItemsInAscendingTimeTag) {
    Queue q;
    q.enqueue(3.0, 30);
    q.enqueue(1.0, 10);
    q.enqueue(2.0, 20);

    auto p1 = q.popEarliest();
    auto p2 = q.popEarliest();
    auto p3 = q.popEarliest();
    ASSERT_TRUE(p1.has_value() && p2.has_value() && p3.has_value());

    EXPECT_DOUBLE_EQ(p1->first, 1.0);
    EXPECT_EQ(p1->second, 10);
    EXPECT_DOUBLE_EQ(p2->first, 2.0);
    EXPECT_EQ(p2->second, 20);
    EXPECT_DOUBLE_EQ(p3->first, 3.0);
    EXPECT_EQ(p3->second, 30);
    EXPECT_TRUE(q.isEmpty());
}

// Reset queue
TEST(MeasurementQueue, ClearEmptiesQueueAndResetsTimeAnchor) {
    Queue q;
    q.enqueue(1.0, 10);
    q.enqueue(2.0, 20);
    q.setTimeOfLastMeasurement(5.0);

    q.clear();

    EXPECT_TRUE(q.isEmpty());
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 0.0);
}

}  // namespace filtering
