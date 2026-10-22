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

// A full queue drops its oldest measurement to make room for a newer one
TEST(MeasurementQueue, FullQueueDropsTheOldestMeasurement) {
    Queue q;
    EXPECT_TRUE(q.enqueue(1.0, 10));
    EXPECT_TRUE(q.enqueue(2.0, 20));
    EXPECT_TRUE(q.enqueue(3.0, 30));
    EXPECT_TRUE(q.enqueue(4.0, 40));
    EXPECT_TRUE(q.isFull());

    EXPECT_TRUE(q.enqueue(5.0, 50)) << "a newer measurement must enter a full queue";
    EXPECT_TRUE(q.isFull());
    for (double const expected : {2.0, 3.0, 4.0, 5.0}) {
        auto const entry = q.popEarliest();
        ASSERT_TRUE(entry.has_value());
        EXPECT_DOUBLE_EQ(entry->first, expected) << "the oldest measurement (1.0) should have been dropped";
    }
}

// A measurement older than everything in a full queue is itself the oldest, so it is the one dropped
TEST(MeasurementQueue, FullQueueDropsANewMeasurementOlderThanAllQueued) {
    Queue q;
    for (double const t : {2.0, 3.0, 4.0, 5.0}) {
        q.enqueue(t, 0);
    }
    EXPECT_FALSE(q.enqueue(1.0, 10));
    auto const earliest = q.popEarliest();
    ASSERT_TRUE(earliest.has_value());
    EXPECT_DOUBLE_EQ(earliest->first, 2.0);
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
    EXPECT_FALSE(q.isTimeSynchronized());
}

// A new queue is not synchronized; setting the time of last measurement synchronizes it, even to zero
TEST(MeasurementQueue, SetTimeOfLastMeasurementSynchronizes) {
    Queue q;
    EXPECT_FALSE(q.isTimeSynchronized());
    q.setTimeOfLastMeasurement(0.0);
    EXPECT_TRUE(q.isTimeSynchronized());
}

// With a delay, popEarliestDue returns a measurement only once callTime - delay has reached its time tag
TEST(MeasurementQueue, PopEarliestDueWaitsForTheDelay) {
    Queue q;
    q.setDelay(3.0);
    q.enqueue(7.0, 70);
    q.enqueue(8.0, 80);

    EXPECT_FALSE(q.popEarliestDue(9.0).has_value()) << "7.0 is due at 10.0";
    auto const due = q.popEarliestDue(10.0);
    ASSERT_TRUE(due.has_value()) << "a measurement exactly at callTime - delay is due";
    EXPECT_DOUBLE_EQ(due->first, 7.0);
    EXPECT_FALSE(q.popEarliestDue(10.0).has_value()) << "8.0 is not due yet and stays queued";
    EXPECT_FALSE(q.isEmpty());
}

// With the default zero delay, every queued measurement is due, even one with a time tag after callTime
TEST(MeasurementQueue, PopEarliestDueWithoutDelayReturnsEveryMeasurement) {
    Queue q;
    EXPECT_DOUBLE_EQ(q.getDelay(), 0.0);
    q.enqueue(1.0, 10);
    q.enqueue(20.0, 200);

    EXPECT_TRUE(q.popEarliestDue(10.0).has_value());
    EXPECT_TRUE(q.popEarliestDue(10.0).has_value());
    EXPECT_TRUE(q.isEmpty());
}

// The delay is configuration: clear() resets the queue state but keeps it
TEST(MeasurementQueue, ClearKeepsTheDelay) {
    Queue q;
    q.setDelay(3.0);
    q.enqueue(1.0, 10);
    q.clear();
    EXPECT_DOUBLE_EQ(q.getDelay(), 3.0);
}

// dropOldest removes the earliest measurement and leaves the others in order
TEST(MeasurementQueue, DropOldestRemovesTheEarliestMeasurement) {
    Queue q;
    q.dropOldest();  // empty: nothing to drop
    EXPECT_TRUE(q.isEmpty());

    q.enqueue(2.0, 20);
    q.enqueue(1.0, 10);
    q.enqueue(3.0, 30);
    q.dropOldest();

    auto const first = q.popEarliest();
    auto const second = q.popEarliest();
    ASSERT_TRUE(first.has_value() && second.has_value());
    EXPECT_DOUBLE_EQ(first->first, 2.0);
    EXPECT_DOUBLE_EQ(second->first, 3.0);
    EXPECT_TRUE(q.isEmpty());
}

}  // namespace filtering
