// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for filtering::applySequential() and applySequentialRobust(): both drain a
// measurement_queue in chronological order, interleaving timeUpdate / measurementUpdate
// calls then a final timeUpdate to callTime. applySequentialRobust additionally reads each
// update's bool result and, on a failed update, calls clear() and holds the anchor.
// The first call on a queue that is not synchronized only sets the anchor to callTime, so the
// scheduling tests start with a queue synchronized at t = 0.

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace filtering {
namespace {

// Call Log to check when we enter time / measurement updates and clear().
struct CallLog {
    enum class Kind { TimeUpdate, MeasurementUpdate, Clear };
    std::vector<std::pair<Kind, double>> entries;
};

// Filter which records the time/measurement update and clear() calls. By default both
// updates return true (valid) so the schedulers follow their success path; the fail flags
// force the robust scheduler down its failure path.
struct RecordingFilter {
    CallLog* log = nullptr;
    bool timeUpdatesFail = false;            // when set, every timeUpdate reports invalid
    bool failOnNegativeMeasurement = false;  // when set, a measurement with value < 0 reports invalid
    bool timeUpdate(double dt) {
        if (log) log->entries.push_back({CallLog::Kind::TimeUpdate, dt});
        return !timeUpdatesFail;
    }
    bool measurementUpdate(double const& m) {
        if (log) log->entries.push_back({CallLog::Kind::MeasurementUpdate, m});
        return !(failOnNegativeMeasurement && m < 0.0);
    }
    void clear() {
        if (log) log->entries.push_back({CallLog::Kind::Clear, 0.0});
    }
};

static_assert(SequentialFilter<RecordingFilter, double>);

// Count log entries of a given kind.
int count(CallLog const& log, CallLog::Kind kind) {
    int n = 0;
    for (auto const& [k, v] : log.entries) {
        if (k == kind) ++n;
    }
    return n;
}

}  // namespace

// Check the update of the time of previous measurement
TEST(ApplySequential, EmptyQueueAdvancesFilterButNotAnchor) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(3.0);

    applySequential(q, filter, 10.0);

    // Anchor stays at 3.0 — does NOT advance to callTime (10.0).
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 3.0);

    // Filter received one timeUpdate of (callTime - anchor) = 7.0.
    int timeCount = 0;
    double timeDt = 0;
    for (auto const& [k, v] : log.entries) {
        if (k == CallLog::Kind::TimeUpdate) {
            ++timeCount;
            timeDt = v;
        }
    }
    EXPECT_EQ(timeCount, 1);
    EXPECT_DOUBLE_EQ(timeDt, 7.0);
}

// Check the measurement update calls
TEST(ApplySequential, MeasurementsProcessesInAscendingTimeOrder) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(0.0);
    q.enqueue(5.0, 0.5);  // enqueued out of order
    q.enqueue(3.0, 0.3);

    applySequential(q, filter, 10.0);

    // Expected: timeUpdate(3), meas(0.3), timeUpdate(2), meas(0.5), timeUpdate(5)
    ASSERT_EQ(log.entries.size(), 5u);
    EXPECT_EQ(log.entries[0].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[0].second, 3.0);
    EXPECT_EQ(log.entries[1].first, CallLog::Kind::MeasurementUpdate);
    EXPECT_DOUBLE_EQ(log.entries[1].second, 0.3);
    EXPECT_EQ(log.entries[2].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[2].second, 2.0);
    EXPECT_EQ(log.entries[3].first, CallLog::Kind::MeasurementUpdate);
    EXPECT_DOUBLE_EQ(log.entries[3].second, 0.5);
    EXPECT_EQ(log.entries[4].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[4].second, 5.0);
}

// Enqueue measurements that are in the past (before a previous measurement time)
TEST(ApplySequential, SkipsMeasurementsBeforeStoredAnchor) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(10.0);
    q.enqueue(5.0, 1.0);   // stale: timeTag < anchor → skip
    q.enqueue(15.0, 2.0);  // fresh: timeTag > anchor → apply

    applySequential(q, filter, 20.0);

    int measCount = 0;
    double measValue = 0;
    for (auto const& [k, v] : log.entries) {
        if (k == CallLog::Kind::MeasurementUpdate) {
            ++measCount;
            measValue = v;
        }
    }
    EXPECT_EQ(measCount, 1);
    EXPECT_DOUBLE_EQ(measValue, 2.0);
}

// Filter updates the time of last measurement
TEST(ApplySequential, AdvancesAnchorToLatestMeasurementTime) {
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(0.0);
    q.enqueue(5.0, 1.0);
    q.enqueue(7.0, 2.0);

    RecordingFilter filter;
    applySequential(q, filter, 10.0);

    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 7.0);
}

// Check that the time update is called after the measurement update to move the state to current time
TEST(ApplySequential, NoFinalTimeUpdateWhenAnchorReachesCallTime) {
    {
        CallLog log;
        RecordingFilter filter{&log};
        measurement_queue<double, 4> q;
        q.setTimeOfLastMeasurement(10.0);
        applySequential(q, filter, 10.0);
        EXPECT_TRUE(log.entries.empty());
    }
    {
        CallLog log;
        RecordingFilter filter{&log};
        measurement_queue<double, 4> q;
        q.setTimeOfLastMeasurement(0.0);
        q.enqueue(10.0, 0.5);
        applySequential(q, filter, 10.0);
        // Expected: timeUpdate(10), meas(0.5) — NO trailing timeUpdate(0).
        ASSERT_EQ(log.entries.size(), 2u);
        EXPECT_EQ(log.entries[0].first, CallLog::Kind::TimeUpdate);
        EXPECT_EQ(log.entries[1].first, CallLog::Kind::MeasurementUpdate);
    }
}

// applySequentialRobust tests

// With all-valid updates the robust scheduler behaves like the basic one: the anchor
// advances to the latest measurement and clear() is never called.
TEST(ApplySequentialRobust, AllValidUpdatesAdvanceAnchorAndNeverClear) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(0.0);
    q.enqueue(5.0, 1.0);
    q.enqueue(7.0, 2.0);

    applySequentialRobust(q, filter, 10.0);

    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 7.0);
    EXPECT_EQ(count(log, CallLog::Kind::MeasurementUpdate), 2);
    EXPECT_EQ(count(log, CallLog::Kind::Clear), 0);
}

// A measurement whose update reports invalid triggers clear() and does NOT advance the
// anchor past it: the anchor stays at the last good measurement and the final propagation
// continues from there.
TEST(ApplySequentialRobust, FailedMeasurementClearsAndHoldsAnchor) {
    CallLog log;
    RecordingFilter filter{&log};
    filter.failOnNegativeMeasurement = true;
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(0.0);
    q.enqueue(2.0, 0.5);   // good
    q.enqueue(4.0, -1.0);  // bad: measurementUpdate reports invalid

    applySequentialRobust(q, filter, 10.0);

    // Full sequence: tU(2), meas(0.5), tU(2), meas(-1) [fails], clear, tU(8) from held anchor.
    ASSERT_EQ(log.entries.size(), 6u);
    EXPECT_EQ(log.entries[0].first, CallLog::Kind::TimeUpdate);
    EXPECT_EQ(log.entries[1].first, CallLog::Kind::MeasurementUpdate);
    EXPECT_DOUBLE_EQ(log.entries[1].second, 0.5);
    EXPECT_EQ(log.entries[2].first, CallLog::Kind::TimeUpdate);
    EXPECT_EQ(log.entries[3].first, CallLog::Kind::MeasurementUpdate);
    EXPECT_DOUBLE_EQ(log.entries[3].second, -1.0);
    EXPECT_EQ(log.entries[4].first, CallLog::Kind::Clear);
    EXPECT_EQ(log.entries[5].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[5].second, 8.0);  // callTime(10) - held anchor(2)

    // Anchor held at the last good measurement, not advanced to the failed one.
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 2.0);
}

// A failed timeUpdate short-circuits the measurement update (|| short-circuit), triggers
// clear(), and does not advance the anchor.
TEST(ApplySequentialRobust, FailedTimeUpdateShortCircuitsMeasurementAndClears) {
    CallLog log;
    RecordingFilter filter{&log};
    filter.timeUpdatesFail = true;
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(0.0);
    q.enqueue(2.0, 0.5);

    applySequentialRobust(q, filter, 10.0);

    // timeUpdate fails, so measurementUpdate is never reached; each failed timeUpdate
    // (one for the measurement, one final) is followed by clear(); the anchor never moves.
    EXPECT_EQ(count(log, CallLog::Kind::MeasurementUpdate), 0);
    EXPECT_EQ(count(log, CallLog::Kind::TimeUpdate), 2);
    EXPECT_EQ(count(log, CallLog::Kind::Clear), 2);
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 0.0);
}

// Processing-delay tests. Timeline: one call per second; at call t a fast sensor delivers a sample taken at t - 0.2 and
// a slow sensor a sample taken at t - 2.5. A slow sample is identified by a measurement value offset by kSlow.

namespace {

constexpr double kSlow = 1000.0;

// Enqueue the samples that arrive at callTime: fast at callTime - 0.2 and slow at callTime - 2.5.
template <std::size_t Capacity>
void enqueueArrivals(measurement_queue<double, Capacity>& q, double callTime) {
    q.enqueue(callTime - 0.2, callTime - 0.2);
    q.enqueue(callTime - 2.5, kSlow + callTime - 2.5);
}

// Measurement values applied, in call order.
std::vector<double> appliedMeasurements(CallLog const& log) {
    std::vector<double> values;
    for (auto const& [k, v] : log.entries) {
        if (k == CallLog::Kind::MeasurementUpdate) values.push_back(v);
    }
    return values;
}

}  // namespace

// Without a delay, the fast sensor moves the anchor ahead of every later slow sample, so the slow sensor is dropped
// from the second call on.
TEST(ApplySequentialDelay, WithoutDelayTheSlowSensorFallsBehindTheAnchor) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 8> q;
    q.setTimeOfLastMeasurement(0.0);
    for (double callTime = 10.0; callTime <= 15.0; callTime += 1.0) {
        enqueueArrivals(q, callTime);
        applySequential(q, filter, callTime);
    }

    int slowApplied = 0;
    for (double const v : appliedMeasurements(log)) {
        if (v >= kSlow) ++slowApplied;
    }
    EXPECT_EQ(slowApplied, 1) << "only the first slow sample, which arrives with the first fast one, is applied";
}

// With a 3 s delay, measurements are processed up to callTime - 3, so the slow samples (2.5 s late) are applied
// with the fast ones, in time order, and none is dropped.
TEST(ApplySequentialDelay, DelayAppliesTheSlowSensorInTimeOrder) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 8> q;
    q.setTimeOfLastMeasurement(0.0);
    double const delay = 3.0;
    q.setDelay(delay);
    for (double callTime = 10.0; callTime <= 15.0; callTime += 1.0) {
        enqueueArrivals(q, callTime);
        applySequential(q, filter, callTime);
        EXPECT_LE(q.getTimeOfLastMeasurement(), callTime - delay) << "the anchor stays at or behind callTime - delay";
    }

    // Every sample up to 15 - 3 = 12 is applied: slow 7.5 ... 11.5 and fast 9.8 ... 11.8, in time order.
    std::vector<double> const expected = {
        kSlow + 7.5, kSlow + 8.5, kSlow + 9.5, 10.0 - 0.2, kSlow + 10.5, 11.0 - 0.2, kSlow + 11.5, 12.0 - 0.2};
    EXPECT_EQ(appliedMeasurements(log), expected);
    EXPECT_FALSE(q.isEmpty()) << "the samples after 12 stay queued for a later call";
}

// The delayed measurements stay queued, and the filter is still propagated from the anchor to callTime.
TEST(ApplySequentialDelay, DelayedMeasurementsStayQueuedAndTheFilterReachesCallTime) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(5.0);
    q.setDelay(3.0);
    q.enqueue(9.8, 9.8);

    applySequential(q, filter, 10.0);

    ASSERT_EQ(log.entries.size(), 1u);
    EXPECT_EQ(log.entries[0].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[0].second, 5.0) << "propagated from the anchor (5) to callTime (10)";
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 5.0);
    auto const delayed = q.popEarliest();
    ASSERT_TRUE(delayed.has_value());
    EXPECT_DOUBLE_EQ(delayed->first, 9.8);
}

// A sample with a latency larger than the delay is behind the anchor: it is dropped.
TEST(ApplySequentialDelay, SampleWithALatencyAboveTheDelayIsDropped) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(10.0);
    q.setDelay(3.0);
    q.enqueue(6.5, kSlow + 6.5);  // latency of 7.5 s at call 14, more than the 3 s delay
    q.enqueue(10.5, 10.5);

    applySequential(q, filter, 14.0);

    EXPECT_EQ(appliedMeasurements(log), std::vector<double>{10.5});
    EXPECT_TRUE(q.isEmpty()) << "the late sample is not kept";
}

// The robust scheduler delays measurements the same way.
TEST(ApplySequentialDelay, RobustSchedulerDelaysTheSameWay) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 8> q;
    q.setTimeOfLastMeasurement(0.0);
    q.setDelay(3.0);
    for (double callTime = 10.0; callTime <= 15.0; callTime += 1.0) {
        enqueueArrivals(q, callTime);
        applySequentialRobust(q, filter, callTime);
    }

    std::vector<double> const expected = {
        kSlow + 7.5, kSlow + 8.5, kSlow + 9.5, 10.0 - 0.2, kSlow + 10.5, 11.0 - 0.2, kSlow + 11.5, 12.0 - 0.2};
    EXPECT_EQ(appliedMeasurements(log), expected);
    EXPECT_EQ(count(log, CallLog::Kind::Clear), 0);
}

// Time synchronization tests

// The first call on a new queue does nothing but set the anchor to callTime: no update and no measurement is popped.
TEST(ApplySequentialSynchronization, FirstCallOnlySetsTheAnchorToCallTime) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    EXPECT_FALSE(q.isTimeSynchronized());
    q.enqueue(95.0, 0.5);
    q.enqueue(100.0, 1.0);

    applySequential(q, filter, 100.0);

    EXPECT_TRUE(log.entries.empty());
    EXPECT_TRUE(q.isTimeSynchronized());
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 100.0);
    EXPECT_FALSE(q.isEmpty()) << "the measurements stay queued for the next call";
}

// After the synchronization, the next call propagates from the synchronized anchor and processes the measurements at or
// after it; the measurements before it are dropped.
TEST(ApplySequentialSynchronization, NextCallProcessesFromTheSynchronizedAnchor) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.enqueue(95.0, 0.5);
    q.enqueue(100.0, 1.0);
    applySequential(q, filter, 100.0);

    q.enqueue(100.5, 2.0);
    applySequential(q, filter, 101.0);

    // Expected: timeUpdate(0), meas(1.0), timeUpdate(0.5), meas(2.0), timeUpdate(0.5)
    ASSERT_EQ(log.entries.size(), 5u);
    EXPECT_DOUBLE_EQ(log.entries[0].second, 0.0);
    EXPECT_DOUBLE_EQ(log.entries[1].second, 1.0);
    EXPECT_DOUBLE_EQ(log.entries[2].second, 0.5);
    EXPECT_DOUBLE_EQ(log.entries[3].second, 2.0);
    EXPECT_EQ(log.entries[4].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[4].second, 0.5);
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 100.5);
}

// A first call well ahead of 0 moves the filter time to that call time: the next call propagates only from there.
TEST(ApplySequentialSynchronization, FirstCallFarAheadOfZeroMovesTheFilterTime) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 0.0);

    applySequentialRobust(q, filter, 1.0E5);
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 1.0E5) << "the filter time moved from 0 to the call time";
    EXPECT_TRUE(log.entries.empty());

    applySequentialRobust(q, filter, 1.0E5 + 2.0);
    ASSERT_EQ(log.entries.size(), 1u);
    EXPECT_EQ(log.entries[0].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[0].second, 2.0) << "propagated from the call time, not from 0";
}

// A first call at t = 0 also only synchronizes; the anchor then stays synchronized even though its value is still 0.
TEST(ApplySequentialSynchronization, FirstCallAtZeroSynchronizesOnce) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;

    applySequential(q, filter, 0.0);
    EXPECT_TRUE(log.entries.empty());

    q.enqueue(0.5, 1.0);
    applySequential(q, filter, 1.0);

    EXPECT_EQ(count(log, CallLog::Kind::MeasurementUpdate), 1);
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 0.5);
}

// clear() makes the next call synchronize again.
TEST(ApplySequentialSynchronization, ClearMakesTheNextCallSynchronizeAgain) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(10.0);
    q.clear();

    applySequential(q, filter, 50.0);

    EXPECT_TRUE(log.entries.empty());
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 50.0);
}

// The robust scheduler synchronizes the same way.
TEST(ApplySequentialSynchronization, RobustSchedulerSynchronizesTheSameWay) {
    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.enqueue(100.0, 1.0);

    applySequentialRobust(q, filter, 100.0);
    EXPECT_TRUE(log.entries.empty());
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 100.0);

    applySequentialRobust(q, filter, 101.0);
    EXPECT_EQ(count(log, CallLog::Kind::MeasurementUpdate), 1);
    EXPECT_EQ(count(log, CallLog::Kind::Clear), 0);
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 100.0);
}

}  // namespace filtering
