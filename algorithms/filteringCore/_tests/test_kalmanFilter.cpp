// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for filtering::applySequential() and applySequentialRobust(): both drain a
// measurement_queue in chronological order, interleaving timeUpdate / measurementUpdate
// calls then a final timeUpdate to callTime. applySequentialRobust additionally reads each
// update's bool result and, on a failed update, calls clear() and holds the anchor.

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
    q.enqueue(2.0, 0.5);

    applySequentialRobust(q, filter, 10.0);

    // timeUpdate fails, so measurementUpdate is never reached; each failed timeUpdate
    // (one for the measurement, one final) is followed by clear(); the anchor never moves.
    EXPECT_EQ(count(log, CallLog::Kind::MeasurementUpdate), 0);
    EXPECT_EQ(count(log, CallLog::Kind::TimeUpdate), 2);
    EXPECT_EQ(count(log, CallLog::Kind::Clear), 2);
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 0.0);
}

}  // namespace filtering
