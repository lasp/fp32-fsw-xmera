// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for filtering::applySequential(): drains a measurement_queue
// in chronological order, interleaving timeUpdate / measurementUpdate calls,
// then a final timeUpdate to callTime.

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace filtering {
namespace {

// Call Log to check when we enter time and measurement updates
struct CallLog {
    enum class Kind { TimeUpdate, MeasurementUpdate };
    std::vector<std::pair<Kind, double>> entries;
};

// Filter which records the time/measruement update calls
struct RecordingFilter {
    CallLog* log = nullptr;
    void timeUpdate(double dt) {
        if (log) log->entries.push_back({CallLog::Kind::TimeUpdate, dt});
    }
    void measurementUpdate(double const& m) {
        if (log) log->entries.push_back({CallLog::Kind::MeasurementUpdate, m});
    }
};

static_assert(SequentialFilter<RecordingFilter, double>);

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

}  // namespace filtering
