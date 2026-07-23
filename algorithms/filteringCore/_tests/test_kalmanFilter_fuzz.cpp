// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz tests for filtering::applySequential, applySequentialRobust, and
// applyTimestepRobust.

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <vector>

namespace filtering {
namespace {

struct CallLog {
    enum class Kind { TimeUpdate, MeasurementUpdate, Clear };
    std::vector<std::pair<Kind, double>> entries;
};

// Records time/measurement update and clear() calls. measurementUpdate logs the measurement
// value; when failOnNegativeMeasurement is set, a value < 0 reports an invalid update, which
// lets the sign of a fuzzed value drive the robust scheduler down its failure path.
struct RecordingFilter {
    CallLog* log = nullptr;
    bool failOnNegativeMeasurement = false;
    bool timeUpdate(double dt) {
        if (log) log->entries.push_back({CallLog::Kind::TimeUpdate, dt});
        return true;
    }
    bool measurementUpdate(double const& m) {
        if (log) log->entries.push_back({CallLog::Kind::MeasurementUpdate, m});
        return !(failOnNegativeMeasurement && m < 0.0);
    }
    void clear() {
        if (log) log->entries.push_back({CallLog::Kind::Clear, 0.0});
    }
};

}  // namespace

// For any 3 measurements, initial anchor, and callTime (>= anchor),
// the call sequence alternates timeUpdate and measurementUpdate,
// it has only postivie dt's,
// the total dt sum equals the total advancement.
void fuzzApplySequentialInvariants(double t0, double t1, double t2, double initialAnchor, double extraCallTime) {
    double const callTime = initialAnchor + extraCallTime;

    CallLog log;
    RecordingFilter filter{&log};
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(initialAnchor);
    q.enqueue(t0, 0.0);
    q.enqueue(t1, 0.0);
    q.enqueue(t2, 0.0);

    applySequential(q, filter, callTime);

    double dtSum = 0;
    int timeCount = 0, measCount = 0;
    bool expectTimeNext = true;
    for (auto const& [k, v] : log.entries) {
        if (k == CallLog::Kind::TimeUpdate) {
            EXPECT_TRUE(expectTimeNext) << "two timeUpdates in a row";
            EXPECT_GE(v, 0.0) << "negative dt";
            dtSum += v;
            ++timeCount;
            expectTimeNext = false;
        } else {
            EXPECT_FALSE(expectTimeNext) << "measurementUpdate without preceding timeUpdate";
            ++measCount;
            expectTimeNext = true;
        }
    }
    EXPECT_TRUE(timeCount == measCount || timeCount == measCount + 1)
        << "timeCount/measCount mismatch beyond the trailing timeUpdate";

    double const finalAnchor = q.getTimeOfLastMeasurement();
    double const target = std::max(finalAnchor, callTime);
    EXPECT_NEAR(dtSum, target - initialAnchor, 1e-9);
}
FUZZ_TEST(ApplySequentialFuzz, fuzzApplySequentialInvariants)
    .WithDomains(fuzztest::InRange(-1e4, 1e4),
                 fuzztest::InRange(-1e4, 1e4),
                 fuzztest::InRange(-1e4, 1e4),
                 fuzztest::InRange(0.0, 1e4),
                 fuzztest::InRange(0.0, 1e4));

// Robust scheduling under a fuzzed failure pattern: the sign of each measurement value
// decides whether its update succeeds (>= 0) or fails (< 0). Regardless of the pattern:
//   * every timeUpdate dt is non-negative (a held anchor never steps backwards),
//   * clear() fires exactly once per failed measurement, immediately after it, and
//   * the anchor ends at the last successfully-processed measurement time.
void fuzzApplySequentialRobustInvariants(double t0,
                                         double t1,
                                         double t2,
                                         double v0,
                                         double v1,
                                         double v2,
                                         double initialAnchor,
                                         double extraCallTime) {
    double const callTime = initialAnchor + extraCallTime;

    CallLog log;
    RecordingFilter filter{&log};
    filter.failOnNegativeMeasurement = true;
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(initialAnchor);
    q.enqueue(t0, double{v0});
    q.enqueue(t1, double{v1});
    q.enqueue(t2, double{v2});

    applySequentialRobust(q, filter, callTime);

    // (1) non-negative dt everywhere; (2) each clear immediately follows a failed
    //     (value < 0) measurement update, and clears count equals failed measurements.
    int clears = 0;
    int failedMeasurements = 0;
    for (std::size_t i = 0; i < log.entries.size(); ++i) {
        auto const& [kind, value] = log.entries[i];
        if (kind == CallLog::Kind::TimeUpdate) {
            EXPECT_GE(value, 0.0) << "negative dt";
        } else if (kind == CallLog::Kind::MeasurementUpdate) {
            if (value < 0.0) ++failedMeasurements;
        } else {  // Clear
            ++clears;
            ASSERT_GT(i, 0u);
            EXPECT_EQ(log.entries[i - 1].first, CallLog::Kind::MeasurementUpdate)
                << "clear must follow a measurement update";
            EXPECT_LT(log.entries[i - 1].second, 0.0) << "clear must follow a failed (value < 0) measurement";
        }
    }
    EXPECT_EQ(clears, failedMeasurements) << "exactly one clear per failed measurement";

    // (3) reference model: the anchor advances only through successfully-processed
    //     measurements (timeUpdate always succeeds here; a measurement succeeds iff v >= 0).
    std::array<std::pair<double, double>, 3> measurements{{{t0, v0}, {t1, v1}, {t2, v2}}};
    std::sort(measurements.begin(), measurements.end(), [](auto const& a, auto const& b) { return a.first < b.first; });
    double cursor = initialAnchor;
    for (auto const& [timeTag, value] : measurements) {
        if (timeTag < cursor) continue;  // stale: before the running anchor
        if (value >= 0.0) cursor = timeTag;
    }
    EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), cursor)
        << "anchor must equal the last successfully-processed measurement time";
}
FUZZ_TEST(ApplySequentialFuzz, fuzzApplySequentialRobustInvariants)
    .WithDomains(fuzztest::InRange(-1e4, 1e4),  // t0
                 fuzztest::InRange(-1e4, 1e4),  // t1
                 fuzztest::InRange(-1e4, 1e4),  // t2
                 fuzztest::InRange(-1e4, 1e4),  // v0 (sign selects success/failure)
                 fuzztest::InRange(-1e4, 1e4),  // v1
                 fuzztest::InRange(-1e4, 1e4),  // v2
                 fuzztest::InRange(0.0, 1e4),   // initialAnchor
                 fuzztest::InRange(0.0, 1e4));  // extraCallTime

// applyTimestepRobust under a fuzzed measurement failure pattern (sign of each value selects
// success/failure; timeUpdate always succeeds here). Regardless of the pattern:
//   * the first call is a single timeUpdate over the full elapsed span (initialSpan + dt),
//   * every other timeUpdate is a zero-dt re-anchor,
//   * clear() fires exactly once per failed measurement, immediately after it, and
//   * the elapsed span resets to 0 iff at least one measurement folded in, else keeps initialSpan + dt.
void fuzzApplyTimestepRobustInvariants(double t0,
                                       double t1,
                                       double t2,
                                       double v0,
                                       double v1,
                                       double v2,
                                       double initialSpan,
                                       double dt) {
    CallLog log;
    RecordingFilter filter{&log};
    filter.failOnNegativeMeasurement = true;
    measurement_queue<double, 4> q;
    q.setTimeOfLastMeasurement(initialSpan);
    q.enqueue(t0, double{v0});
    q.enqueue(t1, double{v1});
    q.enqueue(t2, double{v2});

    double const step = initialSpan + dt;
    applyTimestepRobust(q, filter, dt);

    ASSERT_FALSE(log.entries.empty());
    EXPECT_EQ(log.entries[0].first, CallLog::Kind::TimeUpdate);
    EXPECT_DOUBLE_EQ(log.entries[0].second, step) << "first timeUpdate must span the full elapsed interval";

    int measApplied = 0;
    int clears = 0;
    int failed = 0;
    for (std::size_t i = 0; i < log.entries.size(); ++i) {
        auto const& [kind, value] = log.entries[i];
        if (kind == CallLog::Kind::TimeUpdate) {
            EXPECT_GE(value, 0.0) << "negative dt";
            if (i > 0) EXPECT_DOUBLE_EQ(value, 0.0) << "in-loop re-anchor must be a zero time update";
        } else if (kind == CallLog::Kind::MeasurementUpdate) {
            if (value >= 0.0) {
                ++measApplied;
            } else {
                ++failed;
            }
        } else {  // Clear
            ++clears;
            ASSERT_GT(i, 0u);
            EXPECT_EQ(log.entries[i - 1].first, CallLog::Kind::MeasurementUpdate)
                << "clear must follow a measurement update";
            EXPECT_LT(log.entries[i - 1].second, 0.0) << "clear must follow a failed (value < 0) measurement";
        }
    }
    EXPECT_EQ(clears, failed) << "exactly one clear per failed measurement";

    if (measApplied > 0) {
        EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), 0.0) << "a folded-in measurement resets the span";
    } else {
        EXPECT_DOUBLE_EQ(q.getTimeOfLastMeasurement(), step) << "with nothing folded in the span is carried forward";
    }
}
FUZZ_TEST(ApplySequentialFuzz, fuzzApplyTimestepRobustInvariants)
    .WithDomains(fuzztest::InRange(-1e4, 1e4),  // t0
                 fuzztest::InRange(-1e4, 1e4),  // t1
                 fuzztest::InRange(-1e4, 1e4),  // t2
                 fuzztest::InRange(-1e4, 1e4),  // v0 (sign selects success/failure)
                 fuzztest::InRange(-1e4, 1e4),  // v1
                 fuzztest::InRange(-1e4, 1e4),  // v2
                 fuzztest::InRange(0.0, 1e4),   // initialSpan
                 fuzztest::InRange(0.0, 1e4));  // dt

}  // namespace filtering
