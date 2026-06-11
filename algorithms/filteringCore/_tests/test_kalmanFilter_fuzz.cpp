// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz tests for filtering::applySequential.

#include <filteringCore/measurementQueue.h>
#include <filteringCore/kalmanFilter.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace filtering {
namespace {

struct CallLog {
    enum class Kind { TimeUpdate, MeasurementUpdate };
    std::vector<std::pair<Kind, double>> entries;
};

struct RecordingFilter {
    CallLog* log = nullptr;
    void timeUpdate(double dt) {
        if (log) log->entries.push_back({CallLog::Kind::TimeUpdate, dt});
    }
    void measurementUpdate(double const&) {
        if (log) log->entries.push_back({CallLog::Kind::MeasurementUpdate, 0.0});
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

}  // namespace filtering
