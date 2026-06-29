// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_KALMAN_FILTER_HPP
#define FILTERING_CORE_KALMAN_FILTER_HPP

#include "measurementQueue.h"

namespace filtering {

/*! A sequential filter exposes the time-update and measurement-update functions.
 * At the concept level, they are not constrained to return anything specific. That is only driven by
 * the choice of applySquential. So when implementing a filter, the developer must ensure that their
 * implementations if time and measurement updates match the signature required by the applySequential logic.
 */
template <class Filter, class Measurement>
concept SequentialFilter = requires(Filter f, Measurement& m, double dt) {
    { f.timeUpdate(dt) };
    { f.measurementUpdate(m) };
};

/*! Basic sequential filter scheduling: empty the measurement queue in chronological order, calling
 *  timeUpdate from the last measurement-updated state to the measurement timeTag then measurementUpdate
 *  for each measurement; Call timeUpdate to `callTime` from there so the filter's `state`
 *  reflects the call time on return.
 *  If this function is called, the timeUpdate and measurementUpdates of the specific filter obeying the concept
 *  defined above must both return voids
 *  @param queue    [-] measurement queue (drained on return)
 *  @param filter   [-] filter satisfying SequentialFilter
 *  @param callTime [s] sim time the filter is advancing to */
template <class Filter, class Measurement, std::size_t Capacity>
    requires SequentialFilter<Filter, Measurement>
void applySequential(measurement_queue<Measurement, Capacity>& queue, Filter& filter, double callTime) {
    double timeOfLastMeasurement = queue.getTimeOfLastMeasurement();

    for (auto entry = queue.popEarliest(); entry.has_value(); entry = queue.popEarliest()) {
        auto& [timeTag, measurement] = entry.value();
        if (timeTag < timeOfLastMeasurement) continue;
        filter.timeUpdate(timeTag - timeOfLastMeasurement);
        filter.measurementUpdate(measurement);
        timeOfLastMeasurement = timeTag;
    }

    if (timeOfLastMeasurement < callTime) {
        filter.timeUpdate(callTime - timeOfLastMeasurement);
    }

    queue.setTimeOfLastMeasurement(timeOfLastMeasurement);
}

/*! Robust sequential filter scheduling: empty the measurement queue in chronological order, calling
 *  timeUpdate from the last measurement-updated state to the measurement timeTag then measurementUpdate
 *  for each measurement; Call timeUpdate to `callTime` from there so the filter's `state`
 *  reflects the call time on return.
 *  Because the timeUpdate and measurementUpdates now detect bad updates, they must now both return a bool.
 *  If the update was successful they should return true, and false otherwise. The filter must also have a
 *  clear method so that internal state can be sanitized after a bad update.
 *  @param queue    [-] measurement queue (drained on return)
 *  @param filter   [-] filter satisfying SequentialFilter
 *  @param callTime [s] sim time the filter is advancing to */
template <class Filter, class Measurement, std::size_t Capacity>
    requires SequentialFilter<Filter, Measurement>
void applySequentialRobust(measurement_queue<Measurement, Capacity>& queue, Filter& filter, double callTime) {
    double timeOfLastMeasurement = queue.getTimeOfLastMeasurement();

    for (auto entry = queue.popEarliest(); entry.has_value(); entry = queue.popEarliest()) {
        auto& [timeTag, measurement] = entry.value();
        if (timeTag < timeOfLastMeasurement) continue;

        if (!filter.timeUpdate(timeTag - timeOfLastMeasurement) || !filter.measurementUpdate(measurement)) {
            filter.clear();
        } else {
            timeOfLastMeasurement = timeTag;
        }
    }

    if (timeOfLastMeasurement < callTime) {
        if (!filter.timeUpdate(callTime - timeOfLastMeasurement)) {
            filter.clear();
        }
    }

    queue.setTimeOfLastMeasurement(timeOfLastMeasurement);
}

}  // namespace filtering

#endif
