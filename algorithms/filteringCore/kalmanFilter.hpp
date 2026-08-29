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

/*! Robust filter update with no absolute time: the filter steps by `dt` every call and every queued
 *  measurement is folded in as if taken at the end of that time update. There is no callTime; instead the
 *  queue's `timeOfLastMeasurement` is repurposed as the elapsed span since the anchor (the last folded-in
 *  measurement). Each call propagates over `step = getTimeOfLastMeasurement() + dt` from the SRuKF-style
 *  anchor, so measurement-free (or failed) cycles keep accumulating and the state marches forward
 *  cumulatively; the accumulator resets to zero only when a measurement advances the anchor.
 *  Because timeUpdate and measurementUpdate detect bad updates, they must both return a bool (true on a
 *  good update). The filter must also expose a clear() so internal state can be sanitized after a bad update.
 *  The first queued measurement folds into the propagated sigma points from timeUpdate(step); each
 *  subsequent measurement is preceded by a timeUpdate(0) to re-anchor the sigma points about the latest
 *  posterior before it is folded in.
 *  @param queue    [-] measurement queue (drained on return)
 *  @param filter   [-] filter satisfying SequentialFilter
 *  @param dt       [s] time step of the filter update
 */
template <class Filter, class Measurement, std::size_t Capacity>
    requires SequentialFilter<Filter, Measurement>
void applyTimestepRobust(measurement_queue<Measurement, Capacity>& queue, Filter& filter, double dt) {
    if (double const step = queue.getTimeOfLastMeasurement() + dt; !filter.timeUpdate(step)) {
        // Propagation failed: sanitize and carry the elapsed span forward so the retry propagates the full
        // interval next call. Do not process measurements against a bad prediction.
        filter.clear();
        queue.setTimeOfLastMeasurement(step);
    } else {
        bool measurementApplied = false;
        for (auto entry = queue.popEarliest(); entry.has_value(); entry = queue.popEarliest()) {
            if (measurementApplied) {
                filter.timeUpdate(0);  // re-anchor sigma points about the latest posterior (2nd+ measurement only)
            }
            auto& [_, measurement] = entry.value();
            if (!filter.measurementUpdate(measurement)) {
                filter.clear();
            } else {
                measurementApplied = true;
            }
        }
        // A measurement advances the stateOfLastMeasurement, otherwise keep accumulating the
        // elapsed span so a measurement-free step still propagates cumulatively from the fixed anchor.
        queue.setTimeOfLastMeasurement(measurementApplied ? 0.0 : step);
    }
}

}  // namespace filtering

#endif
