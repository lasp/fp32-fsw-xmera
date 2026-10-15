// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_MEASUREMENT_QUEUE_HPP
#define FILTERING_CORE_MEASUREMENT_QUEUE_HPP

#include <array>
#include <optional>
#include <utility>

namespace filtering {

/*! Measurement container.  */
template <typename Measurement, std::size_t CAPACITY>
class measurement_queue final {
   public:
    bool isEmpty() const { return this->size == 0; }
    bool isFull() const { return this->size >= CAPACITY; }

    /*! Set the time of last measurement.
     *  @param t [s] sim time of the last drained measurement */
    void setTimeOfLastMeasurement(double t) { this->lastMeasurementTime = t; }
    /*! @return time of the most recent measurement processed */
    double getTimeOfLastMeasurement() const { return this->lastMeasurementTime; }

    /*! Set the processing delay: a measurement is processed only once the call time is at least its time tag plus
     *  the delay, so a measurement with a latency up to the delay is still processed in time order. Zero (the
     *  default) processes every queued measurement. This is configuration: clear() keeps it.
     *  @param delay [s] processing delay, >= 0 */
    void setDelay(double delay) { this->processingDelay = delay; }
    /*! @return [s] processing delay */
    double getDelay() const { return this->processingDelay; }

    /*! Insertion-sort a measurement into the queue by descending timeTag.
     *  @return true if enqueued, false if the queue was full
     *  @param timeTag     [s] time the measurement was taken
     *  @param measurement [-] measurement payload (moved in) */
    bool enqueue(double timeTag, Measurement&& measurement) {
        if (this->isFull()) return false;

        std::size_t insertionIndex = 0;
        for (insertionIndex = this->size; insertionIndex > 0; --insertionIndex) {
            if (timeTag <= this->measurements[insertionIndex - 1].value().first) break;
            this->measurements[insertionIndex] = std::move(this->measurements[insertionIndex - 1]);
        }

        this->measurements[insertionIndex] = {timeTag, std::move(measurement)};
        this->size += 1;
        return true;
    }

    /*! Drop all measurements and reset the time anchor to zero. The processing delay is kept. */
    void clear() {
        while (this->size > 0) {
            this->size -= 1;
            this->measurements[this->size] = std::nullopt;
        }
        this->lastMeasurementTime = 0;
    }

    /*! Remove and return the earliest queued measurement.
     *  @return std::optional<{timeTag, measurement}>; nullopt if empty */
    std::optional<std::pair<double, Measurement>> popEarliest() {
        if (this->isEmpty()) return std::nullopt;

        this->size -= 1;
        return std::exchange(this->measurements[this->size], std::nullopt);
    }

    /*! Remove and return the earliest queued measurement if it is due at callTime: with a zero delay any queued
     *  measurement is due, otherwise one with a time tag at or before callTime - delay.
     *  @return std::optional<{timeTag, measurement}>; nullopt if empty or the earliest is not due yet
     *  @param callTime [s] sim time the filter is advancing to */
    std::optional<std::pair<double, Measurement>> popEarliestDue(double callTime) {
        if (this->isEmpty()) return std::nullopt;
        if (this->processingDelay > 0.0 &&
            this->measurements[this->size - 1].value().first > callTime - this->processingDelay) {
            return std::nullopt;
        }
        return this->popEarliest();
    }

   private:
    std::size_t size = 0;
    std::array<std::optional<std::pair<double, Measurement>>, CAPACITY> measurements = {};
    double lastMeasurementTime = 0;
    double processingDelay = 0;
};

}  // namespace filtering

#endif
