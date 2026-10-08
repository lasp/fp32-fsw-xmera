#ifndef AVERAGE_RW_SPEED_DATA_ALGORITHM_H
#define AVERAGE_RW_SPEED_DATA_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include <utilities/fsw/freestandingInvalidArgument.h>
#include <utilities/fsw/timeConstants.h>
#include <utilities/fsw/validDcmCheck.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>

/*! @brief Structure containing RW speed data and its measured time. */
struct RwSpeedSample {
    std::uint64_t measTime{0U};
    std::array<float, kMaxNumRw> wheelSpeeds{};
};

namespace average_rw_speed_detail {
// RW speed sample rate (compile-time fixed). Period in nanoseconds is
// precomputed so the per-sample staleness check stays in integer math.
constexpr double kRwSpeedSampleRateHz = 5.0;

// Compile-time cap on the configured averaging window. The ring holds every
// sample of a window this long at the RW speed rate.
constexpr float kMaxAveragingWindowSec = 2.0F;

// Number of samples a window of windowSec spans at rateHz. The product is rounded up, and one
// sample is added because the window includes both of its ends.
constexpr std::size_t ringCapacityFor(double rateHz, float windowSec) {
    const double samples = rateHz * static_cast<double>(windowSec);
    const auto wholeSamples = static_cast<std::size_t>(samples);
    const std::size_t roundedUp = (static_cast<double>(wholeSamples) < samples) ? wholeSamples + 1U : wholeSamples;
    return roundedUp + 1U;
}

constexpr std::size_t kRingCapacity = ringCapacityFor(kRwSpeedSampleRateHz, kMaxAveragingWindowSec);
}  // namespace average_rw_speed_detail

/*! @brief Validated configuration for AverageRwSpeedDataAlgorithm. Constructed via create(), which
 *         enforces the averaging-window bounds before freezing the values. */
class AverageRwSpeedDataConfig final {
   public:
    static AverageRwSpeedDataConfig create(float rwSpeedAveragingWindow) {
        if (!isValidRwSpeedAveragingWindow(rwSpeedAveragingWindow)) {
            FSW_THROW_INVALID_ARGUMENT(
                "averageRwSpeedData: rwSpeedAveragingWindow must be in (0, kMaxAveragingWindowSec] seconds");
        }
        return {rwSpeedAveragingWindow};
    }

    static bool isValidRwSpeedAveragingWindow(const float window) {
        return window > 0.0F && window <= average_rw_speed_detail::kMaxAveragingWindowSec;
    }

    float getRwSpeedAveragingWindow() const { return this->rwSpeedAveragingWindow; }

   private:
    AverageRwSpeedDataConfig(const float rwSpeedAveragingWindow) : rwSpeedAveragingWindow(rwSpeedAveragingWindow) {}

    float rwSpeedAveragingWindow = 0.F;
};

class AverageRwSpeedDataAlgorithm final {
   public:
    static constexpr float kMaxAveragingWindowSec = average_rw_speed_detail::kMaxAveragingWindowSec;
    static constexpr std::size_t kRingCapacity = average_rw_speed_detail::kRingCapacity;

    explicit AverageRwSpeedDataAlgorithm(const AverageRwSpeedDataConfig& config);
    void setConfig(const AverageRwSpeedDataConfig& config);  //!< Replace the configuration; runtime state is untouched
    void reInitialize();                                     //!< Clear the ring and new-packet tracking

    // Ingests the new, instantaneous RW speed values into the internal ring.
    // Returns the rolling average of fresh samples in the ring rwSpeedAveragingWindow of the newest stored sample.
    std::array<float, kMaxNumRw> update(RwSpeedSample const& wheelSpeedData);

   private:
    AverageRwSpeedDataConfig cfg;
    // Config-derived: window seconds converted to nanoseconds once in setConfig()
    // so the per-sample staleness comparison in update() stays in integer math.
    std::uint64_t rwSpeedAveragingWindowNs{0U};  //!< [ns] RW speed: allowable time difference from "latest"
    // Internal states: ring containing all the RW speed samples within the time window
    // and the next index to insert a sample into
    std::array<RwSpeedSample, kRingCapacity> ring{};  //!< Internal ring of recent samples
    std::size_t insertIdx{0U};                        //!< Next ring slot to overwrite
};

#endif
