#ifndef AVERAGE_MIMU_DATA_ALGORITHM_H
#define AVERAGE_MIMU_DATA_ALGORITHM_H

#include "averageMimuDataTypes.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>

/*! @brief One MIMU sample at the algorithm-internal layer: a gyro/accel
 *         pair in the platform frame. Per-sample timestamps are derived
 *         from the enclosing packet's `measTime` plus the device sample
 *         period (kMimuSamplePeriodNs); the sample itself stores no time. */
struct Sample {
    Eigen::Vector3f gyro_P{Eigen::Vector3f::Zero()};
    Eigen::Vector3f accel_P{Eigen::Vector3f::Zero()};
};

/*! @brief Algorithm-internal view of one MIMU packet. `measTime` is the
 *         first sample's measurement time; the rest of the samples follow
 *         at multiples of kMimuSamplePeriodNs. `isValid` gates the whole
 *         packet; when true, every sample in `samples` is assumed real. */
struct InputPacket {
    bool isValid{false};
    std::uint64_t measTime{0U};
    std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT_C> samples{};
};

/*! @brief Algorithm-internal view of a MimuPacketF32Payload: 4 packets,
 *         each holding MAX_MIMU_SAMPLES_PER_PKT_C samples. */
struct InputPktsData {
    std::array<InputPacket, MAX_MIMU_PKT_C> packets{};
};


/*! @brief Structure containing the OutputAverageAccelAngleVel*/
struct OutputAverageAccelAngleVel {
    Eigen::Vector3f accel_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyroOmega_B = Eigen::Vector3f::Zero();
};

namespace average_mimu_detail {
// Ceiling division so `rateHz * windowSec` samples round up to whole packets.
constexpr std::size_t ceilDivSamplesToPackets(float rateHz, float windowSec, std::size_t samplesPerPkt) {
    const float totalSamples = rateHz * windowSec;
    const std::size_t pkts = static_cast<std::size_t>(totalSamples) / samplesPerPkt;
    return (static_cast<float>(pkts * samplesPerPkt) < totalSamples) ? pkts + 1U : pkts;
}
}  // namespace average_mimu_detail

class AverageMimuDataAlgorithm {
   public:
    // MIMU device sample rate (compile-time fixed). Period in nanoseconds
    // is precomputed so the per-sample staleness check stays in integer math.
    static constexpr float         kMimuSampleRateHz     = 100.0F;
    static constexpr std::uint64_t kMimuSamplePeriodNs   = 10'000'000U;  // 1e9 / 100

    // Compile-time cap on the configured averaging window. Ring capacity is
    // sized to hold exactly this many seconds of samples at the MIMU rate.
    static constexpr float kMaxAveragingWindowSec = 2.0F;

    static constexpr std::size_t kRingCapacity =
        average_mimu_detail::ceilDivSamplesToPackets(
            kMimuSampleRateHz, kMaxAveragingWindowSec, MAX_MIMU_SAMPLES_PER_PKT_C);

    void setGyroAveragingWindow(double window);                 //!< [s] Setter method for windowSec
    double getGyroAveragingWindow() const;                      //!< [s] Getter method for windowSec
    void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn);  //!< Setter method for dcm from platform to body
    Eigen::Matrix3f getDcmPltfToBdy() const;                //!< Getter method for dcm from platform to body

    // Ingests new packets from the snapshot into the internal ring (strict
    // monotonic by per-packet representative time) and returns the rolling
    // average over fresh samples in the ring within averagingWindow of the
    // newest stored sample.
    OutputAverageAccelAngleVel update(InputPktsData const& localPkts);

   private:
    // Ring slot mirrors the InputPacket shape: a packet's first-sample time
    // plus its samples. Per-sample times are derived at average compute time.
    struct RingPacket {
        bool isValid{false};
        std::uint64_t measTime{0U};
        std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT_C> samples{};
    };

    // Stored as nanoseconds so the per-sample comparison in update() is a
    // pure uint64_t compare. Float is only used at the public seconds-based API.
    std::uint64_t gyroAveragingWindowNs{0U};                   //!< [ns] Allowable time difference from "latest"
    Eigen::Matrix3f dcm_BP = Eigen::Matrix3f::Identity();  //!< [-] Transformation from the platform frame to body
    std::array<RingPacket, kRingCapacity> ring{};          //!< Internal ring of recent packets (overwrites oldest on insert)
    std::size_t insertIdx{0U};                             //!< Next ring slot to overwrite
    std::uint64_t lastIngestedMaxMeasTime{0U};             //!< Newest representative time ever ingested (for new-packet detection)
};

#endif
