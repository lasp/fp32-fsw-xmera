#ifndef AVERAGE_MIMU_DATA_ALGORITHM_H
#define AVERAGE_MIMU_DATA_ALGORITHM_H

#include "averageMimuDataTypes.h"
#include <utilities/fsw/freestandingInvalidArgument.h>
#include <utilities/fsw/timeConstants.h>
#include <utilities/fsw/validDcmCheck.h>

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
// MIMU device sample rate (compile-time fixed). Period in nanoseconds is
// precomputed so the per-sample staleness check stays in integer math.
constexpr double kMimuSampleRateHz = 100.0;
constexpr std::uint64_t kMimuSamplePeriodNs = static_cast<std::uint64_t>(kSec2Nano / kMimuSampleRateHz);

// Compile-time cap on the configured averaging window. Ring capacity is
// sized to hold exactly this many seconds of samples at the MIMU rate.
constexpr float kMaxAveragingWindowSec = 2.0F;

// Ceiling division so `rateHz * windowSec` samples round up to whole packets.
constexpr std::size_t ceilDivSamplesToPackets(double rateHz, float windowSec, std::size_t samplesPerPkt) {
    const double totalSamples = rateHz * windowSec;
    const std::size_t pkts = static_cast<std::size_t>(totalSamples) / samplesPerPkt;
    return (static_cast<double>(pkts * samplesPerPkt) < totalSamples) ? pkts + 1U : pkts;
}

constexpr std::size_t kRingCapacity =
    ceilDivSamplesToPackets(kMimuSampleRateHz, kMaxAveragingWindowSec, MAX_MIMU_SAMPLES_PER_PKT_C);
}  // namespace average_mimu_detail

/*! @brief Validated configuration for AverageMimuDataAlgorithm. Constructed via create(), which
 *         enforces the averaging-window bounds and DCM orthonormality before freezing the values. */
class AverageMimuDataConfig final {
   public:
    static AverageMimuDataConfig create(double gyroAveragingWindow,
                                        double accelAveragingWindow,
                                        const Eigen::Matrix3f& dcm_BP) {
        if (!isValidGyroAveragingWindow(gyroAveragingWindow)) {
            FSW_THROW_INVALID_ARGUMENT(
                "averageMimuData: gyroAveragingWindow must be in [0, kMaxAveragingWindowSec] seconds");
        }
        if (!isValidAccelAveragingWindow(accelAveragingWindow)) {
            FSW_THROW_INVALID_ARGUMENT(
                "averageMimuData: accelAveragingWindow must be in [0, kMaxAveragingWindowSec] seconds");
        }
        if (!isValidDcmPltfToBdy(dcm_BP)) {
            FSW_THROW_INVALID_ARGUMENT("averageMimuData: dcm_BP must be orthonormal with det=+1");
        }
        return {gyroAveragingWindow, accelAveragingWindow, dcm_BP};
    }

    static bool isValidGyroAveragingWindow(double window) {
        return window >= 0.0 && window <= average_mimu_detail::kMaxAveragingWindowSec;
    }
    static bool isValidAccelAveragingWindow(double window) {
        return window >= 0.0 && window <= average_mimu_detail::kMaxAveragingWindowSec;
    }
    static bool isValidDcmPltfToBdy(const Eigen::Matrix3f& dcm_BP) { return isValidDcm(dcm_BP); }

    double getGyroAveragingWindow() const { return this->gyroAveragingWindow; }
    double getAccelAveragingWindow() const { return this->accelAveragingWindow; }
    const Eigen::Matrix3f& getDcmPltfToBdy() const { return this->dcm_BP; }

   private:
    AverageMimuDataConfig(double gyroAveragingWindow, double accelAveragingWindow, const Eigen::Matrix3f& dcm_BP)
        : gyroAveragingWindow(gyroAveragingWindow), accelAveragingWindow(accelAveragingWindow), dcm_BP(dcm_BP) {}

    double gyroAveragingWindow;
    double accelAveragingWindow;
    Eigen::Matrix3f dcm_BP;
};

class AverageMimuDataAlgorithm final {
   public:
    static constexpr double kMimuSampleRateHz = average_mimu_detail::kMimuSampleRateHz;
    static constexpr std::uint64_t kMimuSamplePeriodNs = average_mimu_detail::kMimuSamplePeriodNs;
    static constexpr float kMaxAveragingWindowSec = average_mimu_detail::kMaxAveragingWindowSec;
    static constexpr std::size_t kRingCapacity = average_mimu_detail::kRingCapacity;

    explicit AverageMimuDataAlgorithm(const AverageMimuDataConfig& config);
    void setConfig(const AverageMimuDataConfig& config);  //!< Replace the configuration; runtime state is untouched
    void reInitialize();                                  //!< Clear the ring and new-packet tracking

    // Ingests new packets from the snapshot into the internal ring (strict
    // monotonic by per-packet representative time) and returns the rolling
    // average of fresh samples in the ring, gyro within gyroAveragingWindow and
    // accel within accelAveragingWindow of the newest stored sample.
    OutputAverageAccelAngleVel update(InputPktsData const& localPkts);

   private:
    // Ring slot mirrors the InputPacket shape: a packet's first-sample time
    // plus its samples. Per-sample times are derived at average compute time.
    struct RingPacket {
        bool isValid{false};
        std::uint64_t measTime{0U};
        std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT_C> samples{};
    };

    AverageMimuDataConfig cfg;
    // Config-derived: window seconds converted to nanoseconds once in setConfig()
    // so the per-sample staleness comparison in update() stays in integer math.
    std::uint64_t gyroAveragingWindowNs{0U};       //!< [ns] Gyro: allowable time difference from "latest"
    std::uint64_t accelAveragingWindowNs{0U};      //!< [ns] Accel: allowable time difference from "latest"
    std::array<RingPacket, kRingCapacity> ring{};  //!< Internal ring of recent packets (overwrites oldest on insert)
    std::size_t insertIdx{0U};                     //!< Next ring slot to overwrite
};

#endif
