#ifndef TEST_AVERAGE_MIMU_DATA_HELPERS_H
#define TEST_AVERAGE_MIMU_DATA_HELPERS_H

#include "averageMimuDataAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>

#include <array>
#include <vector>

/*! @brief Independent reimplementation of AverageMimuDataAlgorithm's two-phase
 *  update used by the regression and fuzz harnesses. Holds its own ring with
 *  the same capacity as the algorithm so cross-cycle behavior matches
 *  bit-for-bit. The gyroAveragingWindow and dcm_BP are read from the algorithm
 *  via getters at each update() call. */
class ReferenceAverager {
   public:
    explicit ReferenceAverager(AverageMimuDataAlgorithm const& alg) : alg_(alg) {}

    OutputAverageAccelAngleVel update(InputPktsData const& localPkts) {
        // Phase 1: ingest. Freeze prior max for the duration of this call.
        const std::uint64_t priorMax = lastIngestedMaxMeasTime_;
        for (auto const& packet : localPkts.packets) {
            if (!packet.isValid) {
                continue;
            }
            const std::uint64_t firstSampleTime = packet.measTime;
            if ((firstSampleTime == 0U) || (firstSampleTime <= priorMax)) {
                continue;
            }

            ring_[insertIdx_].isValid = true;
            ring_[insertIdx_].measTime = firstSampleTime;
            ring_[insertIdx_].samples = packet.samples;
            insertIdx_ = (insertIdx_ + 1U) % AverageMimuDataAlgorithm::kRingCapacity;
            lastIngestedMaxMeasTime_ = std::max(lastIngestedMaxMeasTime_, firstSampleTime);
        }

        // Phase 2: max-tail-time + per-modality window filter, derived sample
        // schedule. Convert each window to ns once and compare in integer.
        const std::uint64_t gyroAveragingWindowNs =
            static_cast<std::uint64_t>(alg_.getGyroAveragingWindow() * 1.0e9);
        const std::uint64_t accelAveragingWindowNs =
            static_cast<std::uint64_t>(alg_.getAccelAveragingWindow() * 1.0e9);

        std::uint64_t maxSlotMeasTime = 0U;
        for (auto const& slot : ring_) {
            if (slot.isValid && (slot.measTime > maxSlotMeasTime)) {
                maxSlotMeasTime = slot.measTime;
            }
        }

        OutputAverageAccelAngleVel out{};
        if (maxSlotMeasTime == 0U) {
            return out;
        }

        const std::uint64_t maxTimeTag =
            maxSlotMeasTime
            + ((MAX_MIMU_SAMPLES_PER_PKT_C - 1U) * AverageMimuDataAlgorithm::kMimuSamplePeriodNs);

        Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
        Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
        std::uint64_t gyroAvgCount = 0U;
        std::uint64_t accelAvgCount = 0U;

        for (auto const& slot : ring_) {
            if (!slot.isValid) {
                continue;
            }
            for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT_C; ++s) {
                const std::uint64_t sampleMeasTime =
                    slot.measTime + (s * AverageMimuDataAlgorithm::kMimuSamplePeriodNs);
                const std::uint64_t age = maxTimeTag - sampleMeasTime;
                if (age <= gyroAveragingWindowNs) {
                    gyroSum_P += slot.samples[s].gyro_P;
                    gyroAvgCount++;
                }
                if (age <= accelAveragingWindowNs) {
                    accelSum_P += slot.samples[s].accel_P;
                    accelAvgCount++;
                }
            }
        }

        if (gyroAvgCount > 0U) {
            gyroSum_P /= static_cast<float>(gyroAvgCount);
            out.gyroOmega_B = alg_.getDcmPltfToBdy() * gyroSum_P;
        }
        if (accelAvgCount > 0U) {
            accelSum_P /= static_cast<float>(accelAvgCount);
            out.accel_B = alg_.getDcmPltfToBdy() * accelSum_P;
        }

        return out;
    }

   private:
    struct RingPacket {
        bool isValid{false};
        std::uint64_t measTime{0U};
        std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT_C> samples{};
    };

    AverageMimuDataAlgorithm const& alg_;
    std::array<RingPacket, AverageMimuDataAlgorithm::kRingCapacity> ring_{};
    std::size_t insertIdx_{0U};
    std::uint64_t lastIngestedMaxMeasTime_{0U};
};

/*! @brief Fill packet `p` of `in` with the given first-sample timestamp and
 *  per-sample gyro/accel pairs. Marks the packet valid. */
inline void fillPacket(InputPktsData& in,
                       std::size_t p,
                       std::uint64_t firstSampleTimeNs,
                       std::array<Eigen::Vector3f, MAX_MIMU_SAMPLES_PER_PKT_C> const& gyros,
                       std::array<Eigen::Vector3f, MAX_MIMU_SAMPLES_PER_PKT_C> const& accels) {
    in.packets[p].isValid = true;
    in.packets[p].measTime = firstSampleTimeNs;
    for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT_C; ++s) {
        in.packets[p].samples[s].gyro_P = gyros[s];
        in.packets[p].samples[s].accel_P = accels[s];
    }
}

inline void regressionTestAverageMimuData(float window, InputPktsData const& in) {
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(window);
    alg.setAccelAveragingWindow(window);

    ReferenceAverager ref(alg);

    const OutputAverageAccelAngleVel out_alg = alg.update(in);
    const OutputAverageAccelAngleVel out_ref = ref.update(in);

    EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B);
    EXPECT_EQ(out_alg.accel_B, out_ref.accel_B);
}

/*! @brief Drives the algorithm and the reference across a sequence of
 *  snapshots. Both maintain their own internal ring; the cycle-by-cycle
 *  outputs are compared to catch any drift in the staleness / window-filter
 *  logic across cycles or in the ingestion / overflow rules. */
inline void sequencedRegressionTestAverageMimuData(float window, std::vector<InputPktsData> const& frames) {
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(window);
    alg.setAccelAveragingWindow(window);

    ReferenceAverager ref(alg);

    for (auto const& in : frames) {
        const OutputAverageAccelAngleVel out_alg = alg.update(in);
        const OutputAverageAccelAngleVel out_ref = ref.update(in);

        EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B);
        EXPECT_EQ(out_alg.accel_B, out_ref.accel_B);
    }
}

#endif
