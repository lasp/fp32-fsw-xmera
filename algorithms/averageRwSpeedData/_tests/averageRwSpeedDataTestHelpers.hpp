#ifndef TEST_AVERAGE_RW_SPEED_DATA_HELPERS_H
#define TEST_AVERAGE_RW_SPEED_DATA_HELPERS_H

#include "averageRwSpeedDataAlgorithm.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

/*! @brief Independent reimplementation of AverageRwSpeedDataAlgorithm::update() used by the regression and
 *  fuzz harnesses. Holds its own ring with the same capacity as the algorithm, so the outputs match exactly
 *  across cycles. */
class ReferenceAverager {
   public:
    explicit ReferenceAverager(AverageRwSpeedDataConfig const& cfg) : cfg(cfg) {}

    std::array<float, kMaxNumRw> update(RwSpeedSample const& sample) {
        constexpr auto kMaxWindowNs =
    static_cast<std::uint64_t>(AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec * 1.0e9);
        const auto windowNs = static_cast<std::uint64_t>(this->cfg.getRwSpeedAveragingWindow() * 1.0e9);

        std::uint64_t maxTimeTag = 0U;
        for (auto const& slot : this->ring) {
            if (slot.measTime > maxTimeTag) {
                maxTimeTag = slot.measTime;
            }
        }

        if (sample.measTime != 0U && maxTimeTag <= sample.measTime + kMaxWindowNs) {
            this->ring.at(this->insertIdx) = sample;
            this->insertIdx = (this->insertIdx + 1U) % AverageRwSpeedDataAlgorithm::kRingCapacity;
        }

        std::array<float, kMaxNumRw> sum{};
        std::uint64_t count = 0U;
        for (auto const& slot : this->ring) {
            if (slot.measTime != 0U && maxTimeTag <= slot.measTime + windowNs) {
                for (std::size_t w = 0; w < kMaxNumRw; ++w) {
                    sum.at(w) += slot.wheelSpeeds.at(w);
                }
                ++count;
            }
        }

        std::array<float, kMaxNumRw> out{};
        if (count > 0U) {
            for (std::size_t w = 0; w < kMaxNumRw; ++w) {
                out.at(w) = sum.at(w) / static_cast<float>(count);
            }
        }
        return out;
    }

   private:
    AverageRwSpeedDataConfig cfg;
    std::array<RwSpeedSample, AverageRwSpeedDataAlgorithm::kRingCapacity> ring{};
    std::size_t insertIdx{0U};
};

/*! @brief A sample whose wheel w has the speed base + w. */
inline RwSpeedSample makeSample(std::uint64_t measTime, float base) {
    RwSpeedSample sample{};
    sample.measTime = measTime;
    for (std::size_t w = 0; w < kMaxNumRw; ++w) {
        sample.wheelSpeeds.at(w) = base + static_cast<float>(w);
    }
    return sample;
}

/*! @brief The wheel speeds a sample made by makeSample(measTime, base) carries. */
inline std::array<float, kMaxNumRw> speedsFor(float base) { return makeSample(1U, base).wheelSpeeds; }

/*! @brief Drives the algorithm and the reference across a sequence of samples and compares every output. */
inline void sequencedRegressionTestAverageRwSpeedData(float window, std::vector<RwSpeedSample> const& samples) {
    const AverageRwSpeedDataConfig cfg = AverageRwSpeedDataConfig::create(window);
    AverageRwSpeedDataAlgorithm alg(cfg);
    ReferenceAverager ref(cfg);

    for (auto const& sample : samples) {
        EXPECT_EQ(alg.update(sample), ref.update(sample));
    }
}

inline void regressionTestAverageRwSpeedData(float window, RwSpeedSample const& sample) {
    sequencedRegressionTestAverageRwSpeedData(window, {sample});
}

#endif
