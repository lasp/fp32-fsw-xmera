#ifndef TEST_AVERAGE_RW_SPEED_DATA_HELPERS_H
#define TEST_AVERAGE_RW_SPEED_DATA_HELPERS_H

#include "averageRwSpeedDataAlgorithm.h"

#include <gtest/gtest.h>

#include <algorithm>
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

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

/*! @brief Every averaged wheel speed lies between the smallest and the largest speed that wheel
 *  received in an ingestible sample (nonzero measTime). The averaged samples are a subset of those, so a
 *  correct mean can never leave that range; an overflowing sum does. Until a sample is ingestible the output
 *  is zero. */
inline void propertyAverageWithinInputBounds(float window, std::vector<RwSpeedSample> const& samples) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(window));

    std::array<float, kMaxNumRw> minSpeed{};
    std::array<float, kMaxNumRw> maxSpeed{};
    bool anyIngestible = false;

    for (auto const& sample : samples) {
        if (sample.measTime != 0U) {
            for (std::size_t w = 0; w < kMaxNumRw; ++w) {
                const float speed = sample.wheelSpeeds.at(w);
                minSpeed.at(w) = anyIngestible ? std::min(minSpeed.at(w), speed) : speed;
                maxSpeed.at(w) = anyIngestible ? std::max(maxSpeed.at(w), speed) : speed;
            }
            anyIngestible = true;
        }

        const auto out = alg.update(sample);
        for (std::size_t w = 0; w < kMaxNumRw; ++w) {
            EXPECT_GE(out.at(w), minSpeed.at(w)) << "wheel " << w;
            EXPECT_LE(out.at(w), maxSpeed.at(w)) << "wheel " << w;
        }
    }
}

#endif
