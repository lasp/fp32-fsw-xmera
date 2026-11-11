#include "regionsOfInterestPrune.h"

#include "utilities/fsw/timeConversion.hpp"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

/*! @brief Reset internal state, validate that rowColSumInMsg and binGridInMsg are connected, and
 *         construct the algorithm from the currently-set config properties.
 *  @param callTime Current simulation time in nanoseconds (unused).
 *  @throws std::invalid_argument If rowColSumInMsg or binGridInMsg is not connected, or if a config
 *          property is invalid.
 */
void RegionsOfInterestPrune::reset(uint64_t /*callTime*/) {
    if (!this->rowColSumInMsg.isLinked()) {
        throw std::invalid_argument("RegionsOfInterestPrune.rowColSumInMsg wasn't connected.");
    }
    if (!this->binGridInMsg.isLinked()) {
        throw std::invalid_argument("RegionsOfInterestPrune.binGridInMsg wasn't connected.");
    }
    this->algorithm = std::make_unique<RegionsOfInterestPruneAlgorithm>(this->toConfig());
}

/*! @brief Build a validated RegionsOfInterestPruneConfig from the adapter's stored properties. */
RegionsOfInterestPruneConfig RegionsOfInterestPrune::toConfig() const {
    return RegionsOfInterestPruneConfig::create(this->x0, this->y0, this->alphaAreaPruneScaling);
}

/*! @brief Push a fresh configuration into the algorithm without reconstructing it.
 *  @throws XmeraLifecycleException If reset() has not been called yet.
 */
void RegionsOfInterestPrune::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("RegionsOfInterestPrune reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void RegionsOfInterestPrune::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("RegionsOfInterestPrune reset() has not been called.");
    }

    const FpgaRowColSumMsgF32Payload rcMsg = this->rowColSumInMsg();
    // The algorithm takes its own grid type, not the payload: copy the lit counts out of the
    // {x, y, litPixelCount} regions into a contiguous array. numRegions is clamped to the array size;
    // the algorithm rejects a grid whose tile count does not match the image.
    const FpgaBinGridMsgF32Payload& gridMsg = this->binGridInMsg();
    const uint32_t numRegions = std::min(gridMsg.numRegions, static_cast<uint32_t>(FPGA_BIN_GRID_MAX_REGIONS));
    for (uint32_t i = 0; i < numRegions; ++i) {
        this->litCounts[i] = gridMsg.regions[i].litPixelCount;
    }
    const RoiBinGrid binGrid{.regionSize = gridMsg.regionSize,
                             .numRegionRows = gridMsg.numRegionRows,
                             .numRegionCols = gridMsg.numRegionCols,
                             .litCounts = std::span<const uint32_t>(this->litCounts.data(), numRegions)};

    const RoiCandidates candidates = this->algorithm->update(rcMsg.rowSums,
                                                             rcMsg.numRows,
                                                             rcMsg.colSums,
                                                             rcMsg.numCols,
                                                             rcMsg.rowIntensities,
                                                             rcMsg.colIntensities,
                                                             binGrid);

    RegionsIdentifiedMsgF32Payload output{};
    const uint32_t numPublished = std::min(candidates.numCandidates, static_cast<uint32_t>(MAX_NUMBER_REGIONS));
    const double timeTagSec = fsw::nanosToSeconds(callTime);
    output.numberOfRegions = static_cast<int>(numPublished);
    for (uint32_t k = 0; k < numPublished; ++k) {
        const auto& cand = candidates.candidates[k];
        output.timeTag[k] = timeTagSec;
        output.cornerX[k] = static_cast<int>(cand.col);
        output.cornerY[k] = static_cast<int>(cand.row);
        output.width[k] = static_cast<int>(cand.width);
        output.height[k] = static_cast<int>(cand.height);
        // count is saturated at UINT32_MAX; clamp so the int field cannot wrap negative.
        output.numberOfPixels[k] =
            static_cast<int>(std::min(cand.count, static_cast<uint32_t>(std::numeric_limits<int>::max())));
    }
    this->regionsIdentifiedOutMsg.write(output, moduleID, callTime);
}
