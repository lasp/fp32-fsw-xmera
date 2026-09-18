#include "regionsOfInterestPrune.h"

#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

/*! @brief Reset internal state, validate that rowColSumInMsg is connected, and construct the algorithm
 *         from the currently-set config properties.
 *  @param callTime Current simulation time in nanoseconds (unused).
 *  @throws std::invalid_argument If rowColSumInMsg is not connected, or if a config property is invalid.
 */
void RegionsOfInterestPrune::reset(uint64_t /*callTime*/) {
    if (!this->rowColSumInMsg.isLinked()) {
        throw std::invalid_argument("RegionsOfInterestPrune.rowColSumInMsg wasn't connected.");
    }
    this->algorithm = std::make_unique<RegionsOfInterestPruneAlgorithm>(this->toConfig());
    this->numPublished = 0;
    this->lastRegionsOutput = {};
}

/*! @brief Build a validated RegionsOfInterestPruneConfig from the adapter's stored properties. */
RegionsOfInterestPruneConfig RegionsOfInterestPrune::toConfig() const {
    return RegionsOfInterestPruneConfig::create(this->maxRowSpans, this->maxColSpans);
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
    const auto* rowSums = static_cast<const uint16_t*>(rcMsg.rowSumPointer);
    const auto* colSums = static_cast<const uint16_t*>(rcMsg.colSumPointer);

    const RoiCandidates candidates = this->algorithm->update(rowSums, rcMsg.numRows, colSums, rcMsg.numCols);

    this->lastRegionsOutput = {};
    this->numPublished = std::min(candidates.numCandidates, static_cast<uint32_t>(MAX_NUMBER_REGIONS));
    const double timeTagSec = static_cast<double>(callTime) * NANO2SEC;
    for (uint32_t k = 0; k < this->numPublished; ++k) {
        const auto& cand = candidates.candidates[k];
        this->lastRegionsOutput.timeTag[k] = timeTagSec;
        this->lastRegionsOutput.centerX[k] = static_cast<int>(cand.col + cand.width / 2);
        this->lastRegionsOutput.centerY[k] = static_cast<int>(cand.row + cand.height / 2);
        this->lastRegionsOutput.width[k] = static_cast<int>(cand.width);
        this->lastRegionsOutput.height[k] = static_cast<int>(cand.height);
        this->lastRegionsOutput.numberOfPixels[k] = static_cast<int>(cand.count);
    }
    this->regionsIdentifiedOutMsg.write(this->lastRegionsOutput, moduleID, callTime);
}
