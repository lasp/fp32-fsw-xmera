#include "regionsOfInterestPruneAlgorithm_c.h"
#include "regionsOfInterestPruneAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

static_assert(REGIONS_OF_INTEREST_PRUNE_ROI_CANDIDATES_MAX == ROI_CANDIDATES_MAX,
              "C-shim candidates count must match the algorithm's ROI_CANDIDATES_MAX");

namespace {
RegionsOfInterestPruneConfig configFromC(const RegionsOfInterestPruneConfig_c& c) {
    return RegionsOfInterestPruneConfig::create(c.maxRowSpans, c.maxColSpans);
}

RoiCandidates_c outputToC(const RoiCandidates& out) {
    RoiCandidates_c result{};
    result.numCandidates = out.numCandidates;
    for (uint32_t i = 0U; i < out.numCandidates; ++i) {
        result.candidates[i].row = out.candidates[i].row;
        result.candidates[i].col = out.candidates[i].col;
        result.candidates[i].height = out.candidates[i].height;
        result.candidates[i].width = out.candidates[i].width;
        result.candidates[i].count = out.candidates[i].count;
    }
    return result;
}
}  // namespace

uint32_t RegionsOfInterestPruneAlgorithm_getMaxCandidatesCount(void) { return ROI_CANDIDATES_MAX; }

RegionsOfInterestPruneAlgorithmHandle* RegionsOfInterestPruneAlgorithm_create(
    const RegionsOfInterestPruneConfig_c* config) {
    return fsw::createHandle<::RegionsOfInterestPruneAlgorithm, RegionsOfInterestPruneAlgorithmHandle>(
        configFromC(*config));
}

void RegionsOfInterestPruneAlgorithm_destroy(RegionsOfInterestPruneAlgorithmHandle* self) {
    fsw::deleteHandle<::RegionsOfInterestPruneAlgorithm>(self);
}

void RegionsOfInterestPruneAlgorithm_setConfig(RegionsOfInterestPruneAlgorithmHandle* self,
                                               const RegionsOfInterestPruneConfig_c* config) {
    fsw::fromHandle<::RegionsOfInterestPruneAlgorithm>(self)->setConfig(configFromC(*config));
}

RoiCandidates_c RegionsOfInterestPruneAlgorithm_update(const RegionsOfInterestPruneAlgorithmHandle* self,
                                                       const uint16_t* rowSums,
                                                       uint32_t numRows,
                                                       const uint16_t* colSums,
                                                       uint32_t numCols) {
    const RoiCandidates out =
        fsw::fromHandle<const ::RegionsOfInterestPruneAlgorithm>(self)->update(rowSums, numRows, colSums, numCols);
    return outputToC(out);
}
