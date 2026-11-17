#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_H

#include "msgPayloadDef/FpgaBinGridMsgF32Payload.h"
#include "msgPayloadDef/FpgaRowColSumMsgF32Payload.h"
#include "msgPayloadDef/RegionsIdentifiedMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include "regionsOfInterestPruneAlgorithm.h"

#include <array>
#include <memory>

/*! @brief adapter for the regions-of-interest pruning module.
 *
 *  Reads FpgaRowColSumMsgF32Payload, delegates computation to RegionsOfInterestPruneAlgorithm,
 *  and publishes up to MAX_NUMBER_REGIONS candidates sorted by estimated above-threshold
 *  pixel count to regionsIdentifiedOutMsg (RegionsIdentifiedMsgF32Payload).
 */
class RegionsOfInterestPrune : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;

    // Phase 1: Public config properties -- set before reset().
    uint32_t x0 = DEFAULT_X0;  //!< [px] Reference point column, in [0, MAX_IMAGE_COLS]
    uint32_t y0 = DEFAULT_Y0;  //!< [px] Reference point row, in [0, MAX_IMAGE_ROWS]
    float alphaAreaPruneScaling = DEFAULT_ALPHA_AREA_PRUNE_SCALING;  //!< [-] <= 2; negative bypasses the cost

    // --- Message interfaces ---
    ReadFunctor<FpgaRowColSumMsgF32Payload> rowColSumInMsg;           //!< Row/col accumulators from fpgaImagePipeline
    ReadFunctor<FpgaBinGridMsgF32Payload> binGridInMsg;               //!< ROI bin grid from fpgaImagePipeline
    Message<RegionsIdentifiedMsgF32Payload> regionsIdentifiedOutMsg;  //!< Pruned candidates as RegionsIdentifiedMsg

   private:
    RegionsOfInterestPruneConfig toConfig() const;  //!< Single source of truth for reset() + reconfigure()
    std::unique_ptr<RegionsOfInterestPruneAlgorithm> algorithm = nullptr;
    //!< Lit count per tile copied out of binGridInMsg each update, so the algorithm sees them contiguously.
    //!< A member rather than a local keeps the 12 KB off the stack.
    std::array<uint32_t, FPGA_BIN_GRID_MAX_REGIONS> litCounts{};
};

#endif
