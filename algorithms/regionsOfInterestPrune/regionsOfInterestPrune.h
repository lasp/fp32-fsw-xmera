#ifndef F32XMERA_REGIONS_OF_INTEREST_PRUNE_H
#define F32XMERA_REGIONS_OF_INTEREST_PRUNE_H

#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "msgPayloadDef/FpgaRowColSumMsgF32Payload.h"
#include "msgPayloadDef/FpgaThreshImageMsgF32Payload.h"
#include "msgPayloadDef/RegionOfInterestMsgF32Payload.h"
#include "msgPayloadDef/RegionsIdentifiedMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/macroDefinitions.h>

#include "regionsOfInterestPruneAlgorithm.h"

#include <memory>

/*! @brief Basilisk adapter for the regions-of-interest pruning module.
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
    uint32_t maxRowSpans = DEFAULT_MAX_ROW_SPANS;
    uint32_t maxColSpans = DEFAULT_MAX_COL_SPANS;

    // --- Save configuration ---
    void setSaveImages(bool save) { saveImages = save; }
    bool getSaveImages() const { return saveImages; }
    void setSaveDir(const std::string& dir) { saveDir = dir; }
    std::string getSaveDir() const { return saveDir; }

    // --- Message interfaces ---
    ReadFunctor<FpgaRowColSumMsgF32Payload> rowColSumInMsg;           //!< Row/col accumulators from fpgaImagePipeline
    ReadFunctor<FpgaThreshImageMsgF32Payload> threshImageInMsg;       //!< Optional: threshold image for visualization
    Message<RegionsIdentifiedMsgF32Payload> regionsIdentifiedOutMsg;  //!< Pruned candidates as RegionsIdentifiedMsg

   private:
    // Build the BGR background image from the threshold msg (preferred) or the 1-D sums (fallback).
    cv::Mat buildBackground(const FpgaRowColSumMsgF32Payload& rcMsg);
    // Draw a thick bounding-box + filled center dot + label for one ranked region.
    static void drawRegion(cv::Mat& vis,
                           const RegionOfInterestMsgF32Payload& reg,
                           const cv::Scalar& color,
                           int thickness,
                           const std::string& label);
    void saveVisualization(const FpgaRowColSumMsgF32Payload& rcMsg);

    RegionsOfInterestPruneConfig toConfig() const;  //!< Single source of truth for reset() + reconfigure()
    std::unique_ptr<RegionsOfInterestPruneAlgorithm> algorithm = nullptr;

    uint32_t numPublished{};                             //!< Number of valid entries in lastRegionsOutput
    RegionsIdentifiedMsgF32Payload lastRegionsOutput{};  //!< Published center-coordinate form
    bool saveImages{false};
    std::string saveDir{};
};

#endif
