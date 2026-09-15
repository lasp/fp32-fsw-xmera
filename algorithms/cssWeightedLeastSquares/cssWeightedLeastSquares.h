#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_H

#include "cssWeightedLeastSquaresAlgorithm.h"

#include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
#include "msgPayloadDef/CSSConfigMsgF32Payload.h"
#include "msgPayloadDef/FilterMsgF32Payload.h"
#include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Estimates the body-relative sun heading and rate from a coarse sun sensor array. */
class CssWeightedLeastSquares final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure();
    void reInitialize();

    // Phase 1: public config properties -- set before reset()
    bool useWeights{};        //!< [-] flag selecting measurement weighting for the least squares fit
    float sensorUseThresh{};  //!< [-] cosine threshold at or below which a CSS measurement is discarded
    float controlPeriod{};    //!< [s] time between two updateState() calls (must be > 0)

    uint32_t numActiveCss{};  //!< [-] sensors above the use threshold on the most recent cycle (output)

    /* declare module IO interfaces */
    ReadFunctor<CSSArraySensorMsgF32Payload> cssDataInMsg;  //!< CSS array measurement input message
    ReadFunctor<CSSConfigMsgF32Payload> cssConfigInMsg;     //!< CSS geometry config input, read at reset()
    Message<NavAttMsgF32Payload>
        navStateOutMsg;  //!< Navigation output message carrying the estimated sun heading and body rate
    Message<FilterMsgF32Payload> filterOutMsg;  //!< Estimator state output message
    Message<FilterResidualsMsgF32Payload>
        filterCssResOutMsg;  //!< Post-fit residual and observation count output message

   private:
    CssWeightedLeastSquaresConfig toConfig();
    std::unique_ptr<CssWeightedLeastSquaresAlgorithm> algorithm = nullptr;
};

#endif
