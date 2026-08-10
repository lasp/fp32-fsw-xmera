#ifndef F32XMERA_CSS_WLS_EST_H
#define F32XMERA_CSS_WLS_EST_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
#include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/SunlineFilterMsgPayload.h>

#include <stdint.h>

/*! @brief Top level structure for the CSS weighted least squares estimator.
 Used to estimate the sun state in the vehicle body frame*/
class CssWlsEst : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;  //!< CSS array measurement input message
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;     //!< CSS constellation configuration input message
    Message<NavAttMsgPayload>
        navStateOutMsg;  //!< Navigation output message carrying the estimated sun heading and body rate
    Message<SunlineFilterMsgPayload> cssWLSFiltResOutMsg;  //!< Post-fit residual and observation count output message

    uint32_t numActiveCss{};          //!< [-] Number of CSS sensors above the use threshold this cycle
    uint32_t useWeights{};            //!< [-] Flag selecting measurement weighting for the least squares fit
    uint32_t priorSignalAvailable{};  //!< [-] Flag indicating a prior sun heading estimate is available
    double dOld[3]{};                 //!< [-] Prior normalized sun heading estimate, body frame
    double sensorUseThresh{};         //!< [-] Cosine threshold at or below which a CSS measurement is discarded
    uint64_t priorTime{};             //!< [ns] Time of the previous update; zero until the first call after a reset
    CSSConfigMsgPayload cssConfigInBuffer{};  //!< CSS constellation configuration latched at reset
    SunlineFilterMsgPayload filtStatus{};     //!< Post-fit residual output message buffer
};

#endif
