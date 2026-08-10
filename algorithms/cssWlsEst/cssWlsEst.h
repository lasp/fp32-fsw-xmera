#ifndef F32XMERA_CSS_WLS_EST_H
#define F32XMERA_CSS_WLS_EST_H

#include "cssWlsEstAlgorithm.h"

#include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/SunlineFilterMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

#include <stdint.h>
#include <memory>

/*! @brief Estimates the body-relative sun heading and rate from a coarse sun sensor array. */
class CssWlsEst final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure();
    void reInitialize();

    // Phase 1: public config properties -- set before reset()
    /*! [-] Per-sensor boresight unit vectors in body frame components, numCss rows by three columns. */
    Eigen::MatrixXf cssNHat;

    /*! [-] Per-sensor calibration scale factor, at least numCss entries. */
    Eigen::VectorXf cssBias;

    /*! [-] Number of configured coarse sun sensors, in [0, kMaxNumCss]. */
    uint32_t numCss{};

    /*! [-] Flag selecting measurement weighting for the least squares fit. */
    bool useWeights{};

    /*! [-] Cosine threshold at or below which a CSS measurement is discarded. */
    float sensorUseThresh{};

    /*! [-] Number of CSS sensors above the use threshold on the most recent cycle. */
    uint32_t numActiveCss{};

    ReadFunctor<CSSArraySensorMsgF32Payload> cssDataInMsg;  //!< CSS array measurement input message
    Message<NavAttMsgF32Payload>
        navStateOutMsg;  //!< Navigation output message carrying the estimated sun heading and body rate
    Message<SunlineFilterMsgF32Payload>
        cssWLSFiltResOutMsg;  //!< Post-fit residual and observation count output message

   private:
    CssWlsEstConfig toConfig() const;
    std::unique_ptr<CssWlsEstAlgorithm> algorithm = nullptr;
};

#endif
