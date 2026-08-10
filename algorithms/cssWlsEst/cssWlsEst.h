#ifndef F32XMERA_CSS_WLS_EST_H
#define F32XMERA_CSS_WLS_EST_H

#include "cssWlsEstAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/SunlineFilterMsgPayload.h>

#include <Eigen/Core>

#include <stdint.h>

/*! @brief Estimates the body-relative sun heading and rate from a coarse sun sensor array. */
class CssWlsEst : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*! [-] Per-sensor boresight unit vectors in body frame components, numCss rows by three columns. */
    Eigen::MatrixXd cssNHat;

    /*! [-] Per-sensor calibration scale factor, at least numCss entries. */
    Eigen::VectorXd cssBias;

    /*! [-] Number of configured coarse sun sensors, in [0, kMaxNumCss]. */
    uint32_t numCss{};

    /*! [-] Flag selecting measurement weighting for the least squares fit. */
    uint32_t useWeights{};

    /*! [-] Cosine threshold at or below which a CSS measurement is discarded. */
    double sensorUseThresh{};

    /*! [-] Number of CSS sensors above the use threshold on the most recent cycle. */
    uint32_t numActiveCss{};

    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;  //!< CSS array measurement input message
    Message<NavAttMsgPayload>
        navStateOutMsg;  //!< Navigation output message carrying the estimated sun heading and body rate
    Message<SunlineFilterMsgPayload> cssWLSFiltResOutMsg;  //!< Post-fit residual and observation count output message

   private:
    CssWlsEstAlgorithm algorithm;
};

#endif
