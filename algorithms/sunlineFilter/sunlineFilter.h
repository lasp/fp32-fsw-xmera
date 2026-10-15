#ifndef F32XMERA_SUNLINEFILTER_H
#define F32XMERA_SUNLINEFILTER_H

#include "sunlineFilterAlgorithm.h"

#include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
#include "msgPayloadDef/CSSConfigMsgF32Payload.h"
#include "msgPayloadDef/CSSUnitConfigMsgF32Payload.h"
#include "msgPayloadDef/FilterMsgF32Payload.h"
#include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

/*! @brief xmera adapter for the sunline SRuKF. Pack CSS and gyro
 *  messages into the algorithm's input types, run update(), and write the output data to messages. */
class SunlineFilter : public SysModel {
   public:
    using StateMatrix = filtering::sunlineFilter::StateMatrix;  //!< N x N filter matrix, N the filter state size
    using StateColumn = filtering::sunlineFilter::StateColumn;  //!< N-element filter column

    SunlineFilter();
    ~SunlineFilter() override;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void reInitializeExceptPersistentStates();
    void reInitialize();

    // Phase 1: public config properties -- set before reset(). The matrix/vector
    // properties have the filter's fixed size and default to zero / identity.
    float alpha = 0.0F;                    //!< [-] sigma-point spread tunable
    float beta = 0.0F;                     //!< [-] prior-knowledge tunable
    StateMatrix processNoise;              //!< [-] N x N process noise Q (defaults to zero)
    StateColumn initialState;              //!< [-] N-element initial state seed (defaults to zero)
    StateMatrix initialCovariance;         //!< [-] N x N initial covariance P0 (defaults to identity)
    float biasLowerBound = 0.5F;           //!< [-] lower clamp on the CSS bias state (> 0)
    float biasUpperBound = 1.5F;           //!< [-] upper clamp on the CSS bias state (> 0)
    float sensorThreshold = 0.0F;          //!< [-] minimum cosValue to count a sensor active (>= 0)
    float cssMeasurementNoiseStd = 0.0F;   //!< [-] CSS measurement noise std (>= 0)
    float gyroMeasurementNoiseStd = 0.0F;  //!< [rad/s] gyro measurement noise std (>= 0)
    float measurementDelay = 0.0F;         //!< [s] processing delay for late measurements (>= 0, default 0)

    ReadFunctor<NavAttMsgF32Payload> navAttInMsg;           //!< gyro rate input
    ReadFunctor<CSSArraySensorMsgF32Payload> cssDataInMsg;  //!< CSS array reading input
    ReadFunctor<CSSConfigMsgF32Payload> cssConfigInMsg;     //!< CSS geometry config input (read at reset)

    Message<NavAttMsgF32Payload> navAttOutMsg;                  //!< sun-pointing vector output
    Message<FilterMsgF32Payload> filterOutMsg;                  //!< full filter state + covariance output
    Message<FilterResidualsMsgF32Payload> filterGyroResOutMsg;  //!< gyro residuals output
    Message<FilterResidualsMsgF32Payload> filterCssResOutMsg;   //!< CSS residuals output

   private:
    void writeOutputMessages(uint64_t currentSimNanos,
                             filtering::sunlineFilter::SunlineFilterOutput const& filterOutput);

    std::unique_ptr<filtering::sunlineFilter::SunlineFilterAlgorithm> algorithm = nullptr;

    uint32_t numberOfCss = 0;       //!< [-] CSS count latched from cssConfigInMsg at reset()
    double lastNavAttTimeTag = -1;  //!< [s] last NavAtt payload timeTag consumed; -1
    double lastCssTimeTag = -1;     //!< [s] last CSS payload timeTag consumed; -1
};

#endif
