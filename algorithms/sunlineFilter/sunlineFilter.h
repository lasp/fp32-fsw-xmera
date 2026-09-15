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
    SunlineFilter();
    ~SunlineFilter() override;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void reInitializeExceptPersistentStates();
    void reInitialize();

    // Phase 1: public config properties -- set before reset(). The matrix/vector
    // properties are sized to their defaults (zero / identity) in the constructor.
    double alpha = 0.0;                    //!< [-] sigma-point spread tunable
    double beta = 0.0;                     //!< [-] prior-knowledge tunable
    Eigen::MatrixXd processNoise;          //!< [-] N x N process noise Q (defaults to zero)
    Eigen::VectorXd initialState;          //!< [-] N-element initial state seed (defaults to zero)
    Eigen::MatrixXd initialCovariance;     //!< [-] N x N initial covariance P0 (defaults to identity)
    double biasLowerBound = 0.5;           //!< [-] lower clamp on the CSS bias state (> 0)
    double biasUpperBound = 1.5;           //!< [-] upper clamp on the CSS bias state (> 0)
    double sensorThreshold = 0.0;          //!< [-] minimum cosValue to count a sensor active (>= 0)
    double cssMeasurementNoiseStd = 0.0;   //!< [-] CSS measurement noise std (>= 0)
    double gyroMeasurementNoiseStd = 0.0;  //!< [rad/s] gyro measurement noise std (>= 0)

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
