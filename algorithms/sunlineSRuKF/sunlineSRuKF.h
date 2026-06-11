#ifndef F32XMERA_SUNLINESRUKF_H
#define F32XMERA_SUNLINESRUKF_H

#include "sunlineSRuKFAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
#include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

/*! @brief xmera adapter for the sunline SRuKF. Pack CSS and gyro
 *  messages into the algorithm's input types, run update(), and write the output data to messages. */
class SunlineSRuKF : public SysModel {
   public:
    SunlineSRuKF();
    ~SunlineSRuKF() override;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
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

    ReadFunctor<NavAttMsgPayload> navAttInMsg;           //!< gyro rate input
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;  //!< CSS array reading input
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;     //!< CSS geometry config input (read at reset)

    Message<NavAttMsgPayload> navAttOutMsg;                  //!< sun-pointing vector output
    Message<FilterMsgPayload> filterOutMsg;                  //!< full filter state + covariance output
    Message<FilterResidualsMsgPayload> filterGyroResOutMsg;  //!< gyro residuals output
    Message<FilterResidualsMsgPayload> filterCssResOutMsg;   //!< CSS residuals output

   private:
    void writeOutputMessages(uint64_t currentSimNanos, filtering::sunlineSRuKF::SunlineSRuKFOutput const& filterOutput);

    std::unique_ptr<filtering::sunlineSRuKF::SunlineSRuKFAlgorithm> algorithm = nullptr;

    int numberOfCss = 0;            //!< [-] CSS count latched from cssConfigInMsg at reset()
    double lastNavAttTimeTag = -1;  //!< [s] last NavAtt payload timeTag consumed; -1
    double lastCssTimeTag = -1;     //!< [s] last CSS payload timeTag consumed; -1
};

#endif
