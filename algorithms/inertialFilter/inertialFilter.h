#ifndef F32XMERA_INERTIALFILTER_H
#define F32XMERA_INERTIALFILTER_H

#include "inertialFilterAlgorithm.h"
#include "msgPayloadDef/FilterMsgF32Payload.h"
#include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/STAttMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

/*! @brief xmera adapter for the inertial SRuKF. Pack star-tracker attitude and rate
 *  messages into the algorithm's input types, run update(), and write the output data to messages. */
class InertialFilter : public SysModel {
   public:
    InertialFilter();
    ~InertialFilter() override;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void reInitializeExceptPersistentStates();
    void reInitialize();

    // Public config properties -- set before reset(). The matrix/vector
    // properties are sized to their defaults (zero / identity) in the constructor.
    float alpha = 0.0F;                    //!< [-] sigma-point spread tunable
    float beta = 0.0F;                     //!< [-] prior-knowledge tunable
    Eigen::MatrixXf processNoise;          //!< [-] N x N process noise Q (defaults to zero)
    Eigen::VectorXf initialState;          //!< [-] N-element initial state seed (defaults to zero)
    Eigen::MatrixXf initialCovariance;     //!< [-] N x N initial covariance P0 (defaults to identity)
    float stMeasurementNoiseStd = 0.0F;    //!< [-] star-tracker attitude measurement noise std (>= 0)
    float rateMeasurementNoiseStd = 0.0F;  //!< [rad/s] rate measurement noise std (>= 0)

    ReadFunctor<STAttMsgF32Payload> stAttInMsg;  //!< star-tracker attitude and rate input (required)

    Message<NavAttMsgF32Payload> navAttOutMsg;                  //!< estimated attitude + rate output
    Message<FilterMsgF32Payload> filterOutMsg;                  //!< full filter state + covariance output
    Message<FilterResidualsMsgF32Payload> filterStResOutMsg;    //!< star-tracker residuals output
    Message<FilterResidualsMsgF32Payload> filterRateResOutMsg;  //!< rate residuals output

   private:
    void writeOutputMessages(uint64_t currentSimNanos,
                             filtering::inertialFilter::InertialFilterOutput const& filterOutput);

    std::unique_ptr<filtering::inertialFilter::InertialFilterAlgorithm> algorithm = nullptr;

    double lastStTimeTag = -1;  //!< [s] last ST payload timeTag consumed; -1 before reset
};

#endif
