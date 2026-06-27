#ifndef F32XMERA_INERTIALSRUKF_H
#define F32XMERA_INERTIALSRUKF_H

#include "inertialSRuKFAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AccDataMsgPayload.h>
#include <architecture/msgPayloadDef/AccPktDataMsgPayload.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/STAttMsgPayload.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

/*! @brief xmera adapter for the inertial SRuKF. Pack star-tracker attitude and gyro
 *  messages into the algorithm's input types, run update(), and write the output data to messages. */
class InertialSRuKF : public SysModel {
   public:
    InertialSRuKF();
    ~InertialSRuKF() override;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void reInitialize();
    void reInitializeAll();

    // Public config properties -- set before reset(). The matrix/vector
    // properties are sized to their defaults (zero / identity) in the constructor.
    double alpha = 0.0;                    //!< [-] sigma-point spread tunable
    double beta = 0.0;                     //!< [-] prior-knowledge tunable
    Eigen::MatrixXd processNoise;          //!< [-] N x N process noise Q (defaults to zero)
    Eigen::VectorXd initialState;          //!< [-] N-element initial state seed (defaults to zero)
    Eigen::MatrixXd initialCovariance;     //!< [-] N x N initial covariance P0 (defaults to identity)
    double stMeasurementNoiseStd = 0.0;    //!< [-] star-tracker attitude measurement noise std (>= 0)
    double gyroMeasurementNoiseStd = 0.0;  //!< [rad/s] gyro measurement noise std (>= 0)

    ReadFunctor<STAttMsgPayload> stAttInMsg;      //!< star-tracker attitude input (required)
    ReadFunctor<AccDataMsgPayload> gyrBuffInMsg;  //!< gyro buffer input (optional)

    Message<NavAttMsgPayload> navAttOutMsg;                  //!< estimated attitude + rate output
    Message<FilterMsgPayload> filterOutMsg;                  //!< full filter state + covariance output
    Message<FilterResidualsMsgPayload> filterStResOutMsg;    //!< star-tracker residuals output
    Message<FilterResidualsMsgPayload> filterGyroResOutMsg;  //!< gyro residuals output

   private:
    void writeOutputMessages(uint64_t currentSimNanos,
                             filtering::inertialSRuKF::InertialSRuKFOutput const& filterOutput);

    std::unique_ptr<filtering::inertialSRuKF::InertialSRuKFAlgorithm> algorithm = nullptr;

    double lastStTimeTag = -1;  //!< [s] last ST payload timeTag consumed; -1 before reset
};

#endif
