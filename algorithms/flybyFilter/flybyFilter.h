#ifndef F32XMERA_FLYBYFILTER_H
#define F32XMERA_FLYBYFILTER_H

#include "flybyFilterAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

/*! @brief xmera adapter for the angles-only flyby navigation SRuKF. Owns the message ports, converts
 *  payloads to/from the algorithm's Eigen types (SI on the wire, km/(km/s) internally), and drives the
 *  two-phase initialization lifecycle. */
class FlybyFilter : public SysModel {
   public:
    FlybyFilter();
    ~FlybyFilter();

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void reInitializeExceptPersistentStates();
    void reInitialize();

    // ---- Configuration (public, SI units; set by the host before reset()) ----
    double alpha = 0.0;                       //!< [-]       sigma-point spread tunable
    double beta = 0.0;                        //!< [-]       prior-knowledge tunable
    double mu = 0.0;                          //!< [m^3/s^2] central-body gravitational parameter (SI)
    double unitConversion = 1E-3;             //!< [-]       SI -> internal length scale (1e-3 = m -> km)
    double headingMeasurementNoiseStd = 0.0;  //!< [-]       heading (unit-vector) measurement noise std
    double outlierNSigma = 10.0;              //!< [-]       N-sigma innovation gate for outlier rejection (> 0)
    Eigen::MatrixXd processNoise;             //!< [(m/s^2)^2] N x N process noise covariance Q (SI)
    Eigen::VectorXd initialState;             //!< [m, m/s]  N-element initial state seed (SI)
    Eigen::MatrixXd initialCovariance;        //!< [m^2, (m/s)^2] N x N initial covariance P0 (SI)

    // ---- Message ports ----
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;  //!< optical-nav heading input (required)
    Message<NavTransMsgPayload> navTransOutMsg;           //!< estimated position + velocity output
    Message<FilterMsgPayload> filterOutMsg;               //!< full filter state + covariance output
    Message<FilterResidualsMsgPayload> filterResOutMsg;   //!< heading pre/post-fit residuals output

   private:
    void writeOutputMessages(uint64_t currentSimNanos, filtering::flybyFilter::FlybyFilterOutput const& filterOutput);

    std::unique_ptr<filtering::flybyFilter::FlybyFilterAlgorithm> algorithm;
    double lastHeadingTimeTag = 0;  //!< [s] time tag of the last accepted heading, for freshness gating
};

#endif
