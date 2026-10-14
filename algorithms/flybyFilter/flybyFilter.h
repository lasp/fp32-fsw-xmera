#ifndef F32XMERA_FLYBYFILTER_H
#define F32XMERA_FLYBYFILTER_H

#include "flybyFilterAlgorithm.h"

#include "msgPayloadDef/FilterMsgF32Payload.h"
#include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "msgPayloadDef/OpNavUnitVecMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

/*! @brief xmera adapter for the angles-only flyby navigation SRuKF. Owns the message ports, converts
 *  payloads to/from the algorithm's Eigen types (SI on the wire, km/(km/s) internally), and drives the
 *  two-phase initialization lifecycle. */
class FlybyFilter : public SysModel {
   public:
    FlybyFilter();
    ~FlybyFilter() override;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void reInitializeExceptPersistentStates();
    void reInitialize();

    // ---- Configuration (public, SI units; set by the host before reset()) ----
    float alpha = 0.0F;                       //!< [-]       sigma-point spread tunable
    float beta = 0.0F;                        //!< [-]       prior-knowledge tunable
    float mu = 0.0F;                          //!< [m^3/s^2] central-body gravitational parameter (SI)
    float unitConversion = 1E-3F;             //!< [-]       SI -> internal length scale (1e-3 = m -> km)
    float headingMeasurementNoiseStd = 0.0F;  //!< [-]       heading (unit-vector) measurement noise std
    Eigen::MatrixXf processNoise;             //!< [(m/s^2)^2] N x N process noise covariance Q (SI)
    Eigen::VectorXf initialState;             //!< [m, m/s]  N-element initial state seed (SI)
    Eigen::MatrixXf initialCovariance;        //!< [m^2, (m/s)^2] N x N initial covariance P0 (SI)

    // ---- Message ports ----
    ReadFunctor<OpNavUnitVecMsgF32Payload> opNavHeadingMsg;  //!< optical-nav heading input (required)
    Message<NavTransMsgF32Payload> navTransOutMsg;           //!< estimated position + velocity output
    Message<FilterMsgF32Payload> filterOutMsg;               //!< full filter state + covariance output
    Message<FilterResidualsMsgF32Payload> filterResOutMsg;   //!< heading pre/post-fit residuals output

   private:
    void writeOutputMessages(uint64_t currentSimNanos, filtering::flybyFilter::FlybyFilterOutput const& filterOutput);

    std::unique_ptr<filtering::flybyFilter::FlybyFilterAlgorithm> algorithm;
    double lastHeadingTimeTag = 0;  //!< [s] time tag of the last accepted heading, for freshness gating
};

#endif
