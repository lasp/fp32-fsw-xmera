#ifndef F32XMERA_THRUST_VECTORING_H
#define F32XMERA_THRUST_VECTORING_H

#include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/THRConfigMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "thrustVectoringAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>

#include <memory>

/*! @brief Adapter for the thrust vectoring algorithm. */
class ThrustVectoring final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*! Re-push the current configuration properties into the running algorithm without re-seeding its state. */
    void reconfigure();
    /*! Re-seed the running algorithm's runtime integrator state from its configured initial values. */
    void reInitialize();

    /*! Phase 1: user-defined configuration properties, set before reset() */
    Eigen::Vector3f sigma_MB{Eigen::Vector3f::Zero()};  //!< orientation of the M frame w.r.t. the B frame
    Eigen::Vector3f r_MB_B{
        Eigen::Vector3f::Zero()};  //!< position of M frame origin w.r.t. B frame origin, in B frame coordinates
    Eigen::Vector3f r_FM_F{
        Eigen::Vector3f::Zero()};  //!< position of F frame origin w.r.t. M frame origin, in F frame coordinates
    float K{};                     //!< momentum dumping proportional gain [1/s] (must be > 0)
    float Ki{};                    //!< momentum dumping integral gain [1/s2]
    float integralLimit{};         //!< [Nms2] anti-windup clamp on each hsInt_B component (must be > 0 if Ki > 0)
    float controlPeriod{};         //!< integration step for the momentum dumping integral [s] (must be > 0)
    float thetaMax{};              //!< half-angle of the thrust-deflection cone [rad] (must be in (0, pi))

    /*! module IO interfaces */
    ReadFunctor<VehicleConfigMsgF32Payload>
        vehConfigInMsg;  //!< input msg vehicle configuration msg (needed for CM location)
    ReadFunctor<THRConfigMsgF32Payload> thrusterConfigFInMsg;   //!< input thruster configuration msg
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;            //!< input reaction wheel speeds message (required)
    ReadFunctor<RWArrayConfigMsgF32Payload> rwConfigDataInMsg;  //!< input RWA configuration message (required)
    Message<BodyHeadingMsgF32Payload>
        bodyHeadingOutMsg;  //!< output msg containing the thrust heading in body frame coordinates
    Message<THRConfigMsgF32Payload>
        thrusterConfigBOutMsg;  //!< output msg containing the thruster configuration infor in B-frame

   private:
    ThrustVectoringConfig toConfig();
    std::unique_ptr<ThrustVectoringAlgorithm> algorithm = nullptr;  //!< algorithm instance
};

#endif  // F32XMERA_THRUST_VECTORING_H
