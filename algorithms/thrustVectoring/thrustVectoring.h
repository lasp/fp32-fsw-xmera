#ifndef F32XMERA_THRUST_VECTORING_H
#define F32XMERA_THRUST_VECTORING_H

#include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
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

    /*! Re-read the configuration input messages and re-push the current properties into the running algorithm. */
    void reconfigure();

    /*! Phase 1: user-defined configuration properties, set before reset() */
    Eigen::Vector3f sigma_MB{
        Eigen::Vector3f::Zero()};  //!< orientation of the M frame w.r.t. the B frame; M's -z axis is the thrust
    Eigen::Vector3f r_MB_B{
        Eigen::Vector3f::Zero()};  //!< position of M frame origin w.r.t. B frame origin, in B frame coordinates
    float armLength{};             //!< distance from the joint M to the thruster along the thrust [m] (>= 0)
    float thetaMax{};              //!< half-angle of the thrust-deflection cone [rad] (must be in (0, pi))

    /*! module IO interfaces */
    ReadFunctor<VehicleConfigMsgF32Payload>
        vehConfigInMsg;  //!< input msg vehicle configuration msg (needed for CM location)
    ReadFunctor<THRConfigMsgF32Payload> thrusterConfigFInMsg;  //!< input thruster configuration msg
    ReadFunctor<CmdTorqueBodyMsgF32Payload>
        cmdTorqueInMsg;  //!< [Nm] input requested thruster torque about the center of mass, body frame
    Message<BodyHeadingMsgF32Payload>
        bodyHeadingOutMsg;  //!< output msg containing the thrust heading in body frame coordinates
    Message<THRConfigMsgF32Payload>
        thrusterConfigBOutMsg;  //!< output msg containing the thruster configuration infor in B-frame

   private:
    ThrustVectoringConfig toConfig();
    std::unique_ptr<ThrustVectoringAlgorithm> algorithm = nullptr;  //!< algorithm instance
};

#endif  // F32XMERA_THRUST_VECTORING_H
