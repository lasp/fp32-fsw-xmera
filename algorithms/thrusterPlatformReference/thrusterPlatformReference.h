#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_H

#include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/THRConfigMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "thrusterPlatformReferenceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>

/*! @brief Adapter for the thruster platform reference algorithm. */
class ThrusterPlatformReference : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*! user-defined configuration quantities */
    Eigen::Vector3f sigma_MB{Eigen::Vector3f::Zero()};  //!< orientation of the M frame w.r.t. the B frame
    Eigen::Vector3f r_BM_M{
        Eigen::Vector3f::Zero()};  //!< position of B frame origin w.r.t. M frame origin, in M frame coordinates
    Eigen::Vector3f r_FM_F{
        Eigen::Vector3f::Zero()};  //!< position of F frame origin w.r.t. M frame origin, in F frame coordinates
    float K{};                     //!< momentum dumping proportional gain [1/s]
    float Ki{};                    //!< momentum dumping integral gain [1]
    float theta1Max{};             //!< absolute bound on tip angle [rad]
    float theta2Max{};             //!< absolute bound on tilt angle [rad]

    /*! module IO interfaces */
    ReadFunctor<VehicleConfigMsgF32Payload>
        vehConfigInMsg;  //!< input msg vehicle configuration msg (needed for CM location)
    ReadFunctor<THRConfigMsgF32Payload> thrusterConfigFInMsg;   //!< input thruster configuration msg
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;            //!< input reaction wheel speeds message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwConfigDataInMsg;  //!< input RWA configuration message
    Message<HingedRigidBodyMsgF32Payload>
        hingedRigidBodyRef1OutMsg;  //!< output msg containing theta1 reference and thetaDot1 reference
    Message<HingedRigidBodyMsgF32Payload>
        hingedRigidBodyRef2OutMsg;  //!< output msg containing theta2 reference and thetaDot2 reference
    Message<BodyHeadingMsgF32Payload>
        bodyHeadingOutMsg;  //!< output msg containing the thrust heading in body frame coordinates
    Message<CmdTorqueBodyMsgF32Payload>
        thrusterTorqueOutMsg;  //!< output msg containing the opposite of the thruster torque to be compensated by RW's
    Message<THRConfigMsgF32Payload>
        thrusterConfigBOutMsg;  //!< output msg containing the thruster configuration infor in B-frame

   private:
    ThrusterPlatformReferenceAlgorithm algorithm{};  //!< algorithm instance
};

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_H
