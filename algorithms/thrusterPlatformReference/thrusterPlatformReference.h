#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_H

#include "thrusterPlatformReferenceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/THRConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <stdint.h>

/*! @brief Adapter for the thruster platform reference algorithm. */
class ThrusterPlatformReference : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*! user-defined configuration quantities */
    Eigen::Vector3d sigma_MB{Eigen::Vector3d::Zero()};  //!< orientation of the M frame w.r.t. the B frame
    Eigen::Vector3d r_BM_M{
        Eigen::Vector3d::Zero()};  //!< position of B frame origin w.r.t. M frame origin, in M frame coordinates
    Eigen::Vector3d r_FM_F{
        Eigen::Vector3d::Zero()};  //!< position of F frame origin w.r.t. M frame origin, in F frame coordinates
    double K{};                    //!< momentum dumping proportional gain [1/s]
    double Ki{};                   //!< momentum dumping integral gain [1]
    double theta1Max{};            //!< absolute bound on tip angle [rad]
    double theta2Max{};            //!< absolute bound on tilt angle [rad]

    /*! module IO interfaces */
    ReadFunctor<VehicleConfigMsgPayload>
        vehConfigInMsg;  //!< input msg vehicle configuration msg (needed for CM location)
    ReadFunctor<THRConfigMsgPayload> thrusterConfigFInMsg;   //!< input thruster configuration msg
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;            //!< input reaction wheel speeds message
    ReadFunctor<RWArrayConfigMsgPayload> rwConfigDataInMsg;  //!< input RWA configuration message
    Message<HingedRigidBodyMsgPayload>
        hingedRigidBodyRef1OutMsg;  //!< output msg containing theta1 reference and thetaDot1 reference
    Message<HingedRigidBodyMsgPayload>
        hingedRigidBodyRef2OutMsg;  //!< output msg containing theta2 reference and thetaDot2 reference
    Message<BodyHeadingMsgPayload>
        bodyHeadingOutMsg;  //!< output msg containing the thrust heading in body frame coordinates
    Message<CmdTorqueBodyMsgPayload>
        thrusterTorqueOutMsg;  //!< output msg containing the opposite of the thruster torque to be compensated by RW's
    Message<THRConfigMsgPayload>
        thrusterConfigBOutMsg;  //!< output msg containing the thruster configuration infor in B-frame

   private:
    ThrusterPlatformReferenceAlgorithm algorithm{};  //!< algorithm instance
};

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_H
