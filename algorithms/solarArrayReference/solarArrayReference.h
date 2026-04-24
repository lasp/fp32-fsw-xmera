#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <stdint.h>

#include <Eigen/Core>

enum attitudeFrame { referenceFrame = 0, bodyFrame = 1 };

/*! @brief Top level structure for the sub-module routines. */
class SolarArrayReference : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare these user-defined quantities */
    Eigen::Vector3d a1Hat_B{Eigen::Vector3d::Zero()};  //!< solar array drive axis in body frame coordinates
    Eigen::Vector3d a2Hat_B{Eigen::Vector3d::Zero()};  //!< solar array surface normal at zero rotation
    int attitudeFrame{};  //!< flag = 1: compute theta reference based on current attitude instead of attitude reference

    /* declare these variables for internal computations */
    int count{};           //!< counter variable for finite differences
    uint64_t priorT{};     //!< prior call time for finite differences
    double priorThetaR{};  //!< prior output msg for finite differences

    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgPayload> attNavInMsg;                    //!< input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                    //!< input attitude reference message
    ReadFunctor<HingedRigidBodyMsgPayload> hingedRigidBodyInMsg;  //!< input hinged rigid body message
    Message<HingedRigidBodyMsgPayload>
        hingedRigidBodyRefOutMsg;  //!< output msg containing hinged rigid body target angle and angle rate
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_H
