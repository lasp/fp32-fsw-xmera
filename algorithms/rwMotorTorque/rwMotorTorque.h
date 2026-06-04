#ifndef F32XMERA_RW_MOTOR_TORQUE_H
#define F32XMERA_RW_MOTOR_TORQUE_H

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
#include "rwMotorTorqueAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <Eigen/Core>
#include <cstdint>
#include <memory>

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorque : public SysModel {
   public:
    RwMotorTorque() = default;
    ~RwMotorTorque() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Zero()};  //!< [-] control axes mapping matrix CB
    float omegaGain{};  //!< [-] RW null-space despin feedback gain (>= 0; 0 disables despin)

    /* declare module IO interfaces */
    Message<RwMotorTorqueMsgF32Payload> rwMotorTorqueOutMsg;   //!< RW motor torque output message
    ReadFunctor<CmdTorqueBodyMsgF32Payload> vehControlInMsg;   //!<  vehicle control (Lr) Input message
    ReadFunctor<CmdTorqueBodyMsgF32Payload> vehControlIn2Msg;  //!<  optional vehicle control input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg;     //!<  RW Array input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;        //!< optional RWs availability input message
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;           //!< optional current RW speeds (null-space despin)
    ReadFunctor<RWSpeedMsgF32Payload> rwDesiredSpeedsInMsg;    //!< optional desired RW speeds (null-space despin)

   private:
    std::unique_ptr<RwMotorTorqueAlgorithm> algorithm = nullptr;
};

#endif
