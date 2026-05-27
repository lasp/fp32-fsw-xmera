#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_H

#include "forceTorqueThrForceMappingAlgorithm.h"
#include "msgPayloadDef/CmdForceBodyMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <array>
#include <memory>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMapping : public SysModel {
   public:
    ForceTorqueThrForceMapping() = default;
    ~ForceTorqueThrForceMapping() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setDesiredControlAxes(const std::array<bool, 6>& desiredControlAxes);
    std::array<bool, 6> getDesiredControlAxes() const;

    /* declare module IO interfaces */
    ReadFunctor<CmdTorqueBodyMsgF32Payload> cmdTorqueInMsg;    //!< (optional) vehicle control (Lr) input message
    ReadFunctor<CmdForceBodyMsgF32Payload> cmdForceInMsg;      //!< (optional) vehicle control force input message
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfigInMsg;   //!< thruster cluster configuration input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;    //!< vehicle config input message
    Message<THRArrayCmdForceMsgF32Payload> thrForceCmdOutMsg;  //!< thruster force command output message

   private:
    std::unique_ptr<ForceTorqueThrForceMappingAlgorithm> algorithm = nullptr;
    //! per-axis controllability assertions (torque xyz then force xyz, all in body frame B)
    std::array<bool, 6> desiredControlAxes_B{true, true, true, true, true, true};
};

#endif  // F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_H
