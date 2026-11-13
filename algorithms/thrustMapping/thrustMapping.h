#ifndef F32XMERA_THRUST_MAPPING_H
#define F32XMERA_THRUST_MAPPING_H

#include "msgPayloadDef/CmdForceBodyMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/THRArrayAvailabilityMsgF32Payload.h"
#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "thrustMappingAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <array>
#include <memory>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ThrustMapping final : public SysModel {
   public:
    ThrustMapping() = default;
    ~ThrustMapping() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure();

    void setDesiredControlAxes(const std::array<bool, 6>& desiredControlAxes);
    std::array<bool, 6> getDesiredControlAxes() const;

    /* declare module IO interfaces */
    ReadFunctor<CmdTorqueBodyMsgF32Payload> cmdTorqueInMsg;   //!< (optional) vehicle control (Lr) input message
    ReadFunctor<CmdForceBodyMsgF32Payload> cmdForceInMsg;     //!< (optional) vehicle control force input message
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfigInMsg;  //!< thruster cluster configuration input message
    //! (optional) thruster availability input message; every thruster is available when it is not connected
    ReadFunctor<THRArrayAvailabilityMsgF32Payload> thrAvailInMsg;
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;    //!< vehicle config input message
    Message<THRArrayCmdForceMsgF32Payload> thrForceCmdOutMsg;  //!< thruster force command output message

   private:
    ThrustMappingConfig toConfig();
    ThrusterArrayConfiguration toThrusterArrayConfiguration(const THRArrayConfigMsgF32Payload& thrConfigIn);
    std::unique_ptr<ThrustMappingAlgorithm> algorithm = nullptr;
    //! the axes the mapping controls (torque xyz then force xyz, all in body frame B)
    std::array<bool, 6> desiredControlAxes_B{true, true, true, true, true, true};
};

#endif  // F32XMERA_THRUST_MAPPING_H
