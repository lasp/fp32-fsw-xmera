// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERAF32_MRP_PD_H
#define XMERAF32_MRP_PD_H

#include <stdint.h>

#include <memory>

#include <Eigen/Core>

#include "mrpPDAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

class MrpPD final : public SysModel {
   public:
    MrpPD() = default;
    ~MrpPD() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    // Phase 1: public config properties -- set before reset().
    float K = 0.0F;                                               //!< [N*m]    proportional gain on MRP error
    float P = 0.0F;                                               //!< [N*m*s]  rate-error feedback gain
    Eigen::Vector3f knownTorquePntB_B = Eigen::Vector3f::Zero();  //!< [N*m]    feedforward known external torque

    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< Commanded torque output message

   private:
    std::unique_ptr<MrpPDAlgorithm> algorithm = nullptr;
};

#endif
