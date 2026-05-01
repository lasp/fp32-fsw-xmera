// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERAF32_MRP_PD_H
#define XMERAF32_MRP_PD_H

#include <stdint.h>

#include <Eigen/Dense>

#include "mrpPDAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

class MrpPD : public SysModel {
   public:
    MrpPD() = default;
    ~MrpPD() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void setK(float K);
    float getK() const;
    void setP(float P);
    float getP() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B);
    const Eigen::Vector3f& getKnownTorquePntB_B() const;

    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< Commanded torque output message

   private:
    void rebuildAlgorithmConfig();

    float K = 0.0F;
    float P = 0.0F;
    Eigen::Vector3f knownTorquePntB_B = Eigen::Vector3f::Zero();
    Eigen::Matrix3f spacecraftInertia = Eigen::Matrix3f::Identity();

    MrpPDAlgorithm algorithm{MrpPDConfig::create(0.0F, 0.0F, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity())};
};

#endif
