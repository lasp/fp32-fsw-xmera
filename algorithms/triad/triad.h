// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TRIAD_H
#define F32XMERA_TRIAD_H
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "triadAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <Eigen/Core>
#include <stdint.h>
#include <memory>

class Triad final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;
    ReadFunctor<BodyHeadingMsgF32Payload> bodyHeadingInMsg;
    Message<AttRefMsgF32Payload> attRefOutMsg;

    Eigen::Vector3f sadaHat_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::Zero();
    float signOfZHat_N = 1.0F;

   private:
    std::unique_ptr<TriadAlgorithm> algorithm = nullptr;
    TriadConfig toConfig() const;
};

#endif
