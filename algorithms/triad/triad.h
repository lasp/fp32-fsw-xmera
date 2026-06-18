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

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;
    ReadFunctor<BodyHeadingMsgF32Payload> bodyHeadingInMsg;
    Message<AttRefMsgF32Payload> attRefOutMsg;

    // Phase 1: Public config properties — set before reset()
    Eigen::Vector3f a1Hat_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f hHat_N = Eigen::Vector3f::Zero();

   private:
    std::unique_ptr<TriadAlgorithm> algorithm = nullptr;
};

#endif
